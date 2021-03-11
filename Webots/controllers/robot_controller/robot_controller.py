"""robot_controller controller."""

# sys hash to fix python's broken modules
import sys
sys.path.append("..")

from controller import Robot
from drive_controller import DriveController
from positioning_systems import PositioningSystem
from pincer_controller import PincerController
from ultrasonic_emulator import RealUltrasonic
from block_collection import BlockCollection
from scanning import ServoArmController
from infrared_controller import IRSensor

from common.communication import Radio
from common import protocol

from enum import Enum

TIME_STEP = 5  # ms


class Tasks(Enum):
    NONE = 0
    NEGOTIATE_CHANNEL = 1
    STATIONARY_SCAN = 2
    NAVIGATE_TO_WAYPOINT = 3
    GRAB_BLOCK = 4
    BLOCK_COLLECTION = 5


class RobotController:
    def __init__(self, sensor_polling=5):
        self.robot = Robot()
        self.current_task = Tasks.NONE
        self.queued_task = Tasks.STATIONARY_SCAN

        self.robot_color = "green"

        # Setup radio for communication with external controller
        self.radio = Radio(
            self.robot.getDevice("emitter"),
            self.robot.getDevice("receiver"),
            sampling_rate=sensor_polling
        )

        # Setup all positioning sensors
        self.positioning_system = PositioningSystem(
            self.robot.getDevice("turret_motor"),
            self.robot.getDevice("gps"),
            self.robot.getDevice("compass"),
            RealUltrasonic(self.robot, "distance_sensor_front", 21),
            RealUltrasonic(self.robot, "distance_sensor_rear", 21),
            sampling_rate=sensor_polling
        )

        # Setup the pincer
        self.pincer_controller = PincerController(
            self.robot.getDevice("pincer_motor_1"),
            self.robot.getDevice("pincer_motor_2")
        )

        # Setup drive controller for navigation
        self.drive_controller = DriveController(
            self.robot.getDevice("drive_motor_1"),
            self.robot.getDevice("drive_motor_2")
        )

        self.scanning_controller = ServoArmController()

        # Enable light sensor
        self.light = self.robot.getDevice("light_sensor")
        self.light.enable(sensor_polling)

        self.IR_sensor = IRSensor(self.robot.getDevice("IR sensor"), sensor_polling)

        # Start with the pincer fully open
        self.pincer_controller.open_pincer()

        self.get_color_data()

        self.queued_waypoints = None

    def send_new_scan(self):
        message = protocol.ScanDistanceReading(
            self.robot.getName(),
            self.positioning_system.get_2D_position(),
            self.positioning_system.get_world_bearing(),
            self.positioning_system.get_turret_angle(),
            self.positioning_system.get_distance_readings()
        )
        self.radio.send_message(message)

    def get_color_data(self):
        data = self.light.getValue()
        print("[LOG] Color data: ", data)

    def process_message(self, message):
        """
            Take action on the received message.
        """
        if isinstance(message, protocol.WaypointList):
            print("processing waypoint message")
            self.queued_waypoints = message.waypoints
        else:
            raise NotImplementedError()

    def process_controller_instructions(self):
        """
            Receive and process all instructions on the current channel.
        """
        for message in self.radio.get_messages():
            # Ensure this is robot the correct recipient
            print("Received message", message.robot_name)
            if message.robot_name == self.robot.getName():
                self.process_message(message)

    def __clean_current_task(self, about_to_end, about_to_start):
        """
            Runs when the current robot task is about to end.
            Note:   'self.current_task' is about_to_end
                    'self.queued_task' is about_to_start
                    DO NOT MODIFY THESE VARIABLES HERE

        """
        if about_to_end == Tasks.NAVIGATE_TO_WAYPOINT:
            self.drive_controller.halt()
        if about_to_end == Tasks.BLOCK_COLLECTION:
            self.block_collection_controller = None

    def __initialize_queued_task(self, about_to_end, about_to_start):
        """
            Runs when the next robot task is about to start.
            Note:   'self.current_task' is about_to_end
                    'self.queued_task' is about_to_start
                    DO NOT MODIFY THESE VARIABLES HERE

        """
        if about_to_start == Tasks.STATIONARY_SCAN:
            self.drive_controller.halt()
        if about_to_start == Tasks.GRAB_BLOCK:
            self.pincer_controller.close_pincer()
        if about_to_start == Tasks.BLOCK_COLLECTION:
            self.block_collection_controller = BlockCollection(
                self.robot.getName(),
                self.drive_controller,
                self.positioning_system,
                self.pincer_controller,
                self.light, self.radio,
                self.IR_sensor
            )

    def switch_to_queued_task(self):
        """
            Robot will switch to the queued task after completing
            the necessary initialization and cleanup.
        """
        if self.current_task == self.queued_task:
            return  # nothing to do

        self.__initialize_queued_task(self.current_task, self.queued_task)
        self.__clean_current_task(self.current_task, self.queued_task)
        self.current_task = self.queued_task

    def process_queued_waypoints(self):
        if self.drive_controller.waypoints_locked or self.queued_waypoints is None:
            return

        print("pushing waypoints")
        if self.current_task == Tasks.BLOCK_COLLECTION:
            self.drive_controller.set_waypoints(self.queued_waypoints[:-1])
            self.block_collection_controller.set_block_pos(self.queued_waypoints[-1])
        elif self.current_task == Tasks.NAVIGATE_TO_WAYPOINT:
            self.drive_controller.set_waypoints(self.queued_waypoints)

        self.queued_waypoints = None

    def tick(self):
        # Get the lastest information from controller
        self.process_controller_instructions()

        # Check if any received waypoints can be loaded safely into navigate_waypoints
        self.process_queued_waypoints()

        # Update the controller
        self.send_new_scan()

        # Execute the current task
        if self.current_task == Tasks.NAVIGATE_TO_WAYPOINT:
            if self.drive_controller.navigate_waypoints(self.positioning_system, reverse=True):
                # Reached final waypoint! Change to grabbing state
                self.queued_task = Tasks.BLOCK_COLLECTION

        elif self.current_task == Tasks.STATIONARY_SCAN:
            if self.scanning_controller.stationary_scan(self.positioning_system):
                self.queued_task = Tasks.BLOCK_COLLECTION

        elif self.current_task == Tasks.BLOCK_COLLECTION:
            if self.block_collection_controller():
                self.queued_task = Tasks.STATIONARY_SCAN

        elif self.current_task == Tasks.NONE:
            pass  # TODO: maybe query controller here
        else:
            print("[ERROR] Unimplemented task type: ", self.current_task)
            raise NotImplementedError()

        # Demonstrate the mapping for moving turret
        if self.current_task != Tasks.STATIONARY_SCAN:
            self.scanning_controller.driving_scan(self.positioning_system)

        # Tick finished, switch tasks if necessary
        self.switch_to_queued_task()


def main():
    fluffy = RobotController()
    while fluffy.robot.step(TIME_STEP) != -1:
        fluffy.tick()


if __name__ == "__main__":
    main()
