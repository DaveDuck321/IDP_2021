"""robot_controller controller."""

# sys hash to fix python's broken modules
import sys
sys.path.append("..")

from controller import Robot
from drive_controller import DriveController
from positioning_systems import PositioningSystem
from pincer_controller import PincerController
from ultrasonic_emulator import RealUltrasonic

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


class RobotController:
    def __init__(self, sensor_polling=5):
        self.robot = Robot()
        self.current_task = Tasks.NONE
        self.queued_task = Tasks.NAVIGATE_TO_WAYPOINT

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

        # Enable light sensor
        self.light = self.robot.getDevice("light_sensor")
        self.light.enable(sensor_polling)

        # Start with the pincer fully open
        self.pincer_controller.open_pincer()

        # self.stationary_scan()
        # self.get_waypoints()
        self.get_color_data()

    def send_new_scan(self):
        message = protocol.ScanDistanceReading(
            self.positioning_system.get_2D_position(),
            self.positioning_system.get_world_bearing(),
            self.positioning_system.get_turret_angle(),
            self.positioning_system.get_distance_readings()
        )
        self.radio.send_message(message)

    def get_color_data(self):
        data = self.light.getValue()
        print("[LOG] Color data: ", data)

    """
    def stationary_scan(self):
        gps_data = self.gps.getValues()
        compass_data = self.compass.getValues()
        bearing = math.atan2(compass_data[0], compass_data[2])

        self.turret_motor.setPosition(0.0)
        self.turret_motor.setVelocity(self.turret_motor.getMaxVelocity())
        NUM_SCANS = 150

        while self.robot.step(self.timestep) != -1:
            for i in range(NUM_SCANS):
                self.turret_motor.setPosition(3.141592 * i/NUM_SCANS)
                self.robot.step(self.timestep)
                self.distance_data(3.141592 * i/NUM_SCANS, bearing, gps_data)
            return
    """

    def process_message(self, message):
        """
            Take action on the received message.
        """
        if isinstance(message, protocol.WaypointList):
            self.drive_controller.set_waypoints(message.waypoints)
        else:
            raise NotImplementedError()

    def process_controller_instructions(self):
        """
            Receive and process all instructions on the current channel.
        """
        for message in self.radio.get_messages():
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

    def tick(self):
        # Get the lastest information from controller
        self.process_controller_instructions()

        # Update the controller
        self.send_new_scan()

        # Execute the current task
        if self.current_task == Tasks.NAVIGATE_TO_WAYPOINT:
            if self.drive_controller.navigate_waypoints(self.positioning_system):
                # Reached final waypoint! Change to grabbing state
                self.queued_task = Tasks.GRAB_BLOCK
        elif self.current_task == Tasks.STATIONARY_SCAN:
            pass
        elif self.current_task == Tasks.NONE:
            pass  # TODO: maybe query controller here
        else:
            print("[ERROR] Unimplemented task type: ", self.current_task)
            raise NotImplementedError()

        # Demonstrate the mapping for moving turret
        self.positioning_system.spin_turret(0.01)

        # Tick finished, switch tasks if necessary
        self.switch_to_queued_task()


def main():
    fluffy = RobotController()
    while fluffy.robot.step(TIME_STEP) != -1:
        fluffy.tick()


if __name__ == "__main__":
    main()
