"""robot_controller controller."""

# sys hash to fix python's broken modules
import sys
sys.path.append("..")

from controller import Robot
from drive_controller import DriveController
from positioning_systems import PositioningSystem
from pincer_controller import PincerController
from ultrasonic_emulator import RealUltrasonic
from scanning import ServoArmController
from infrared_controller import IRSensor
from IR_block_search import BlockSearch
from reverse_away import Reversing
from block_deposit import BlockDeposit

from common.communication import Radio
from common import protocol

from enum import Enum

TIME_STEP = 5  # ms


class Tasks(Enum):
    DEAD = -1
    NONE = 0
    INITIAL_SCAN = 1
    FOLLOWING_CONTROLLER = 2
    SEARCHING_BLOCK = 3
    REVERSING = 4
    DEPOSITING_BLOCK = 5


class RobotController:
    def __init__(self, sensor_polling=5):
        self.robot = Robot()
        self.current_task = Tasks.NONE
        self.queued_task = Tasks.INITIAL_SCAN

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

        self.requested_target = None

        # These algorithms are created/ destroyed as needed
        self.block_searching_algorithm = None
        self.reversing_algorithm = None
        self.depositing_algorithm = None

    def send_new_scan(self):
        message = protocol.ScanDistanceReading(
            self.robot.getName(),
            self.positioning_system.get_2D_position(),
            self.positioning_system.get_world_bearing(),
            self.positioning_system.get_turret_angle(),
            self.positioning_system.get_distance_readings(),
            self.pincer_controller.is_closed,
        )
        self.radio.send_message(message)

    def process_message(self, message):
        """
            Take action on the received message.
        """
        if isinstance(message, protocol.KillImmediately):
            self.queued_task = Tasks.DEAD

        elif isinstance(message, protocol.GiveRobotTarget):
            self.requested_target = message.target

            # Wake the robot if its not doing anything
            if self.current_task == Tasks.NONE:
                self.queued_task = Tasks.FOLLOWING_CONTROLLER

        elif isinstance(message, protocol.AskRobotSearch):
            self.queued_task = Tasks.SEARCHING_BLOCK

        elif isinstance(message, protocol.AskRobotDeposit):
            self.queued_task = Tasks.DEPOSITING_BLOCK

        else:
            raise NotImplementedError(f"Could not process message {message}")

    def process_controller_instructions(self):
        """
            Receive and process all instructions on the current channel.
        """
        for message in self.radio.get_messages():
            # Ensure this is robot the correct recipient
            if message.robot_name == self.robot.getName():
                self.process_message(message)

    def __clean_current_task(self, about_to_end, about_to_start):
        """
            Runs when the current robot task is about to end.
            Note:   'self.current_task' is about_to_end
                    'self.queued_task' is about_to_start
                    DO NOT MODIFY THESE VARIABLES HERE

        """
        if about_to_end == Tasks.SEARCHING_BLOCK:
            self.block_searching_algorithm = None
        if about_to_end == Tasks.REVERSING:
            self.reversing_algorithm = None
        if about_to_end == Tasks.DEPOSITING_BLOCK:
            # Report the exact dropoff position
            self.radio.send_message(protocol.ReportBlockDropoff(
                self.robot.getName(),
                self.depositing_algorithm.get_block_position()
            ))
            self.depositing_algorithm = None

    def __initialize_queued_task(self, about_to_end, about_to_start):
        """
            Runs when the next robot task is about to start.
            Note:   'self.current_task' is about_to_end
                    'self.queued_task' is about_to_start
                    DO NOT MODIFY THESE VARIABLES HERE

        """
        if about_to_start == Tasks.INITIAL_SCAN:
            self.drive_controller.halt()
        if about_to_start == Tasks.DEAD:
            self.drive_controller.halt()
            self.positioning_system.kill_turret()
        if about_to_start == Tasks.SEARCHING_BLOCK:
            self.block_searching_algorithm = BlockSearch(
                self.IR_sensor,
                self.positioning_system,
                self.drive_controller,
                self.pincer_controller
            )
        if about_to_start == Tasks.REVERSING:
            self.reversing_algorithm = Reversing(
                self.drive_controller,
                100  # Reverse for 10 ticks to ensure robot doesn't collide with hidden blocks
            )
        if about_to_start == Tasks.DEPOSITING_BLOCK:
            self.depositing_algorithm = BlockDeposit(
                self.robot.getName(),
                self.positioning_system,
                self.drive_controller,
                self.pincer_controller
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

    def tick(self):
        # A dead robot can never be revived, don't even try to do anything
        if self.current_task == Tasks.DEAD:
            return

        # Get the lastest information from controller
        self.process_controller_instructions()

        # Update the controller
        self.send_new_scan()

        # Execute the current task
        if self.current_task == Tasks.FOLLOWING_CONTROLLER:
            # Ensure robot knows where to go. If not, wait for instructions
            if self.requested_target is None:
                self.queued_task = Tasks.NONE  # Wait for further instructions

            self.drive_controller.navigate_toward_point(self.positioning_system, self.requested_target)

            # Also scan while driving
            self.scanning_controller.driving_scan(self.positioning_system)

        elif self.current_task == Tasks.INITIAL_SCAN:
            if self.scanning_controller.stationary_scan(self.positioning_system):
                # Only blank the robot controller if nothing is planned
                self.queued_task = Tasks.NONE

        elif self.current_task == Tasks.NONE:
            pass  # TODO: maybe query controller here

        elif self.current_task == Tasks.SEARCHING_BLOCK:
            # Algorithm returns True upon completion
            if self.block_searching_algorithm():
                self.queued_task = Tasks.REVERSING

        elif self.current_task == Tasks.REVERSING:
            # Algorithm returns True upon completion
            if self.reversing_algorithm():
                self.queued_task = Tasks.NONE

        elif self.current_task == Tasks.DEPOSITING_BLOCK:
            # Algorithm returns True upon completion
            if self.depositing_algorithm():
                print("[INFO] Block deposited")
                self.queued_task = Tasks.REVERSING

        else:
            print("[ERROR] Unimplemented task type: ", self.current_task)
            raise NotImplementedError()

        # Tick finished, switch tasks if necessary
        self.switch_to_queued_task()


def main():
    fluffy = RobotController()
    while fluffy.robot.step(TIME_STEP) != -1:
        fluffy.tick()


if __name__ == "__main__":
    main()
