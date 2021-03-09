"""external_pc controller."""

# sys hash to fix python's broken modules
import sys
sys.path.append("..")

from controller import Robot
from common.communication import Radio
from common import protocol
from mapping import MappingController
from pathfinding import PathfindingController

# import numpy as np

TIME_STEP = 1


class ExternalController:
    def __init__(self, emitter_channel, receiver_channel, polling_time=1):
        self.robot = Robot()

        # Current robot positions
        self.robot_positions = {}

        # Controller IO
        self.radio = Radio(
            self.robot.getDevice("emitter"),
            self.robot.getDevice("receiver")
        )
        self.radio.set_emitter_channel(emitter_channel)
        self.radio.set_receiver_channel(receiver_channel)

        # Controller state
        self.mapping_controller = MappingController(
            self.robot_positions,  # This is a const reference
            self.robot.getDevice("display_explored"),
            self.robot.getDevice("display_occupancy")
        )

        self.pathfinding_controller = PathfindingController()

        # delete me
        self.delay_index = 0

    def produce_dummy_path(self):
        self.delay_index += 1
        if self.delay_index == 1000:
            arena_map = self.mapping_controller.get_clear_movement_map()
            block_positions = ((0.5, 0.45), (0.79, 0.38), (-0.41, -0.76), (0.78, -0.44))
            waypoints, dropoff_pos = self.pathfinding_controller.get_nearest_block_path(
                arena_map, block_positions, self.robot_positions["Small"], "green")

            message = protocol.WaypointList(waypoints)

            self.radio.send_message(message)

    def process_message(self, message):
        if isinstance(message, protocol.ScanDistanceReading):
            self.robot_positions[message.robot_name] = message.robot_position

            self.mapping_controller.update_with_scan_result(
                message.robot_name,
                message.robot_bearing,
                message.arm_angle,
                message.distance_readings
            )
        else:
            raise NotImplementedError()

    def process_robot_messages(self):
        """
            Receive all messages currently queued in robot radio.
            Act on the messages one-by-one
        """
        for message in self.radio.get_messages():
            self.process_message(message)

    def tick(self):
        # Check for robot messages, take an necessary actions
        self.process_robot_messages()
        self.mapping_controller.output_to_displays()
        self.produce_dummy_path()


def main():
    controller = ExternalController(
        emitter_channel=2,
        receiver_channel=1,
        polling_time=TIME_STEP
    )

    while controller.robot.step(TIME_STEP) != -1:
        controller.tick()


if __name__ == "__main__":
    main()
