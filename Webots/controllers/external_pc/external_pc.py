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

        self.pathfinding_controller = PathfindingController(self.robot.getDevice("display_pathfinding"))

    def process_message(self, message):
        if isinstance(message, protocol.ScanDistanceReading):
            self.robot_positions[message.robot_name] = message.robot_position

            self.mapping_controller.update_with_scan_result(
                message.robot_name,
                message.robot_bearing,
                message.arm_angle,
                message.distance_readings
            )
        elif isinstance(message, protocol.AskForBlockPath):
            arena_map = self.mapping_controller.get_clear_movement_map()
            robot_name, robot_pos = message.robot_name, message.robot_position

            cluster_position = self.mapping_controller.predict_block_locations()

            waypoint_path, block_release_pos = \
                self.pathfinding_controller.get_nearest_block_path(
                    robot_name, arena_map,
                    cluster_position, robot_pos
                )

            message = protocol.WaypointList(robot_name, waypoint_path + [block_release_pos])
            self.radio.send_message(message)

        elif isinstance(message, protocol.BlockScanResult):
            arena_map = self.mapping_controller.get_clear_movement_map()
            robot_name, robot_pos = message.robot_name, message.robot_position
            print(f"Scan result controller, block color: {message.color}")

            if message.is_moving_block:
                # Block has been successfully picked up, inform the mapping program
                self.mapping_controller.invalid_region(message.block_position)

                # Send the robot direction back to base
                waypoint_path, block_release_pos = \
                    self.pathfinding_controller.get_dropoff_path(
                        arena_map, robot_pos, robot_name
                    )
            else:
                print("Update color reading ", message.color)
                self.mapping_controller.update_with_color_reading(message.block_position, message.color)

                cluster_position = self.mapping_controller.predict_block_locations()

                waypoint_path, block_release_pos = \
                    self.pathfinding_controller.get_nearest_block_path(
                        robot_name, arena_map,
                        cluster_position, robot_pos
                    )

            message = protocol.WaypointList(robot_name, waypoint_path + [block_release_pos])
            self.radio.send_message(message)
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
