"""external_pc controller."""

# sys hash to fix python's broken modules
import sys
sys.path.append("..")

from collections import namedtuple

from controller import Robot
from common.communication import Radio
from common import protocol, util
from mapping import MappingController
from pathfinding import PathfindingController

import numpy as np

ROBOT_TAKEOVER_DISTANCE = 0.2

TIME_STEP = 1
APPROXIMATE_POSITION = 0.1  # 10cm
NO_TURNING_BACK_REGION = 0.5  # 50cm
RobotState = namedtuple("RobotState", ["position", "bearing", "holding_block"])


class ExternalController:
    def __init__(self, emitter_channel, receiver_channel, polling_time=1):
        self.robot = Robot()

        # Current robot positions and status, this is a RobotState object
        self.robot_states = {}

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
            # Robot regularly updates controller with its current status
            # Record this and use it to generate the map
            self.robot_states[message.robot_name] = RobotState(
                message.robot_position,
                message.robot_bearing,
                message.holding_block
            )

            self.mapping_controller.update_with_scan_result(
                message.robot_name,
                message.robot_bearing,
                message.arm_angle,
                message.distance_readings
            )

        elif isinstance(message, protocol.BlockScanResult):
            arena_map = self.mapping_controller.get_clear_movement_map()
            robot_name, robot_pos = message.robot_name, message.robot_position
            # print(f"Scan result controller, block color: {message.color}")

            if message.is_moving_block:
                # Block has been successfully picked up, inform the mapping program
                self.mapping_controller.invalid_region(message.block_position)

                # Send the robot direction back to base
                waypoint_path, block_release_pos = \
                    self.pathfinding_controller.get_dropoff_path(
                        arena_map, robot_pos, robot_name
                    )

                message = protocol.WaypointList(robot_name, waypoint_path + [block_release_pos])
                self.radio.send_message(message)
            else:
                # print("Update color reading ", message.color)
                self.mapping_controller.update_with_color_reading(message.block_position, message.color)

                cluster_position = self.mapping_controller.predict_block_locations()

                shortest_path = self.pathfinding_controller.get_nearest_block_path(
                    robot_name, arena_map,
                    cluster_position, robot_pos
                )

                self.robot_targets[message.robot_name] = RobotTarget(shortest_path.goal_pos, True)
                message = protocol.WaypointList(robot_name, shortest_path.waypoint_path + [shortest_path.release_pos])
                self.radio.send_message(message)

        elif isinstance(message, protocol.ReportBlockDropoff):
            self.mapping_controller.add_drop_off_region(message.block_position)
            if self.pathfinding_controller.number_returned[message.robot_name] == 4:
                self.radio.send_message(protocol.KillImmediately(message.robot_name))
        else:
            raise NotImplementedError()

    def process_robot_messages(self):
        """
            Receive all messages currently queued in robot radio.
            Act on the messages one-by-one
        """
        for message in self.radio.get_messages():
            self.process_message(message)

    def choose_action_for_robot(self, robot_name):
        # Navigate towards nearest block
        block_locations = self.mapping_controller.predict_block_locations()

        # Find the closest block of the correct color
        closest_block = (np.inf, np.inf)
        for cluster in block_locations:
            # Check, is known to be the other color
            if cluster.color is not None and cluster.color != robot_name:
                continue

            this_distance = util.get_distance(cluster.coord, self.robot_positions[robot_name])
            if this_distance < util.get_distance(closest_block):
                closest_block = cluster.coord

        # Should the robot fine tune this part itself?
        closest_distance = util.get_distance(closest_block)
        if closest_distance < ROBOT_TAKEOVER_DISTANCE:
            self.radio.send_message(protocol.AskRobotTakeover(robot_name, closest_block))
        else:
            # Send the robot its new target position (TODO prevent collisions)
            self.radio.send_message(protocol.GiveRobotTarget(robot_name, closest_block))

    def tick(self):
        # Check for robot messages, take an necessary actions
        self.process_robot_messages()
        self.mapping_controller.output_to_displays()

        # Each robot is assigned its own task
        for robot_name in self.robot_states:
            self.choose_action_for_robot(robot_name)


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
