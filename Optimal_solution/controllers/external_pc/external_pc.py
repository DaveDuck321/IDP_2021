"""external_pc controller."""

# sys hash to fix python's broken modules
import sys
sys.path.append("..")

from collections import namedtuple, defaultdict

from controller import Robot
from common.communication import Radio
from common import protocol, util
from mapping import MappingController
from pathfinding_test import PathfindingController

import numpy as np

ROBOT_TAKEOVER_DISTANCE = 0.2
ROBOT_SPAWN = {
    "Fluffy": (0.0, -0.28),
    "Small": (0.0, 0.28),
}

TIME_STEP = 1
RobotState = namedtuple("RobotState", ["position", "bearing", "holding_block"])


class ExternalController:
    def __init__(self, emitter_channel, receiver_channel, polling_time=1):
        self.robot = Robot()

        # Current robot positions and status, this is a RobotState object
        self.robot_states = {}
        self.robot_dropoffs = defaultdict(int)

        # Controller IO
        self.radio = Radio(
            self.robot.getDevice("emitter"),
            self.robot.getDevice("receiver")
        )
        self.radio.set_emitter_channel(emitter_channel)
        self.radio.set_receiver_channel(receiver_channel)

        # Controller state
        self.mapping_controller = MappingController(
            self.robot_states,  # This is a const reference
            self.robot.getDevice("display_explored"),
            self.robot.getDevice("display_occupancy")
        )

        self.pathfinding = PathfindingController(
            self.robot.getDevice("display_pathfinding"),
        )

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

        elif isinstance(message, protocol.ReportIncorrectColor):
            # The robot failed to pickup block due to its color, report this
            self.mapping_controller.update_with_color_reading(message.block_position, message.color)

        elif isinstance(message, protocol.ReportBlockDropoff):
            self.mapping_controller.add_drop_off_region(message.block_position)
            self.robot_dropoffs += 1

            # Kill robot if it has just dropped off its block
            # The robot is guaranteed to be in the correct place
            if self.robot_dropoffs == 4:
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

    def request_block_collection(self, robot_name):
        """
            Find the closest block of the correct color, ask the robot to drive towards it.
            If the robot is close enough for mapping to be useless, let the robot takeover.
        """

        block_locations = self.mapping_controller.predict_block_locations()
        closest_block = (np.inf, np.inf)
        for cluster in block_locations:
            # Check, is known to be the other color
            if cluster.color is not None and cluster.color != robot_name:
                continue

            this_distance = util.get_distance(
                cluster.coord,
                self.robot_states[robot_name].position
            )
            if this_distance < util.get_distance(closest_block):
                closest_block = tuple(cluster.coord)

        # Should the robot fine tune this part itself?
        closest_distance = util.get_distance(closest_block)
        if closest_distance < ROBOT_TAKEOVER_DISTANCE:
            self.radio.send_message(protocol.AskRobotTakeover(
                robot_name, closest_block
            ))
        else:
            # Send the robot its new target position (TODO prevent collisions)
            self.radio.send_message(protocol.GiveRobotTarget(
                robot_name, closest_block
            ))

    def choose_action_for_robot(self, robot_name):
        if self.robot_states[robot_name].holding_block:
            # Robot should carry block back to base
            # (TODO prevent collisions)
            self.radio.send_message(protocol.GiveRobotTarget(robot_name, ROBOT_SPAWN[robot_name]))
        else:
            # Robot is available, ask it to find a block
            self.request_block_collection(robot_name)

    def tick(self):
        # Check for robot messages, take an necessary actions
        self.process_robot_messages()
        self.pathfinding.update_map(
            self.mapping_controller.get_clear_movement_map()
        )

        # Each robot is assigned its own task
        for robot_name in self.robot_states:
            self.choose_action_for_robot(robot_name)

        # Debug visualizations
        self.mapping_controller.output_to_displays()
        self.pathfinding.output_to_display()


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
