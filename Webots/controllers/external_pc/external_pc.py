"""external_pc controller."""

# sys hash to fix python's broken modules
import sys
sys.path.append("..")

from controller import Robot
from common.communication import Radio
from common import protocol
from mapping import MappingController
import pathfinding

import numpy as np

TIME_STEP = 1


class ExternalController:
    def __init__(self, emitter_channel, receiver_channel, polling_time=1):
        self.robot = Robot()

        # Controller IO

        self.radio = Radio(
            self.robot.getDevice("emitter"),
            self.robot.getDevice("receiver")
        )
        self.radio.set_emitter_channel(emitter_channel)
        self.radio.set_receiver_channel(receiver_channel)

        # Controller state
        self.mapping_controller = MappingController(
            self.robot.getDevice("display")
        )

        self.produce_dummy_path()

    def produce_dummy_path(self):
        arena_map = np.zeros((240, 240), dtype=np.int32)
        arena_map[:20, :] = 1
        arena_map[-20:, :] = 1
        arena_map[:, :20] = 1
        arena_map[:, -20:] = 1
        start = (30, 30)
        goal = (60, 180)

        waypoints = pathfinding.calculate_route(arena_map, start, goal)
        message = protocol.WaypointList(waypoints)

        self.radio.send_message(message)

    def process_message(self, message):
        if isinstance(message, protocol.ScanDistanceReading):
            self.mapping_controller.update_with_scan_result(
                message.robot_position,
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
        self.mapping_controller.output_to_display()


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
