"""external_pc controller."""

# sys hash to fix python's broken modules
import sys
sys.path.append("..")

from collections import namedtuple, defaultdict

from controller import Robot
from common.communication import Radio
from common import protocol, util
from mapping import MappingController
from pathfinding import PathfindingController


GOAL_TOLERANCE = 0.3
ROBOT_TAKEOVER_DISTANCE = 0.2

TIME_STEP = 20
RobotState = namedtuple("RobotState", ["position", "bearing", "holding_block"])


class CurrentRoute:
    """
        Describes the current route of a robot.
        The route will change once enough votes are received
    """

    def __init__(self):
        self.waypoints = []
        self.votes = -1

    def reset_votes(self):
        self.waypoints = []
        self.votes = -1

    def vote_for_route(self, new_waypoints):
        # Check if routes are identical. This must be right, stick with it
        if new_waypoints == self.waypoints:
            self.votes = min(50, self.votes + 5)
            return

        routes_different = False
        if len(new_waypoints) < len(self.waypoints):
            # Check if the route just got shorter (or its completely different)
            for new_p, old_p in zip(new_waypoints[::-1], self.waypoints[::-1]):
                if GOAL_TOLERANCE > util.get_distance(new_p, old_p):
                    routes_different = True
                    break

            if not routes_different:
                # Route is just the same (but shorter)
                self.waypoints = new_waypoints
                return

        # Routes are certainly different, vote against current route
        self.votes -= 5

        # If the current route is unpopular, change it
        if self.votes < 0:
            self.waypoints = new_waypoints
            self.votes = 50


class ExternalController:
    def __init__(self, emitter_channel, receiver_channel, polling_time=5):
        self.robot = Robot()

        # Current robot positions and status, this is a RobotState object
        self.robot_states = {}
        self.robot_dropoffs = defaultdict(int)

        # Used to ensure robot pathfinding isn't too noisy
        self.robot_paths = defaultdict(CurrentRoute)

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

        elif isinstance(message, protocol.IRReportFailed):
            self.mapping_controller.invalid_region(message.block_position, 0.1)

        elif isinstance(message, protocol.ReportBlockColor):
            print("Block color reported")
            self.robot_paths[message.robot_name].reset_votes()
            if message.color != message.robot_name:
                # The robot failed to pickup block due to its color, report this
                self.mapping_controller.update_with_color_reading(message.block_position, message.color)
            else:
                # The robot was the correct color so is now moving the block
                # Invalidate this old region now that it has changed
                self.mapping_controller.invalid_region(message.block_position, 1)

        elif isinstance(message, protocol.ReportBlockDropoff):
            self.robot_paths[message.robot_name].reset_votes()

            self.mapping_controller.add_drop_off_region(message.block_position)
            self.pathfinding.add_dropoff(message.block_position)
            self.robot_dropoffs[message.robot_name] += 1

            # Kill robot if it has just dropped off its block
            # The robot is guaranteed to be in the correct place
            if self.robot_dropoffs[message.robot_name] == 4:
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
        robot_position = self.robot_states[robot_name].position
        block_locations = self.mapping_controller.predict_block_locations()
        closest_path = self.pathfinding.get_nearest_block_path(
            block_locations, robot_name, self.robot_paths,
            robot_position
        )

        # Check if map was detailed enough to find path
        if not closest_path.success:
            return

        # Should to robot switch over to this new path?
        self.robot_paths[robot_name].vote_for_route(closest_path.waypoints)
        chosen_path = self.robot_paths[robot_name].waypoints

        next_waypoint = chosen_path[0]
        last_waypoint = chosen_path[-1]

        # Should the robot fine tune this part itself?
        target_distance = util.get_distance(robot_position, last_waypoint)
        if target_distance < ROBOT_TAKEOVER_DISTANCE:
            self.radio.send_message(protocol.AskRobotSearch(
                robot_name, next_waypoint
            ))

            return

        # Robot needs better instructions, it cannot do this by itself

        # Sometimes waypoints will flicker and robot will be trapped
        # Smooth neighboring waypoints to minimize this
        if len(chosen_path) > 1:
            next_waypoint = (
                0.5 * (next_waypoint[0] + chosen_path[1][0]),
                0.5 * (next_waypoint[1] + chosen_path[1][1])
            )

        # Send the robot its new target position (TODO prevent collisions)
        self.radio.send_message(protocol.GiveRobotTarget(
            robot_name, next_waypoint
        ))

    def request_block_dropoff(self, robot_name):
        """
            The robot is holding a block, ensure it returns to its spawn position.
            If the robot is close enough for mapping to be useless, let the robot takeover.
        """
        robot_position = self.robot_states[robot_name].position
        closest_path = self.pathfinding.get_robot_path(
            robot_name, self.robot_paths, robot_position, util.ROBOT_SPAWN[robot_name]
        )

        # Check if it was possible to find a path
        if not closest_path.success:
            return

        next_waypoint = closest_path.waypoints[0]
        waypoint_distance = util.get_distance(robot_position, next_waypoint)

        # Should the robot takeover and deposit the block itself?
        if waypoint_distance < ROBOT_TAKEOVER_DISTANCE:
            self.radio.send_message(protocol.AskRobotDeposit(
                robot_name, next_waypoint
            ))
        else:
            # Begin navigating back home
            self.radio.send_message(protocol.GiveRobotTarget(robot_name, next_waypoint))

    def choose_action_for_robot(self, robot_name):
        if self.robot_states[robot_name].holding_block:
            # Robot should carry block back to base
            # (TODO prevent collisions)
            self.request_block_dropoff(robot_name)
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
        block_locations = self.mapping_controller.predict_block_locations()
        self.pathfinding.output_to_display(self.robot_states, block_locations, self.robot_paths)
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
