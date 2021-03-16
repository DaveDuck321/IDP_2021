from fuzzy_mapping import UltrasoundMapping
from hard_mapping import ExactMapping

from common import util

import numpy as np


ARM_RADIUS = 0.15  # 150 mm
BLOCK_WIDTH = 12  # In px
BLOCK_OBSTACLE_WIDTH = 0.1
ROBOT_RADIUS = 1.4 * ARM_RADIUS
ULTRASOUND_MINIMUM_READING = 0.1
ULTRASOUND_RANGE = 1.5  # 1.5 m
ULTRASOUND_NOISE = 0.03  # Standard deviation in distance measurement = 3 cm
ULTRASOUND_ANGLE = (27.2 / 360) * np.pi  # 27.2 degrees total FOV
WORLD_RESOLUTION = 50  # px per meter (reduce for faster computation)
MAP_RESULTION = (
    int(2 * WORLD_RESOLUTION * util.WORLD_BOUNDS[0]),
    int(2 * WORLD_RESOLUTION * util.WORLD_BOUNDS[1])
)

# Region what will be invalidated when a block is collected
INVALIDATION_REGION = ROBOT_RADIUS
CLUSTER_FUDGE_DISTANCE = 0.5  # m
CLUSTER_BLOCK_OVERLAP = 0.8
CLUSTER_OVERLAP_THRESHOLD = 0.8
CLUSTER_PROXIMITY_THRESHOLD = 0.1
CLUSTER_THRESHOLD = 4  # Require 10px to recognize block
FORCE_INVAILD_THRESHOLD = 0.5


def _to_screenspace(coord):
    return util.to_screenspace(coord, util.WORLD_BOUNDS, MAP_RESULTION)


def _to_worldspace(coord):
    return util.to_worldspace(coord, util.WORLD_BOUNDS, MAP_RESULTION)


def get_sensor_position(robot_pos, sensor_angle):
    return (
        robot_pos[0] + ARM_RADIUS * np.cos(sensor_angle),
        robot_pos[1] + ARM_RADIUS * np.sin(sensor_angle)
    )


class MappingController:
    def __init__(self, robot_positions_ref, display_explored, display_occupancy):

        self.__robot_positions_ref = robot_positions_ref

        # The diffrent types of mapping
        self.fuzzy_mapping = UltrasoundMapping()
        self.hard_mapping = ExactMapping()

        # Display objects for map debugging
        self._display_explored = display_explored
        self._display_occupancy = display_occupancy
        self.__last_sensor_positions = {}

    def invalid_region(self, position, scale):
        """
            Invalidates the region surrounding a single block.
            This should be used after the robot changes the environment for any reason.
        """
        block_locations = self.predict_block_locations()

        invalidation_region = scale * INVALIDATION_REGION / 2
        self.hard_mapping.invalid_region(block_locations, position, invalidation_region)
        self.fuzzy_mapping.invalid_region(position, invalidation_region)

    def update_with_color_reading(self, block_location, block_color):
        """
            A robot has just read a block color, log this block position to the world map.
        """
        invalidation_region = INVALIDATION_REGION

        # Old map might have just become illegal, fix it now
        self.hard_mapping.invalid_region([], block_location, invalidation_region)
        self.fuzzy_mapping.invalid_region(block_location, invalidation_region)

        self.hard_mapping.update_with_color_reading(block_location, block_color)

    def update_with_scan_result(self, robot_name, robot_bearing, arm_angle, sensor_readings):
        """
            Processes a pair of sensor measurements and updates the internal probability maps based on the reading.
            It is assumed that these sensor lie on either end of the robot's arm.
        """
        robot_position = self.__robot_positions_ref[robot_name].position

        sensor_bearings = (
            util.normalize_radian(robot_bearing - arm_angle),
            util.normalize_radian(robot_bearing - arm_angle + np.pi)
        )
        sensor_positions = (
            get_sensor_position(robot_position, sensor_bearings[0]),
            get_sensor_position(robot_position, sensor_bearings[1])
        )

        other_robot_pos = [
            self.__robot_positions_ref[other_robot].position
            for other_robot in self.__robot_positions_ref
            if other_robot != robot_name]

        other_robot_radii = [ROBOT_RADIUS] * len(other_robot_pos)
        block_radii = [BLOCK_OBSTACLE_WIDTH] * len(self.hard_mapping.dropped_blocks)

        self.fuzzy_mapping.update_with_scan_result(
            sensor_positions, sensor_bearings, sensor_readings,
            other_robot_pos + self.hard_mapping.dropped_blocks,
            other_robot_radii + block_radii)

        # Save this info for the visualization
        self.__last_sensor_positions[robot_name] = sensor_positions

    def get_clear_movement_map(self):
        """
            Returns a boolean array of shape MAP_RESULTION.
            True elements indicate an area is almost certainly safe to enter:
                its been explored and nothing was found.
            NOTE: This does not account for robot width, take care
        """
        return self.fuzzy_mapping.get_clear_movement_map(self.hard_mapping.confirmed_blocks)

    def get_explore_status_map(self):
        """
            Returns a boolean array of shape MAP_RESULTION.
            True elements indicate that this area has already been scanned.
        """
        return self.fuzzy_mapping.get_explore_status_map()

    def get_occupancy_map(self):
        """
            Outputs a probability map where each tile is assigned a value between 0 and 1.
            1 - Certainly contains a object
            0 - Either unexplored or empty
        """

        return self.fuzzy_mapping.get_occupancy_map()

    def add_drop_off_region(self, position):
        """
            The block has been dropped off. Block should be excluded from all future scans.
        """
        self.hard_mapping.add_drop_off_region(position)

    def predict_block_locations(self):
        """
            Returns a list of ClusterLocation objects, containing the positions of blocks
            and the relative certainties.
        """
        occupancy_map = self.fuzzy_mapping.get_occupancy_map()
        cluster_candidates = self.fuzzy_mapping.generate_potential_clusters()
        return self.hard_mapping.predict_block_locations(cluster_candidates, occupancy_map)

    def output_to_displays(self):
        occupancy_map = util.get_intensity_map_pixels(self.get_occupancy_map())

        # Display block locations on this map
        block_locations = self.predict_block_locations()

        # Draw robot and sensor positions to visualization
        for robot_name in self.__robot_positions_ref:
            # Multiple robots
            robot_pos = _to_screenspace(self.__robot_positions_ref[robot_name].position)
            occupancy_map[robot_pos[0], robot_pos[1], :] = (255, 0, 0)

            # Multiple sensors per robot
            for sensor in self.__last_sensor_positions[robot_name]:
                sensor_pos = _to_screenspace(sensor)
                occupancy_map[sensor_pos[0], sensor_pos[1], :] = (0, 255, 0)

        # Output confirmed block positions
        for (block_location, _) in self.hard_mapping.confirmed_blocks:
            coord = _to_screenspace(block_location)
            occupancy_map[coord[0], coord[1], :] = (255, 0, 255)

        for block_location in block_locations:
            coord = _to_screenspace(block_location.coord)
            block_color = util.get_robot_color(block_location.color)

            occupancy_map[coord[0], coord[1], :] = block_color

        # Output to webots display
        util.display_numpy_pixels(self._display_occupancy, occupancy_map)

        # Output movement status map, marking block locations
        movement_status = util.get_intensity_map_pixels(self.get_clear_movement_map())

        # Output confirmed block positions
        for (block_location, _) in self.hard_mapping.confirmed_blocks:
            coord = _to_screenspace(block_location)
            movement_status[coord[0], coord[1], :] = (255, 0, 255)

        for block_location in block_locations:
            coord = _to_screenspace(block_location.coord)
            block_color = util.get_robot_color(block_location.color)

            movement_status[coord[0], coord[1], :] = block_color

        # Output to display
        util.display_numpy_pixels(self._display_explored, movement_status)
