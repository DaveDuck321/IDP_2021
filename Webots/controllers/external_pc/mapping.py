from controller import Display
from common import util

import numpy as np

from functools import reduce
from collections import namedtuple
import itertools

ARM_LENGTH = 0.15  # 150 mm
BLOCK_WIDTH = 6  # In px
BLOCK_OBSTACLE_WIDTH = 0.1
ROBOT_RADIUS = 1.4 * ARM_LENGTH
ULTRASOUND_MINIMUM_READING = 0.1
ULTRASOUND_RANGE = 1.3  # 1.3 m
ULTRASOUND_NOISE = 0.03  # Standard deviation in distance measurement = 3 cm
ULTRASOUND_ANGLE = (27.2 / 360) * np.pi  # 27.2 degrees total FOV
WORLD_BOUNDS = np.array([1.2 - 0.01, 1.2 - 0.01])  # UPDATE THIS WITH WORLD FILE
WORLD_RESOLUTION = 50  # px per meter (reduce for faster computation)
MAP_RESULTION = (
    int(2 * WORLD_RESOLUTION * WORLD_BOUNDS[0]),
    int(2 * WORLD_RESOLUTION * WORLD_BOUNDS[1])
)

# Region what will be invalidated when a block is collected
INVALIDATION_REGION = ROBOT_RADIUS
CLUSTER_BLOCK_OVERLAP = 0.8
CLUSTER_OVERLAP_THRESHOLD = 0.8
CLUSTER_PROXIMITY_THRESHOLD = 0.1
CLUSTER_THRESHOLD = 4  # Require 10px to recognize block
FORCE_INVAILD_THRESHOLD = 0.0


def _ultrasound_pdf(mean, x):
    # Ultrasound gets less accurate with distance, this is gaussian
    sigma = ULTRASOUND_NOISE
    return (1 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((mean - x) / sigma) ** 2)


def _to_screenspace(coords):
    return np.clip(
        np.flip((coords + WORLD_BOUNDS) / (2 * WORLD_BOUNDS) * MAP_RESULTION),
        (0, 0), (MAP_RESULTION[0] - 1, MAP_RESULTION[1] - 1)
    ).astype(int)


def _to_worldspace(_coords):
    coords = np.array([_coords[1], _coords[0]])
    return (2 * (coords / MAP_RESULTION) - 1) * WORLD_BOUNDS


def estimate_measured_static_distance(sensor_pos, sensor_facing, obstacles_pos, obstacles_radius):
    """
        Returns the shortest distance the ultrasound sensor could measure if it
        were facing at bearing sensor_facing and had no unknown targets.
    """
    angles = sensor_facing + np.linspace(-ULTRASOUND_ANGLE, ULTRASOUND_ANGLE, 30)
    return get_static_distance(sensor_pos, angles, obstacles_pos, obstacles_radius)


def get_static_distance(sensor_pos, sensor_facing, obstacles_pos, obstacles_radius):
    """
        Returns the ray distance to the nearest static obstacle.
    """
    # Collisions with walls
    x_dir = np.cos(sensor_facing)
    y_dir = np.sin(sensor_facing)

    dist_right = (WORLD_BOUNDS[0] - sensor_pos[0]) / x_dir
    dist_left = (-WORLD_BOUNDS[0] - sensor_pos[0]) / x_dir

    dist_top = (WORLD_BOUNDS[1] - sensor_pos[1]) / y_dir
    dist_bottom = (-WORLD_BOUNDS[1] - sensor_pos[1]) / y_dir

    combined_array = np.array([dist_right, dist_left, dist_top, dist_bottom])

    # Collisions with other robots and static obstacles
    for obstacle_pos, obstacle_radius in zip(obstacles_pos, obstacles_radius):
        a = x_dir ** 2 + y_dir ** 2
        b = 2 * x_dir * (sensor_pos[0] - obstacle_pos[0]) + 2 * y_dir * (sensor_pos[1] - obstacle_pos[1])
        c = (sensor_pos[0] - obstacle_pos[0])**2 + (sensor_pos[1] - obstacle_pos[1])**2 - obstacle_radius**2

        # Filter invalid values
        determinant = b**2 - 4 * a * c
        mask = np.logical_and(determinant > 0, a != 0)
        a, b, determinant = a[mask], b[mask], determinant[mask]

        distance_outer = (-b - determinant) / (2 * a)
        distance_inner = (-b + determinant) / (2 * a)
        distances = np.append(distance_outer, distance_inner)
        combined_array = np.append(combined_array, distances)

    return np.amin(combined_array[combined_array > 0])


def get_cluster_average(cluster_candidates, weights):
    """
        Groups together True pixels and returns a list of ClusterLocation objects.
        Output is entirely in world coordinates.
    """

    # Err, comment to explain this absolute mess of a line
    intense_coords = dict(zip(zip(*np.where(cluster_candidates)), itertools.repeat(False)))
    strides = np.array([(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)])

    # Almost functional iterator
    def find_cluster_coords(coord):
        # Needs to be a tuple for indexing
        index = tuple(coord)
        if not util.tuple_in_bound(index, MAP_RESULTION) or \
                not cluster_candidates[index] or intense_coords[index]:
            # Block has already been counted or shouldn't be included
            return []

        # Mark this pixel to avoid repeated counting
        intense_coords[index] = True
        return list(reduce(
            lambda old, stride: np.concatenate((old, find_cluster_coords(coord + stride))),
            strides, coord
        ))

    # Now finally, group the clusters
    clusters_coords = []
    for coord in intense_coords:
        if intense_coords[coord] or len(coord) == 0:
            continue  # Already processed
        cluster_coords = np.reshape(find_cluster_coords(np.array(coord)), (-1, 2)).astype(int)
        clusters_coords.append(cluster_coords)

    # Now do some statistics to find centers
    cluster_locations = []  # list of BlockLocation
    for cluster_coords in clusters_coords:
        if len(cluster_coords) < CLUSTER_THRESHOLD:
            continue

        total_weight = 0
        cluster_sum = np.array((0.0, 0.0))
        for coord in cluster_coords:
            total_weight += weights[tuple(coord)]
            cluster_sum += coord * weights[tuple(coord)]

        # Create a cluster representing the average properties of these coordinates
        # Convert everything to worldspace
        average_position = _to_worldspace(cluster_sum / total_weight)

        radius = 0
        for coord in cluster_coords:
            world_coord = _to_worldspace(coord)
            radius = max(radius, util.get_distance(average_position, world_coord))

        cluster_locations.append(ClusterLocation(average_position, len(cluster_coords), total_weight, radius))

    # Merge nearby clusters
    merged_clusters = []
    for cluster in cluster_locations:
        for large_cluster in merged_clusters:
            cluster_distance = util.get_distance(large_cluster.coord, cluster.coord)

            # Test if clusters are close
            if cluster_distance < CLUSTER_OVERLAP_THRESHOLD * (large_cluster.radius + cluster.radius):
                # Clusters overlap, take a weighted average and change the original cluster
                large_cluster.merge_cluster(cluster)
                break
        else:
            # Cluster could not be merged into another cluster, add it to merged_clusters
            merged_clusters.append(cluster)

    return merged_clusters


def get_sensor_position(robot_pos, sensor_angle):
    return (
        robot_pos[0] + ARM_LENGTH * np.cos(sensor_angle),
        robot_pos[1] + ARM_LENGTH * np.sin(sensor_angle)
    )


def get_intensity_map_pixels(intensity_map):
    """
        Outputs a greyscale RGB image representing a brightness intensity map.
    """
    brightness_map = 255 * intensity_map

    output_map = np.empty((*MAP_RESULTION, 3))
    output_map[:, :, 0] = brightness_map
    output_map[:, :, 1] = brightness_map
    output_map[:, :, 2] = brightness_map

    return output_map.astype(np.uint8)


class ClusterLocation:
    """
        Represents the location and statistical certainty of a block reading.
        This can be merged with nearby block locations to remove noise.
    """

    def __init__(self, coord, area, total_weight, radius):
        self.coord = coord
        self.area = area
        self.total_weight = total_weight
        self.radius = radius
        self.color = None
        self.known_index = None

    def assign_known_color(self, known_index, color):
        self.known_index = known_index
        self.color = color

    def merge_cluster(self, other):
        """
            Takes a weighted average of cluster coordinates in order to merge two clusters.
            This causes the cluster to grow in weight and area.
        """
        combined_weight = self.total_weight + other.total_weight
        self.coord = (
            self.total_weight * self.coord + other.total_weight * other.coord
        ) / combined_weight
        self.total_weight = combined_weight
        self.area += other.area


class MappingController:
    """
        Generates and maintains a probability map of the surrounding environment.
        Sensor reading should be continuously streamed into update_with_scan_result
    """

    def __init__(self, robot_positions_ref, display_explored, display_occupancy):
        # For fast numpy computation
        self._x = np.linspace(-WORLD_BOUNDS[0], WORLD_BOUNDS[0], MAP_RESULTION[0])
        self._y = np.linspace(-WORLD_BOUNDS[1], WORLD_BOUNDS[1], MAP_RESULTION[1])
        self._xx, self._yy = np.meshgrid(self._x, self._y)

        # Contains the probabilities of finding a block in each location
        # The mean probability density function is calculate
        self._probability_mask = np.ones(MAP_RESULTION)
        self._probability_sum = np.zeros(MAP_RESULTION)
        self._probability_count = np.ones(MAP_RESULTION)

        self._confirmed_blocks = []  # (coords, color)
        self._dropped_blocks = []

        # Display objects for map debugging
        self._display_explored = display_explored
        self._display_occupancy = display_occupancy

        self.__robot_positions_ref = robot_positions_ref
        self.__last_sensor_positions = {}

        # Cache for performance
        self.invalid_cache()

    def invalid_cache(self):
        """
            Should be run every time more information is learned about the world
        """
        self.__cache_occupancy_map = None
        self.__cache_block_locations = None
        self.__cache_clear_movement_map = None

    def update_with_color_reading(self, block_location, block_color):
        """
            A robot has just read a block color, log this block position to the world map.
        """
        self.invalid_cache()
        self._confirmed_blocks.append((block_location, block_color))

    def update_with_scan_result(self, robot_name, robot_bearing, arm_angle, sensor_readings):
        """
            Processes a pair of sensor measurements and updates the internal probability maps based on the reading.
            It is assumed that these sensor lie on either end of the robot's arm.
        """
        self.invalid_cache()
        robot_position = self.__robot_positions_ref[robot_name]

        sensor_bearings = (
            util.normalize_radian(robot_bearing - arm_angle),
            util.normalize_radian(robot_bearing - arm_angle + np.pi)
        )
        sensor_positions = (
            get_sensor_position(robot_position, sensor_bearings[0]),
            get_sensor_position(robot_position, sensor_bearings[1])
        )

        other_robot_pos = [
            self.__robot_positions_ref[other_robot]
            for other_robot in self.__robot_positions_ref
            if other_robot != robot_name]

        other_robot_radius = [ROBOT_RADIUS] * len(other_robot_pos)
        obstacle_radius = [BLOCK_OBSTACLE_WIDTH] * len(self._dropped_blocks)

        for pos, bearing, reading in zip(sensor_positions, sensor_bearings, sensor_readings):
            self.update_with_distance(
                reading, bearing, pos,
                other_robot_pos + self._dropped_blocks, other_robot_radius + obstacle_radius
            )

        # Save this info for the visualization
        self.__last_sensor_positions[robot_name] = sensor_positions

    def add_drop_off_region(self, position):
        """
            The block has been dropped off. Block should be excluded from all future scans.
        """
        self.invalid_cache()
        self._dropped_blocks.append(position)

    def invalid_region(self, position):
        """
            Invalidates the region surrounding a single block.
            This should be used after the robot changes the environment for an reason.
        """
        self.invalid_cache()
        invalidation_size = INVALIDATION_REGION / 2

        for cluster in self.predict_block_locations():
            if util.get_distance(cluster.coord, position) < cluster.radius:
                invalidation_size = max(invalidation_size, cluster.radius)
                if(cluster.known_index is not None):
                    print("[info] Block in invalidation cluster, removing")
                    self._confirmed_blocks.pop(cluster.known_index)

        # Also delete overlapping blocks if they are not in a cluster
        for index in range(len(self._confirmed_blocks) - 1, 0, -1):
            block_pos, _ = self._confirmed_blocks[index]
            if util.get_distance(block_pos, position) < invalidation_size:
                # Purge this block
                print("[info] Block in invalidation region, removing")
                self._confirmed_blocks.pop(index)

        invalid_min = _to_screenspace(np.array(position) - invalidation_size)
        invalid_max = _to_screenspace(np.array(position) + invalidation_size)
        invalid_size = tuple(invalid_max - invalid_min)

        self._probability_sum[invalid_min[0]:invalid_max[0], invalid_min[1]:invalid_max[1]] = np.zeros(invalid_size)
        self._probability_count[invalid_min[0]:invalid_max[0], invalid_min[1]:invalid_max[1]] = np.ones(invalid_size)

    def update_with_distance(self, detected_distance, measurement_angle, scan_position, obstacles_pos, obstacle_radius):
        """
            Processes a single distance measurement: updating the internal probability maps based on the reading.
        """
        self.invalid_cache()

        if detected_distance < ULTRASOUND_MINIMUM_READING:
            return  # This is probably noise, ignore it

        # Find guaranteed block angle: hlh detected
        pass  # to be implemented

        # Look at sensor readings: is a block in range
        x_vectors = self._xx - scan_position[0]
        y_vectors = self._yy - scan_position[1]
        distances = np.hypot(x_vectors, y_vectors)
        bearings = np.abs(np.arctan2(y_vectors, x_vectors) - measurement_angle)

        # If reading is statistically indistinct (2 * sd) from wall, ignore it: it might be a wall
        distance_from_wall = estimate_measured_static_distance(
            scan_position, measurement_angle, obstacles_pos, obstacle_radius
        )
        reading_delta = distance_from_wall - detected_distance

        if ULTRASOUND_NOISE < reading_delta < 3 * ULTRASOUND_NOISE:
            # In proximity of wall, can't say anything for certain
            # Vote against it with half a vote
            fov_range = np.logical_and(
                bearings < ULTRASOUND_ANGLE,
                distances > 0
            )
            self._probability_count += np.where(fov_range, 0.5, 0)

        elif detected_distance > ULTRASOUND_RANGE or reading_delta < ULTRASOUND_NOISE:
            # No block was detected, with a relatively high certainty
            # Mark exclusion reading on map with 90% certainty
            exclusion_mask = np.logical_and(
                np.logical_and(
                    distances < min(distance_from_wall - ULTRASOUND_NOISE, 0.8 * ULTRASOUND_RANGE),
                    bearings < 0.8 * ULTRASOUND_ANGLE
                ),
                distances > 0
            )

            self._probability_mask = np.where(exclusion_mask, 0.4 * self._probability_mask, self._probability_mask)

        elif detected_distance < ULTRASOUND_RANGE:
            # Block detected (consider accurate) (ULTRASOUND_RANGE already includes safety factor)
            # Now mark this on a fuzzy probability map

            # Account for potential FOV distortion
            # NOTE: due to this, statistical values are purely relative

            fov_range = np.logical_and(
                bearings < ULTRASOUND_ANGLE,
                np.logical_and(
                    distances > 0,
                    distances < detected_distance + ULTRASOUND_NOISE
                )
            )
            probabilities = _ultrasound_pdf(detected_distance, distances)

            self._probability_sum += np.where(fov_range, probabilities, 0)
            self._probability_count += np.where(fov_range, 1, 0)

    def predict_block_locations(self):
        if self.__cache_block_locations is not None:
            return self.__cache_block_locations

        GRID_SIZE = (
            int(MAP_RESULTION[0] / 5),
            int(MAP_RESULTION[1] / 5)
        )

        # Identify all clusters
        intensity_map = np.empty(MAP_RESULTION)
        occupancy_map = self.get_occupancy_map()
        mean_amplitude = np.quantile(occupancy_map, 0.99)

        # Account for local variation in intensity
        for x in range(0, MAP_RESULTION[0], 16):
            for y in range(0, MAP_RESULTION[1], 16):
                map_segment = occupancy_map[x:x + GRID_SIZE[0], y:y + GRID_SIZE[1]]
                peak_amplitude = np.quantile(map_segment, 0.98)
                intensity_map[x:x + GRID_SIZE[0], y:y + GRID_SIZE[1]] = map_segment > peak_amplitude

        cluster_candidates = np.logical_and(
            intensity_map,
            np.logical_and(
                occupancy_map > mean_amplitude,
                occupancy_map > FORCE_INVAILD_THRESHOLD
            )
        )

        # Smooth clusters into single block
        clusters = get_cluster_average(cluster_candidates, occupancy_map)

        # Some block colors are known, mark these in the clusters object
        for index, (block_location, block_color) in enumerate(self._confirmed_blocks):
            # Check if block is in radius of any clusters
            color_consumed = False

            # NOTE: this is a back hackfix
            ClosestCluster = namedtuple("ClosestBlock", ["distance", "cluster"])
            closest_cluster = ClosestCluster(np.inf, None)

            for cluster in clusters:
                cluster_distance = util.get_distance(cluster.coord, block_location)

                if cluster_distance < CLUSTER_PROXIMITY_THRESHOLD:
                    if (cluster.color is not None):
                        print("[Warning] Multiple blocks identified in the same cluster")

                    if color_consumed:
                        print("[Warning] The same blocks has been assigned to multiple clusters")

                    color_consumed = True
                    cluster.assign_known_color(index, block_color)

                if cluster_distance < closest_cluster.distance:
                    closest_cluster = ClosestCluster(
                        cluster_distance,
                        cluster
                    )

            if not color_consumed:
                # Hackfix, no cluster identified properly, assign to closest one
                # print("[Warning] Block color has been identified but does not exist on map")
                # closest_cluster.cluster.assign_known_color(index, block_color)
                new_cluster = ClusterLocation(
                    block_location, 20, 10, BLOCK_OBSTACLE_WIDTH
                )
                new_cluster.assign_known_color(index, block_color)
                clusters.append(new_cluster)

        self.__cache_block_locations = clusters
        return self.__cache_block_locations

    def get_clear_movement_map(self):
        """
            Returns a boolean array of shape MAP_RESULTION.
            True elements indicate an area is almost certainly safe to enter:
                its been explored and nothing was found.
            NOTE: This does not account for robot width, take care
        """
        if self.__cache_clear_movement_map is not None:
            return self.__cache_clear_movement_map

        clear_movement_map = np.logical_and(
            self.get_explore_status_map(),
            self.get_occupancy_map() < 0.005
        )

        for block_position, _ in self._confirmed_blocks:
            coord = _to_screenspace(block_position)
            min_coord = coord - BLOCK_WIDTH // 2
            max_coord = coord + BLOCK_WIDTH // 2
            clear_movement_map[min_coord[0]: max_coord[0], min_coord[1]: max_coord[1]] = False

        self.__cache_clear_movement_map = clear_movement_map
        return self.__cache_clear_movement_map

    def get_explore_status_map(self):
        """
            Returns a boolean array of shape MAP_RESULTION.
            True elements indicate that this area has already been scanned.
        """
        return self._probability_count != 1

    def get_occupancy_map(self):
        """
            Outputs a probability map where each tile is assigned a value between 0 and 1.
            1 - Certainly contains a object
            0 - Either unexplored or empty
        """
        if self.__cache_occupancy_map is not None:
            return self.__cache_occupancy_map

        # Normalize all probabilities according to the number of votes
        probabilities = self._probability_sum / self._probability_count
        max_probability_intensity = max(np.amax(probabilities), 0.1)

        self.__cache_occupancy_map = self._probability_mask * (probabilities / max_probability_intensity)
        return self.__cache_occupancy_map

    def output_to_displays(self):
        self.invalid_cache()
        occupancy_map = get_intensity_map_pixels(self.get_occupancy_map())

        # Draw robot and sensor positions to visualization
        for robot_name in self.__robot_positions_ref:
            # Multiple robots
            robot_pos = _to_screenspace(self.__robot_positions_ref[robot_name])
            occupancy_map[robot_pos[0], robot_pos[1], :] = (255, 0, 0)

            # Multiple sensors per robot
            for sensor in self.__last_sensor_positions[robot_name]:
                sensor_pos = _to_screenspace(sensor)
                occupancy_map[sensor_pos[0], sensor_pos[1], :] = (0, 255, 0)

        # Output to webots display
        image = self._display_occupancy.imageNew(
            occupancy_map.tobytes(),
            Display.RGB,
            MAP_RESULTION[0],
            MAP_RESULTION[1]
        )
        self._display_occupancy.imagePaste(image, 0, 0, False)
        self._display_occupancy.imageDelete(image)

        # Display block locations on this map
        block_locations = self.predict_block_locations()

        # Output movement status map, marking block locations
        movement_status = get_intensity_map_pixels(self.get_clear_movement_map())

        for block_location in block_locations:
            coord = _to_screenspace(block_location.coord)
            if block_location.color is None:
                block_color = (0, 0, 255)
            if block_location.color == "Fluffy":
                block_color = (0, 255, 0)
            if block_location.color == "Small":
                block_color = (255, 0, 0)

            movement_status[coord[0], coord[1], :] = block_color

        # Output confirmed block positions
        for (block_location, _) in self._confirmed_blocks:
            coord = _to_screenspace(block_location)
            movement_status[coord[0], coord[1], :] = (255, 0, 255)

        image = self._display_explored.imageNew(
            movement_status.tobytes(),
            Display.RGB,
            MAP_RESULTION[0],
            MAP_RESULTION[1]
        )
        self._display_explored.imagePaste(image, 0, 0, False)
        self._display_explored.imageDelete(image)
