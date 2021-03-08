from controller import Display
from common import util
import numpy as np

ARM_LENGTH = 0.2  # 20 cm
ULTRASOUND_RANGE = 1.3  # 1.3 m
ULTRASOUND_NOISE = 0.03  # Standard deviation in distance measurement = 3 cm
ULTRASOUND_ANGLE = (27.2 / 360) * np.pi  # 27.2 degrees total FOV
WORLD_BOUNDS = np.array([1.2 - 0.01, 1.2 - 0.01])  # UPDATE THIS WITH WORLD FILE
WORLD_RESOLUTION = 100  # px per meter (reduce for faster computation)
MAP_RESULTION = (
    int(2 * WORLD_RESOLUTION * WORLD_BOUNDS[0]),
    int(2 * WORLD_RESOLUTION * WORLD_BOUNDS[1])
)


def _ultrasound_pdf(mean, x):
    # Ultrasound gets less accurate with distance, this is gaussian
    sigma = ULTRASOUND_NOISE
    return (1 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((mean - x) / sigma) ** 2)


def _to_screenspace(coords):
    return np.flip((coords + WORLD_BOUNDS) / (2 * WORLD_BOUNDS) * MAP_RESULTION).astype(int)


def _to_worldspace(_coords):
    coords = np.array([_coords[1], _coords[0]])
    return (2 * (coords / MAP_RESULTION) - 1) * WORLD_BOUNDS


def estimate_measured_static_distance(sensor_pos, sensor_facing, obstacles_pos):
    """
        Returns the shortest distance the ultrasound sensor could measure if it
        were facing at bearing sensor_facing and had no unknown targets.
    """
    angles = sensor_facing + np.linspace(-ULTRASOUND_ANGLE, ULTRASOUND_ANGLE, 30)
    return get_static_distance(sensor_pos, angles, obstacles_pos)


def get_static_distance(sensor_pos, sensor_facing, obstacles_pos):
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

    # Collisions with other robots and static obsticles
    for obstacle_pos in obstacles_pos:
        a = x_dir ** 2 + y_dir ** 2
        b = 2 * x_dir * (sensor_pos[0] - obstacle_pos[0]) + 2 * y_dir * (sensor_pos[1] - obstacle_pos[1])
        c = (sensor_pos[0] - obstacle_pos[0])**2 + (sensor_pos[1] - obstacle_pos[1])**2 - (1.1 * ARM_LENGTH)**2

        # Filter invalid values
        determinant = b**2 - 4 * a * c
        mask = np.logical_and(determinant > 0, a != 0)
        a, b, determinant = a[mask], b[mask], determinant[mask]

        distance_outer = (-b - determinant) / (2 * a)
        distance_inner = (-b + determinant) / (2 * a)
        distances = np.append(distance_outer, distance_inner)
        combined_array = np.append(combined_array, distances)

    return np.amin(combined_array[combined_array > 0])


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

        # Display objects for map debugging
        self._display_explored = display_explored
        self._display_occupancy = display_occupancy

        self.__robot_positions_ref = robot_positions_ref
        self.__last_sensor_positions = {}

    def update_with_scan_result(self, robot_name, robot_bearing, arm_angle, sensor_readings):
        """
            Processes a pair of sensor measurements and updates the internal probability maps based on the reading.
            It is assumed that these sensor lie on either end of the robot's arm.
        """
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

        for pos, bearing, reading in zip(sensor_positions, sensor_bearings, sensor_readings):
            self.update_with_distance(reading, bearing, pos, other_robot_pos)

        # Save this info for the visualization
        self.__last_sensor_positions[robot_name] = sensor_positions

    def update_with_distance(self, detected_distance, measurement_angle, scan_position, obstacles_pos):
        """
            Processes a single distance measurement: updating the internal probability maps based on the reading.
        """
        # Find guaranteed block angle: hlh detected
        pass  # to be implemented

        # Look at sensor readings: is a block in range
        x_vectors = self._xx - scan_position[0]
        y_vectors = self._yy - scan_position[1]
        distances = np.hypot(x_vectors, y_vectors)
        bearings = np.abs(np.arctan2(y_vectors, x_vectors) - measurement_angle)

        # If reading is statistically indistinct (2 * sd) from wall, ignore it: it might be a wall
        distance_from_wall = estimate_measured_static_distance(scan_position, measurement_angle, obstacles_pos)
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
                    distances < distance_from_wall - ULTRASOUND_NOISE,
                    bearings < 0.8 * ULTRASOUND_ANGLE
                ),
                distances > 0
            )

            self._probability_mask = np.where(exclusion_mask, 0.4 * self._probability_mask, self._probability_mask)

        elif detected_distance < ULTRASOUND_RANGE:
            # Block detected (consider accurate) (ULTRASOUND_RANGE already includes safety factor)
            # Now mark this on a fuzzy probability map

            # Account for potential FOV distortion
            # Note: due to this, statistical values are purely relative

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

    def get_clear_movement_map(self):
        """
            Returns a boolean array of shape MAP_RESULTION.
            True elements indicate an area is almost certainly safe to enter:
                its been explored and nothing was found.
            NOTE: This does not account for robot width, take care
        """
        return np.logical_and(
            self.get_explore_status_map(),
            self.get_occupancy_map() < 0.005
        )

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

        # Normalize all probabilities according to the number of votes
        probabilities = self._probability_sum / self._probability_count
        max_probability_intensity = max(np.amax(probabilities), 0.1)

        return self._probability_mask * (probabilities / max_probability_intensity)

    def output_to_displays(self):
        occupancy_map = get_intensity_map_pixels(self.get_occupancy_map())

        # Draw robot and sensor positions to visualization
        for robot_name in self.__robot_positions_ref:
            # Multiple robots
            robot_pos = _to_screenspace(self.__robot_positions_ref[robot_name])
            occupancy_map[robot_pos[0], robot_pos[1], 0] = 255
            occupancy_map[robot_pos[0], robot_pos[1], 1] = 0
            occupancy_map[robot_pos[0], robot_pos[1], 2] = 0

            # Multiple sensors per robot
            for sensor in self.__last_sensor_positions[robot_name]:
                sensor_pos = _to_screenspace(sensor)
                occupancy_map[sensor_pos[0], sensor_pos[1], 1] = 255
                occupancy_map[robot_pos[0], sensor_pos[1], 0] = 0
                occupancy_map[robot_pos[0], sensor_pos[1], 2] = 0

        # Output to webots display
        image = self._display_occupancy.imageNew(
            occupancy_map.tobytes(),
            Display.RGB,
            MAP_RESULTION[0],
            MAP_RESULTION[1]
        )
        self._display_occupancy.imagePaste(image, 0, 0, False)
        self._display_occupancy.imageDelete(image)

        # Output movement status map
        movement_status = self.get_clear_movement_map()
        image = self._display_explored.imageNew(
            get_intensity_map_pixels(movement_status).tobytes(),
            Display.RGB,
            MAP_RESULTION[0],
            MAP_RESULTION[1]
        )
        self._display_explored.imagePaste(image, 0, 0, False)
        self._display_explored.imageDelete(image)
