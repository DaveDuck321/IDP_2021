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


def estimate_measured_boarder_distance(sensor_pos, sensor_facing):
    """
        Returns the shortest distance the ultrasound sensor could measure if it
        were facing in this direction with no obstacles.
    """
    angles = sensor_facing + np.linspace(-ULTRASOUND_ANGLE, ULTRASOUND_ANGLE, 30)
    return get_boarder_distances(sensor_pos, angles)


def get_boarder_distances(sensor_pos, sensor_facing):
    """
        Returns the ray distance to the world boarder given a position and bearing.
    """
    x_dir = np.cos(sensor_facing)
    y_dir = np.sin(sensor_facing)

    dist_right = (WORLD_BOUNDS[0] - sensor_pos[0]) / x_dir
    dist_left = (-WORLD_BOUNDS[0] - sensor_pos[0]) / x_dir

    dist_top = (WORLD_BOUNDS[1] - sensor_pos[1]) / y_dir
    dist_bottom = (-WORLD_BOUNDS[1] - sensor_pos[1]) / y_dir

    combined_array = np.array([dist_right, dist_left, dist_top, dist_bottom])
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

    def __init__(self, display_explored, display_occupancy):
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
        self.__last_robot_position = (0, 0)
        self.__last_sensor_positions = [(0, 0), (0, 0)]

    def update_with_scan_result(self, robot_position, robot_bearing, arm_angle, sensor_readings):
        """
            Processes a pair of sensor measurements and updates the internal probability maps based on the reading.
            It is assumed that these sensor lie on either end of the robot's arm.
        """
        sensor_bearings = (
            util.normalize_radian(robot_bearing - arm_angle),
            util.normalize_radian(robot_bearing - arm_angle + np.pi)
        )
        sensor_positions = (
            get_sensor_position(robot_position, sensor_bearings[0]),
            get_sensor_position(robot_position, sensor_bearings[1])
        )
        for pos, bearing, reading in zip(sensor_positions, sensor_bearings, sensor_readings):
            self.update_with_distance(reading, bearing, pos)

        # Save this info for the visualization
        self.__last_robot_position = robot_position
        self.__last_sensor_positions = sensor_positions

    def update_with_distance(self, detected_distance, measurement_angle, scan_position):
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
        distance_from_wall = estimate_measured_boarder_distance(scan_position, measurement_angle)
        reading_delta = abs(detected_distance - distance_from_wall)

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
                    distances < ULTRASOUND_RANGE - 2 * ULTRASOUND_NOISE,
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
                distances > 0
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
        robot_pos = _to_screenspace(self.__last_robot_position)
        occupancy_map[robot_pos[0], robot_pos[1], 0] = 255
        occupancy_map[robot_pos[0], robot_pos[1], 1] = 0
        occupancy_map[robot_pos[0], robot_pos[1], 2] = 0

        for sensor in self.__last_sensor_positions:
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
