from controller import Display
from common import util
import numpy as np

ARM_LENGTH = 0.2  # 10 cm
ULTRASOUND_RANGE = 1.3  # 1.3 m
ULTRASOUND_NOISE = 0.03  # Standard deviation in distance measurement = 0.3 cm
ULTRASOUND_ANGLE = (27.2 / 360) * np.pi  # 27.2 total degrees FOV
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
    angles = sensor_facing + np.linspace(-ULTRASOUND_ANGLE, ULTRASOUND_ANGLE, 30)
    return get_boarder_distances(sensor_pos, angles)


def get_boarder_distances(sensor_pos, sensor_facing):
    # Top edge
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


class MappingController:
    """
        Generates and maintains a probability map of the surrounding environment.
        Sensor reading should be continuously streamed into update_with_scan_result
    """

    def __init__(self, display):
        # For fast numpy computation
        self._x = np.linspace(-WORLD_BOUNDS[0], WORLD_BOUNDS[0], MAP_RESULTION[0])
        self._y = np.linspace(-WORLD_BOUNDS[1], WORLD_BOUNDS[1], MAP_RESULTION[1])
        self._xx, self._yy = np.meshgrid(self._x, self._y)

        # Contains the probabilities of finding a block in each location
        # The mean probability density function is calculate
        self._probability_mask = 255 * np.ones(MAP_RESULTION)
        self._probability_sum = np.zeros(MAP_RESULTION)
        self._probability_count = np.ones(MAP_RESULTION)

        # Display object for map debugging
        self._display = display
        self._display.setColor(0xFF00FF)
        self.__last_robot_position = (0, 0)
        self.__last_sensor_positions = [(0, 0), (0, 0)]

    def update_with_scan_result(self, robot_position, robot_bearing, arm_angle, sensor_readings):
        """
            Processes a pair of sensor measurements and updates the internal probability maps based on the reading.
            It is assumed that these sensor lie on either end of the robot's arm.
        """
        robot_bearing = util.normalize_radian(robot_bearing)
        arm_angle = util.normalize_radian(arm_angle)

        sensor_bearings = (
            robot_bearing - arm_angle,
            robot_bearing - arm_angle + np.pi
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

    def get_color_probability_map(self):
        """
            Output a representation of the probability map suitable for displaying as an image.
        """
        probabilities = self._probability_sum / self._probability_count
        max_probability_intensity = max(np.amax(probabilities), 0.1)
        output_map = np.empty((*MAP_RESULTION, 3))
        output_map[:, :, 0] = self._probability_mask * (probabilities / max_probability_intensity)
        output_map[:, :, 1] = output_map[:, :, 0]
        output_map[:, :, 2] = output_map[:, :, 0]

        return output_map.astype(np.uint8)

    def output_to_display(self):
        probability_map = self.get_color_probability_map()

        # Draw robot and sensor positions to visualization
        robot_pos = _to_screenspace(self.__last_robot_position)
        probability_map[robot_pos[0], robot_pos[1], 0] = 255
        probability_map[robot_pos[0], robot_pos[1], 1] = 0
        probability_map[robot_pos[0], robot_pos[1], 2] = 0

        for sensor in self.__last_sensor_positions:
            sensor_pos = _to_screenspace(sensor)
            probability_map[sensor_pos[0], sensor_pos[1], 1] = 255
            probability_map[robot_pos[0], sensor_pos[1], 0] = 0
            probability_map[robot_pos[0], sensor_pos[1], 2] = 0

        image = self._display.imageNew(
            probability_map.tobytes(),
            Display.RGB,
            MAP_RESULTION[0],
            MAP_RESULTION[1]
        )
        self._display.imagePaste(image, 0, 0, False)
        self._display.imageDelete(image)
