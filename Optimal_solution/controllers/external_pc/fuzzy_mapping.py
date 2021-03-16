import numpy as np
import mapping
from common import util


def _ultrasound_pdf(mean, x):
    # Ultrasound gets less accurate with distance, this is gaussian
    sigma = mapping.ULTRASOUND_NOISE
    return (1 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((mean - x) / sigma) ** 2)


def estimate_measured_static_distance(sensor_pos, sensor_facing, obstacles_pos, obstacles_radius):
    """
        Returns the shortest distance the ultrasound sensor could measure if it
        were facing at bearing sensor_facing and had no unknown targets.
    """
    angles = sensor_facing + np.linspace(-mapping.ULTRASOUND_ANGLE, mapping.ULTRASOUND_ANGLE, 30)
    return util.get_static_distance(sensor_pos, angles, obstacles_pos, obstacles_radius)


class UltrasoundMapping:
    """
        Generates and maintains a probability map of the surrounding environment.
        Sensor reading should be continuously streamed into update_with_scan_result
    """

    def __init__(self):
        # For fast numpy computation
        self._x = np.linspace(-util.WORLD_BOUNDS[0], util.WORLD_BOUNDS[0], mapping.MAP_RESULTION[0])
        self._y = np.linspace(-util.WORLD_BOUNDS[1], util.WORLD_BOUNDS[1], mapping.MAP_RESULTION[1])
        self._xx, self._yy = np.meshgrid(self._x, self._y)

        # Contains the probabilities of finding a block in each location
        # The mean probability density function is calculate
        self._probability_mask = np.ones(mapping.MAP_RESULTION)
        self._probability_sum = np.zeros(mapping.MAP_RESULTION)
        self._probability_count = np.ones(mapping.MAP_RESULTION)

    def update_with_scan_result(
            self, sensor_positions, sensor_bearings, sensor_readings, obstacle_positions, obstacle_radii):
        """
            Processes a pair of sensor measurements and updates the internal probability maps based on the reading.
            It is assumed that these sensor lie on either end of the robot's arm.
        """

        for pos, bearing, reading in zip(sensor_positions, sensor_bearings, sensor_readings):
            self.update_with_distance(
                reading, bearing, pos,
                obstacle_positions, obstacle_radii
            )

        # Counteract map degradation
        self._probability_mask = np.clip(self._probability_mask * 1.01, 0, 1)

    def invalid_region(self, position, invalidation_size):
        """
            Invalidates the region surrounding a single block.
            This should be used after the robot changes the environment for an reason.
        """
        # Clear this region of the probability map to allow fresh scan results
        invalid_min = mapping._to_screenspace(np.array(position) - invalidation_size)
        invalid_max = mapping._to_screenspace(np.array(position) + invalidation_size)
        invalid_size = tuple(invalid_max - invalid_min)

        self._probability_sum[invalid_min[0]:invalid_max[0], invalid_min[1]:invalid_max[1]] = np.zeros(invalid_size)
        self._probability_count[invalid_min[0]:invalid_max[0], invalid_min[1]:invalid_max[1]] = np.ones(invalid_size)

    def update_with_distance(self, detected_distance, measurement_angle, scan_position, obstacles_pos, obstacle_radius):
        """
            Processes a single distance measurement: updating the internal probability maps based on the reading.
        """
        if detected_distance < mapping.ULTRASOUND_MINIMUM_READING:
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

        if mapping.ULTRASOUND_NOISE < reading_delta < 3 * mapping.ULTRASOUND_NOISE:
            # In proximity of wall, can't say anything for certain
            # Vote against it with half a vote
            fov_range = np.logical_and(
                bearings < mapping.ULTRASOUND_ANGLE,
                distances > 0
            )
            self._probability_count += np.where(fov_range, 0.5, 0)

        elif detected_distance > mapping.ULTRASOUND_RANGE or reading_delta < mapping.ULTRASOUND_NOISE:
            # No block was detected, with a relatively high certainty
            # Mark exclusion reading on map with 90% certainty
            exclusion_mask = np.logical_and(
                np.logical_and(
                    distances < min(distance_from_wall - mapping.ULTRASOUND_NOISE, 0.8 * mapping.ULTRASOUND_RANGE),
                    bearings < 0.8 * mapping.ULTRASOUND_ANGLE
                ),
                distances > 0
            )

            self._probability_mask = np.where(exclusion_mask, 0.4 * self._probability_mask, self._probability_mask)

        elif detected_distance < mapping.ULTRASOUND_RANGE:
            # Block detected (consider accurate) (ULTRASOUND_RANGE already includes safety factor)
            # Now mark this on a fuzzy probability map

            # Account for potential FOV distortion
            # NOTE: due to this, statistical values are purely relative

            fov_range = np.logical_and(
                bearings < mapping.ULTRASOUND_ANGLE,
                np.logical_and(
                    distances > 0,
                    distances < detected_distance + mapping.ULTRASOUND_NOISE
                )
            )
            probabilities = _ultrasound_pdf(detected_distance, distances)

            self._probability_sum += np.where(fov_range, probabilities, 0)
            self._probability_count += np.where(fov_range, 1, 0)

    def generate_potential_clusters(self):
        """
            Generates a list of clusters based on the intensity_map.
            No additional process is done to verify these locations.
        """
        GRID_SIZE = (
            int(mapping.MAP_RESULTION[0] / 5),
            int(mapping.MAP_RESULTION[1] / 5)
        )

        # Identify all clusters
        intensity_map = np.empty(mapping.MAP_RESULTION)
        occupancy_map = self.get_occupancy_map()
        mean_amplitude = np.quantile(occupancy_map, 0.99)

        # Account for local variation in intensity
        for x in range(0, mapping.MAP_RESULTION[0], 16):
            for y in range(0, mapping.MAP_RESULTION[1], 16):
                map_segment = occupancy_map[x:x + GRID_SIZE[0], y:y + GRID_SIZE[1]]
                peak_amplitude = np.quantile(map_segment, 0.98)
                intensity_map[x:x + GRID_SIZE[0], y:y + GRID_SIZE[1]] = map_segment > peak_amplitude

        cluster_candidates = np.logical_and(
            intensity_map,
            np.logical_and(
                occupancy_map > mean_amplitude,
                occupancy_map > mapping.FORCE_INVAILD_THRESHOLD
            )
        )

        return cluster_candidates

    def get_clear_movement_map(self, confirmed_blocks):
        """
            Returns a boolean array of shape MAP_RESULTION.
            True elements indicate an area is almost certainly safe to enter:
                its been explored and nothing was found.
            NOTE: This does not account for robot width, take care
        """

        clear_movement_map = np.logical_and(
            self.get_explore_status_map(),
            self.get_occupancy_map() < 0.005
        )

        for block_position, _ in confirmed_blocks:
            coord = mapping._to_screenspace(block_position)
            min_coord = coord - mapping.BLOCK_WIDTH // 2
            max_coord = coord + mapping.BLOCK_WIDTH // 2
            clear_movement_map[min_coord[0]: max_coord[0], min_coord[1]: max_coord[1]] = False

        return clear_movement_map

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
