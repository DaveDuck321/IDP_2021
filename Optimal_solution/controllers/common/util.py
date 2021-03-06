import numpy as np
from controller import Display
from enum import Enum

LOGGING_LEVEL = 3
WORLD_BOUNDS = np.array([1.2 - 0.01, 1.2 - 0.01])  # UPDATE THIS WITH WORLD FILE
ROBOT_SPAWN = {
    "Fluffy": (0.0, -0.28),
    "Small": (0.0, 0.28),
}


class Logging(Enum):
    DEBUG = 0
    INFO = 1
    WARNING = 2
    COMPETITION = 3


def log_msg(log_type, message):
    """
        Log to console, based on current logging level.
    """
    if not isinstance(log_type, Logging):
        raise ValueError("Bad logging level type")

    if log_type.value >= LOGGING_LEVEL:
        print(f"[{log_type}] {message}")


def filter_nans(iterator, default=np.inf):
    """
        If number is NaN, return the default value.
        Else return number.
    """
    return tuple(
        filter_nan(number, default) for number in iterator
    )


def filter_nan(number, default=np.inf):
    """
        If number is NaN, return the default value.
        Else return number.
    """
    if np.isnan(number):
        return default

    return number


def get_distance(p1, p2=(0, 0)):
    """
        Returns the distance between two tuple coordinates.
    """
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def get_bearing(p1, p2):
    """
        Returns the bearing between two tuple coordinates.
    """
    return np.arctan2(p1[1] - p2[1], p1[0] - p2[0])


def tuple_in_bound(p1, bounds):
    """
        Returns true if a numpy array of shape (bounds) could be indexed by the tuple.
    """
    return p1[0] >= 0 and p1[1] >= 0 and p1[0] < bounds[0] and p1[1] < bounds[1]


def normalize_radian(phase):
    """
        Normalizes a phase radian between +/- pi.
    """
    return (phase + np.pi) % (2 * np.pi) - np.pi


def get_robot_color_string(name):
    """
        Returns a color name matching the robot name.
        This is useful for debugging visualizations.
    """
    if name is None:
        # Purple is an unknown color
        return "Unknown"
    if name == "Fluffy":
        return "Green"
    elif name == "Small":
        return "Red"

    raise ValueError(f"Bad robot name {name}")


def get_robot_color(name):
    """
        Returns a color tuple matching the robot name.
        This is useful for debugging visualizations.
    """
    if name is None:
        # Purple is an unknown color
        return (255, 0, 255)
    if name == "Fluffy":
        return (0, 255, 0)
    elif name == "Small":
        return (255, 0, 0)

    raise ValueError(f"Bad robot name {name}")


def get_intensity_map_pixels(intensity_map):
    """
        Outputs a greyscale RGB image representing a brightness intensity map.
    """
    brightness_map = 255 * intensity_map

    output_map = np.empty((*intensity_map.shape, 3))
    output_map[:, :, 0] = brightness_map
    output_map[:, :, 1] = brightness_map
    output_map[:, :, 2] = brightness_map

    return output_map.astype(np.uint8)


def display_numpy_pixels(display, pixels):
    """
        Draws a numpy array to the display.
        Accepts either RGB pairs or individual intensities.
    """
    if len(pixels.shape) == 2:
        # Image contains brightness data rather than pixel data
        pixels = get_intensity_map_pixels(pixels)

    # Output image to display
    image = display.imageNew(
        pixels.tobytes(),
        Display.RGB,
        pixels.shape[0],
        pixels.shape[1]
    )
    display.imagePaste(image, 0, 0, False)
    display.imageDelete(image)


def to_screenspace(coord, world_bounds, map_resolution):
    return np.rint(np.clip(
        np.flip((coord + world_bounds) / (2 * world_bounds) * map_resolution),
        (0, 0), (map_resolution[0] - 1, map_resolution[1] - 1)
    )).astype(int)


def to_worldspace(_coord, world_bounds, map_resolution):
    coord = np.array([_coord[1], _coord[0]])
    return (2 * (coord / map_resolution) - 1) * world_bounds


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

    # This is strange, maybe the robot gets perfectly aligned with the coordinate axis?
    # This error has only ever been encountered once, check for it anyway
    filtered_array = combined_array[combined_array > 0]
    if len(filtered_array) == 0:
        return 0

    return np.amin(filtered_array)
