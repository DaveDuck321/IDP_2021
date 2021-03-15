import numpy as np
from controller import Display


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
    return np.clip(
        np.flip((coord + world_bounds) / (2 * world_bounds) * map_resolution),
        (0, 0), (map_resolution[0] - 1, map_resolution[1] - 1)
    ).astype(int)


def to_worldspace(_coord, world_bounds, map_resolution):
    coord = np.array([_coord[1], _coord[0]])
    return (2 * (coord / map_resolution) - 1) * world_bounds
