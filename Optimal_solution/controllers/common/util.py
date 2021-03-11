import math


def filter_nans(iterator, default=math.inf):
    """
        If number is NaN, return the default value.
        Else return number.
    """
    return tuple(
        filter_nan(number, default) for number in iterator
    )


def filter_nan(number, default=math.inf):
    """
        If number is NaN, return the default value.
        Else return number.
    """
    if math.isnan(number):
        return default

    return number


def get_distance(p1, p2=(0, 0)):
    """
        Returns the distance between two tuple coordinates.
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def get_bearing(p1, p2):
    """
        Returns the bearing between two tuple coordinates.
    """
    return math.atan2(p1[1] - p2[1], p1[0] - p2[0])


def tuple_in_bound(p1, bounds):
    """
        Returns true if a numpy array of shape (bounds) could be indexed by the tuple.
    """
    return p1[0] >= 0 and p1[1] >= 0 and p1[0] < bounds[0] and p1[1] < bounds[1]


def normalize_radian(phase):
    """
        Normalizes a phase radian between +/- pi.
    """
    return (phase + math.pi) % (2 * math.pi) - math.pi


def get_robot_name(color):
    if color == "green":
        return "Fluffy"
    elif color == "red":
        return "Small"

    raise ValueError(f"Bad robot color {color}")
