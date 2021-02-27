import math


def filter_nan(number, default=math.inf):
    """
        If number is NaN, return the default value.
        Else return number.
    """
    if math.isnan(number):
        return default

    return number


def get_distance(p1, p2):
    """
        Returns the distance between two tuple coordinates.
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def get_bearing(p1, p2):
    """
        Returns the bearing between two tuple coordinates.
    """
    return math.atan2(p1[1] - p2[1], p1[0] - p2[0])
