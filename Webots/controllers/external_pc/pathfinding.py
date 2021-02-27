from common import util

import numpy as np
from queue import PriorityQueue


def calculate_route(arena_map, start, goal):
    walks = ((1, 0, 1), (1, 1, 2**0.5), (0, 1, 1), (-1, 1, 2**0.5),
             (-1, 0, 1), (-1, -1, 2**0.5), (0, -1, 1), (1, -1, 2**0.5))
    distances = -np.ones((arena_map.shape), dtype=np.float64)
    directions = -np.ones((arena_map.shape), dtype=np.int32)
    distances[start] = 0
    to_explore = PriorityQueue()
    to_explore.put((0, start))

    while not to_explore.empty():
        _, (cur_x, cur_z) = to_explore.get()
        if (cur_x, cur_z) == goal:
            print(distances[goal])
            break
        for direction, (x, z, dist) in enumerate(walks):
            # check if movement is in map and free to move into
            if not (0 <= cur_x + x < 240 and 0 <= cur_z + z < 240 and arena_map[cur_x + x, cur_z + z] != 1):
                continue  # Movement is outside of map

            # if already processed and too expensive, skip reevaluation
            if distances[cur_x + x, cur_z + z] != -1 and distances[cur_x, cur_z] + dist >= distances[cur_x + x, cur_z + z]:
                continue

            directions[cur_x + x, cur_z + z] = direction
            manhattan_dist = util.get_distance((cur_x + x, cur_z + z), goal)
            distances[cur_x + x, cur_z + z] = distances[cur_x, cur_z] + dist
            to_explore.put((
                distances[cur_x + x, cur_z + z] + manhattan_dist,
                (cur_x + x, cur_z + z)
            ))
    else:
        return []

    waypoints = [((goal[0] - 120.0) / 100.0, (goal[1] - 120.0) / 100.0)]
    cur_x, cur_z = goal
    direction = directions[goal]
    while (cur_x, cur_z) != start:
        cur_x -= walks[direction][0]
        cur_z -= walks[direction][1]
        if directions[cur_x, cur_z] != direction:
            waypoints.append((
                (cur_x - 120.0) / 100.0,
                (cur_z - 120.0) / 100.0
            ))
            direction = directions[cur_x, cur_z]

    return waypoints
