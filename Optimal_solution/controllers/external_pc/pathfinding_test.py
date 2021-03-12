import mapping
from common import util

import numpy as np

from collections import namedtuple
import heapq

BLOCK_SIZE = (5, 5)  # In raw map pixels
SIMPLIFIED_RESOLUTION = (8, 8)
SIGNIFICANT_OBSTACLE_DENSITY = 5  # Mapping will ignore obstacles of less than 5px

SQRT_2 = np.sqrt(2)


def _to_screenspace(coord):
    return util.to_screenspace(coord, mapping.WORLD_BOUNDS, SIMPLIFIED_RESOLUTION)


def _to_worldspace(coord):
    return util.to_worldspace(coord, mapping.WORLD_BOUNDS, SIMPLIFIED_RESOLUTION)


def dilate(grid, iterations=1):
    """
        Create buffer area around obstacles to avoid robot collision.
    """
    for _ in range(iterations):
        grid[:-1, :] = grid[1:, :] | grid[:-1, :]
        grid[:, :-1] = grid[:, 1:] | grid[:, :-1]
        grid[1:, :] = grid[:-1, :] | grid[1:, :]
        grid[:, 1:] = grid[:, :-1] | grid[:, 1:]


class PathfindingController:
    def __init__(self, display):
        self.path_masks = {}

        self._display = display
        self._simple_map = None

    def _generate_simple_map(self, raw_arena_map):
        """
            Converts a high resolution raw_arena_map into an equivalent map while simplifying the geometry.
            each cell in raw_arena_map must be False (occupied) or True (free)
        """
        # add the locations of already delivered blocks to the "Do Not Travel" areas
        death_mask = np.invert(raw_arena_map)  # | self.mask_delivered_blocks(raw_arena_map)

        # Simplify the map
        MAP_SCALE = (
            int(raw_arena_map.shape[0] / SIMPLIFIED_RESOLUTION[0]),
            int(raw_arena_map.shape[1] / SIMPLIFIED_RESOLUTION[1])
        )

        simple_map = np.zeros(SIMPLIFIED_RESOLUTION, dtype=np.bool8)
        for x in range(0, SIMPLIFIED_RESOLUTION[0]):
            for y in range(0, SIMPLIFIED_RESOLUTION[1]):
                lower = (x * MAP_SCALE[0], y * MAP_SCALE[1])
                upper = (lower[0] + MAP_SCALE[0], lower[1] + MAP_SCALE[1])

                obstacle_density = np.sum(death_mask[lower[0]:upper[0], lower[1]:upper[1]])
                if obstacle_density > SIGNIFICANT_OBSTACLE_DENSITY:
                    simple_map[x, y] = True

        # Flip bits to correct to standard format
        self._simple_map = np.invert(simple_map)

    def output_to_display(self):
        """
            Outputs the current calculated routes and map to the Webots display
        """
        util.display_numpy_pixels(self._display, self._simple_map)

    def update_map(self, raw_arena_map):
        """
            Invalidates cache and generates the simplified map ready for pathfinding.
        """
        self._generate_simple_map(raw_arena_map)

    def mask_delivered_blocks(self, out_map):
        """
            Returns mask of starting area based on number of blocks delivered
        """
        for robot_name in self.dropoff_locations:
            for block_pos in self.dropoff_locations[robot_name][:self.number_returned[robot_name]]:
                (x, z) = self.world_to_grid_coords(block_pos)
                out_map[x - 2:x + 3, z - 2:z + 3] = True

    def calculate_route(self, robot_name, start_pos, goal_pos):
        """
            Uses A* algorithm and knowledge of current path of other robot, along with obstacles
            in arena to avoid collisions and find shortest path and convert that into a list of
            waypoints.
        """
        start, goal = _to_screenspace(start_pos), _to_screenspace(goal_pos)

        # Setup the pathfinding
        walks = np.array([
            (1, 0, 1),
            (1, 1, SQRT_2),
            (0, 1, 1),
            (-1, 1, SQRT_2),
            (-1, 0, 1),
            (-1, -1, SQRT_2),
            (0, -1, 1),
            (1, -1, SQRT_2)
        ])

        distances = np.inf * np.ones(SIMPLIFIED_RESOLUTION)
        directions = np.empty((*SIMPLIFIED_RESOLUTION, 2))

        # PriorityQueue to order squares to visit by their distance + manhatte_distance to the goal
        # meaning that it can find the optimal route without exploring the whole map
        # Format: (distance from start + manhattan distance to goal, (x, z))
        to_explore = []
        to_explore.put((0, start))

        # explore the map with added heuristic until it is all explored or you have reached the goal
        while len(to_explore) > 0:
            _, (cur_x, cur_z) = heapq.heappop(to_explore)
            if (cur_x, cur_z) == goal:
                break

            for direction, (x, z, step_dist) in enumerate(walks):
                new_x, new_z = x, z
                # check if movement is in map
                if not (0 <= new_x < SIMPLIFIED_RESOLUTION[0] and 0 <= new_z < SIMPLIFIED_RESOLUTION[1]):
                    continue
                if self._simple_map[new_x, new_z]:  # check if movement is in obstacle
                    continue

                # if already processed and too expensive, skip reevaluation
                if distances[cur_x, cur_z] + step_dist >= distances[new_x, new_z]:
                    continue

                manhattan_dist = abs(new_x - goal[0]) + abs(new_z - goal[1])
                distances[new_x, new_z] = distances[cur_x, cur_z] + step_dist
                directions[new_x, new_z] = (x, z)
                # store the newly reached square ready to explore its neighbours
                heapq.heappush(to_explore, (
                    distances[new_x, new_z] + manhattan_dist,
                    (new_x, new_z)
                ))
        else:
            return []  # No path found

        # Create a list of waypoints for the robot to navigate along
        waypoints = [goal_pos]

        # backtrack from the goal following the saved directions used to reach each square
        direction = directions[goal]

        cur_x, cur_z = goal
        while (cur_x, cur_z) != start:
            # NOTE: this is a floating point error for large grids
            cur_x -= direction[0]
            cur_z -= direction[1]

            # Output a minimum number of waypoint to allow better control
            if directions[cur_x, cur_z] != direction:
                waypoints.append(_to_worldspace((cur_x, cur_z)))
                direction = directions[cur_x, cur_z]

        return distances[goal], waypoints

    def get_nearest_block_path(self, robot_name, raw_arena_map, clusters, robot_pos):
        """
            Finds paths for each known block from current robot.
            Returns: namedtuple("WaypointPath", ["distance", "waypoint_path", "release_pos", "goal_pos"])

        """
        # print("calculating path to nearest block")
        arena_map = np.swapaxes(arena_map, 0, 1)
        robot_pos_matrix = self.world_to_grid_coords(robot_pos)
        arena_map_with_border = np.zeros(self.arena_shape, dtype=np.bool)
        arena_map_with_border[1: -1, 1: -1] = arena_map

        WaypointPath = namedtuple("WaypointPath", ["distance", "waypoint_path", "release_pos", "goal_pos"])
        shortest_path = WaypointPath(np.inf, [], (0, 0), (0, 0))

        path_mask = np.zeros((self.arena_shape), dtype=np.bool)
        for cluster in clusters:
            # Filter out blocks of an incorrect color
            if cluster.color is not None and cluster.color != robot_name:
                continue

            block_pos_matrix = self.world_to_grid_coords(cluster.coord)

            # Calculate the route for this block
            dist, waypoints, release_pos, path_mask = self.calculate_route(
                robot_name, arena_map_with_border, robot_pos_matrix, block_pos_matrix
            )

            if dist < shortest_path.distance:
                shortest_path = WaypointPath(
                    dist, waypoints[::-1], release_pos, cluster.coord
                )
                self.path_masks[robot_name] = path_mask

        # self.send_to_display(arena_map_with_border)
        return shortest_path
