import mapping
from common import util

import numpy as np
import heapq

BLOCK_SIZE = (5, 5)  # In raw map pixels
SIMPLIFIED_RESOLUTION = (9, 9)
SIGNIFICANT_OBSTACLE_DENSITY = 5  # Mapping will ignore obstacles of less than 5px
EMPTY_MAP = np.zeros(SIMPLIFIED_RESOLUTION, dtype=np.bool8)


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


class PathfindingResult:
    def __init__(self, pathmask, waypoints, length, goal_pos, success=True):
        self.success = success
        self.pathmask = pathmask
        self.waypoints = waypoints
        self.length = length
        self.goal_pos = goal_pos

    @classmethod
    def NotFound(cls, goal_pos):
        return cls(
            EMPTY_MAP, [], np.inf,
            goal_pos, success=False
        )


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

    def _generate_cooperation_map(self, robot_name):
        """
            Returns a simple map for use in pathfinding. Preexisting robot paths are marked as invalid regions.
        """
        cooperation_map = np.invert(self._simple_map)

        # Mask off other robot paths
        for other_robot in self.path_masks:
            if other_robot != robot_name:
                cooperation_map |= self.path_masks[other_robot]

        # Return this map to the standard format
        return np.invert(cooperation_map)

    def output_to_display(self, robot_states, clusters):
        """
            Outputs the current calculated routes and map to the Webots display
        """
        output_map = util.get_intensity_map_pixels(self._simple_map)

        # Draw the targets
        for cluster in clusters:
            color = util.get_robot_color(cluster.color)
            output_map[tuple(_to_screenspace(cluster.coord))] = color

        # Draw any calculated paths
        for robot_name in self.path_masks:
            color = util.get_robot_color(robot_name)
            output_map[self.path_masks[robot_name]] = color

        # Draw the robots:
        for robot_name in robot_states:
            color = util.get_robot_color(robot_name)
            robot_position = tuple(_to_screenspace(robot_states[robot_name].position))
            output_map[robot_position] = color

        util.display_numpy_pixels(self._display, output_map)

    def update_map(self, raw_arena_map):
        """
            Invalidates cache and generates the simplified map ready for pathfinding.
        """
        self._generate_simple_map(raw_arena_map)

    def calculate_route(self, robot_name, start_pos, goal_pos):
        """
            Uses A* algorithm and knowledge of current path of other robot, along with obstacles
            in arena to avoid collisions and find shortest path and convert that into a list of
            waypoints.
        """
        start, goal = tuple(_to_screenspace(start_pos)), tuple(_to_screenspace(goal_pos))

        # Setup the pathfinding
        UNIT_STEP = 2
        DIAGONAL_STEP = 3  # 3/2 ≈ √2
        walks = np.array([
            (1, 0, UNIT_STEP),
            (1, 1, DIAGONAL_STEP),
            (0, 1, UNIT_STEP),
            (-1, 1, DIAGONAL_STEP),
            (-1, 0, UNIT_STEP),
            (-1, -1, DIAGONAL_STEP),
            (0, -1, UNIT_STEP),
            (1, -1, DIAGONAL_STEP)
        ], dtype=np.int32)

        pathfinding_map = self._simple_map  # self._generate_cooperation_map(robot_name)
        costs = np.inf * np.ones(SIMPLIFIED_RESOLUTION)
        directions = np.empty(SIMPLIFIED_RESOLUTION, dtype=np.int32)

        # PriorityQueue to order squares to visit by their distance + manhatte_distance to the goal
        # meaning that it can find the optimal route without exploring the whole map
        # Format: (distance from start + manhattan distance to goal, (x, z))
        to_explore = [(0, start)]
        costs[start] = 0

        # explore the map with added heuristic until it is all explored or you have reached the goal
        while len(to_explore) > 0:
            _, current_pos = heapq.heappop(to_explore)

            for direction_id, (x, z, step_dist) in enumerate(walks):
                new_pos = (
                    current_pos[0] + x,
                    current_pos[1] + z
                )
                # check if movement is in map
                if not util.tuple_in_bound(new_pos, SIMPLIFIED_RESOLUTION):
                    continue

                # check if movement is in obstacle
                # Targets cannot be obstacles
                if new_pos != goal and (not pathfinding_map[new_pos]):
                    continue

                # if already processed and too expensive, skip reevaluation
                new_cost = costs[current_pos] + step_dist
                if new_cost >= costs[new_pos] or new_cost >= costs[goal]:
                    continue

                manhattan_dist = abs(new_pos[0] - goal[0]) + abs(new_pos[1] - goal[1])
                costs[new_pos] = new_cost
                directions[new_pos] = direction_id
                # store the newly reached square ready to explore its neighbours
                heapq.heappush(to_explore, (
                    new_cost + manhattan_dist,
                    new_pos
                ))

        # Check if pathfinding succeeded
        if np.isinf(costs[goal]):
            return PathfindingResult.NotFound(goal_pos)

        # Mask to avoid collisions with other robots
        pathmask = np.zeros(SIMPLIFIED_RESOLUTION, dtype=np.bool8)

        # Create a list of waypoints for the robot to navigate along
        waypoints = [tuple(goal_pos)]

        # backtrack from the goal following the saved directions used to reach each square
        current_coord = goal
        direction_index = directions[current_coord]
        pathmask[current_coord] = True

        while current_coord != start:
            current_coord = (
                current_coord[0] - walks[direction_index][0],
                current_coord[1] - walks[direction_index][1]
            )

            # Record this on the mask
            pathmask[current_coord] = True

            # Output a minimum number of waypoint to allow better control
            if directions[current_coord] != direction_index:
                waypoints.append(tuple(_to_worldspace(current_coord)))
                direction_index = directions[current_coord]

        return PathfindingResult(pathmask, waypoints[::-1], costs[goal], goal_pos)

    def get_nearest_block_path(self, clusters, robot_name, robot_pos):
        """
            Finds paths for each known block from current robot.
            Returns: namedtuple("WaypointPath", ["distance", "waypoint_path", "release_pos", "goal_pos"])

        """
        shortest_path = PathfindingResult.NotFound((0.0, 0.0))

        for cluster in clusters:
            # Filter out blocks of an incorrect color
            if cluster.color is not None and cluster.color != robot_name:
                continue

            # Calculate the route for this block
            route = self.calculate_route(robot_name, robot_pos, cluster.coord)

            if route.length < shortest_path.length:
                shortest_path = route
                self.path_masks[robot_name] = shortest_path.pathmask

        # If no paths are found, delete any pending paths
        if not shortest_path.success:
            self.path_masks[robot_name] = EMPTY_MAP

        return shortest_path
