from common import util

import numpy as np
from queue import PriorityQueue


class PathfindingController:
    def __init__(self, arena_shape=(240, 240)):
        self.green_dropoff = [(-0.16, -0.56), (-0.06, -0.56), (0.04, -0.56), (0.14, -0.56)]
        self.red_dropoff = [(-0.16, 0.25), (-0.06, 0.25), (0.04, 0.25), (0.14, 0.25)]

        self.num_green_returned = 0
        self.num_red_returned = 0

        self.green_path_mask = np.zeros(arena_shape, dtype=np.int32)
        self.red_path_mask = np.zeros(arena_shape, dtype=np.int32)
        self.dropoff_mask = np.zeros(arena_shape, dtype=np.int32)

    def dilate(self, grid, iterations):
        """
            Create buffer area around obstacles to avoid robot collision.
        """
        grid = grid.copy()
        for i in range(iterations):
            grid[:-1, :] += grid[1:, :]
            grid[:, :-1] += grid[:, 1:]
            grid[1:, :] += grid[:-1, :]
            grid[:, 1:] += grid[:, :-1]

        grid[grid > 1] = 1
        return grid

    def mask_delivered_blocks(self, arena_map):
        """
            Returns mask of starting area based on number of blocks delivered
        """
        delivered_blocks_mask = np.zeros((arena_map.shape), dtype=np.int32)
        # TODO use self.num_green/red_returned to mask of part of starting area
        return delivered_blocks_mask

    def calculate_route(self, arena_map, mask_path, start, goal):
        """
            Uses A* algorithm and knowledge of current path of other robot, along with obstacles
            in arena to avoid collisions and find shortest path and convert that into a list of
            waypoints.
        """
        walks = ((1, 0, 1), (1, 1, 2**0.5), (0, 1, 1), (-1, 1, 2**0.5),
                 (-1, 0, 1), (-1, -1, 2**0.5), (0, -1, 1), (1, -1, 2**0.5))
        distances = -np.ones((arena_map.shape), dtype=np.float64)
        directions = -np.ones((arena_map.shape), dtype=np.int32)
        path_mask = np.zeros((arena_map.shape), dtype=np.int32)
        distances[start] = 0
        to_explore = PriorityQueue()
        to_explore.put((0, start))

        arena_map = self.dilate(arena_map, 10)
        arena_map += self.dilate(mask_path, 15)
        arena_map += self.mask_delivered_blocks(arena_map)
        arena_map[arena_map > 1] = 1

        while not to_explore.empty():
            _, (cur_x, cur_z) = to_explore.get()
            if (cur_x, cur_z) == goal:
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
            return float('inf'), [], None, path_mask.copy()

        waypoints = []
        release_pos = None

        cur_x, cur_z = goal
        path_mask[goal] = 1
        direction = directions[goal]
        while (cur_x, cur_z) != start:
            cur_x -= walks[direction][0]
            cur_z -= walks[direction][1]

            goal_dist = distances[goal] - distances[cur_x, cur_z]
            if 10.25 < goal_dist < 11.32:
                release_pos = (cur_x, cur_z)
            if 15.0 < goal_dist < 16.06:
                waypoints = [((cur_x - 120.0) / 100.0, (cur_z - 120.0) / 100.0)]
            path_mask[cur_x, cur_z] = 1
            if directions[cur_x, cur_z] != direction:
                waypoints.append((
                    (cur_x - 120.0) / 100.0,
                    (cur_z - 120.0) / 100.0
                ))
                direction = directions[cur_x, cur_z]

        return distances[goal], waypoints, release_pos, path_mask.copy()

    def get_nearest_block_path(self, arena_map, block_positions, robot_pos, robot_color):
        """
            Finds paths for each known block from current robot and returns path to nearest, together with location
            for BlockCollection
        """
        robot_x_pos = int(robot_pos[0] * 100.0 + 0.5) + 120
        robot_y_pos = int(robot_pos[1] * 100.0 + 0.5) + 120
        robot_pos_matrix = (robot_x_pos, robot_y_pos)
        arena_map_with_border = np.ones((240, 240), dtype=np.int32)
        arena_map_int = arena_map.astype('int32')
        arena_map_with_border[1:-1, 1:-1] = arena_map_int

        shortest_path = float('inf')
        waypoint_path = []
        block_release_pos = None
        path_mask = np.zeros((arena_map.shape), dtype=np.int32)
        for block_pos in block_positions:
            block_pos_matrix = (int(block_pos[0] * 100.0 + 0.5) + 120, int(block_pos[1] * 100.0 + 0.5) + 120)
            if robot_color == "green":
                dist, waypoints, release_pos, path_mask = self.calculate_route(
                    arena_map_with_border, self.red_path_mask, robot_pos_matrix, block_pos_matrix)
            elif robot_color == "red":
                dist, waypoints, release_pos, path_mask = self.calculate_route(
                    arena_map_with_border, self.green_path_mask, robot_pos_matrix, block_pos_matrix)
            if dist < shortest_path:
                shortest_path = dist
                waypoint_path = waypoints
                block_release_pos = release_pos
                if robot_color == "green":
                    self.green_path_mask = path_mask
                elif robot_color == "red":
                    self.red_path_mask = path_mask

        return waypoint_path, block_release_pos

    def get_dropoff_path(self, arena_map, robot_pos, robot_color):
        """
            Returns a list of tuples (x, z) in world coordinate system for robot path back to base, together with
            location for BlockCollection release.
        """
        robot_x_pos = int(robot_pos[0] * 100.0 + 0.5) + 120
        robot_y_pos = int(robot_pos[1] * 100.0 + 0.5) + 120
        robot_pos_matrix = (robot_x_pos, robot_y_pos)
        arena_map_with_border = np.ones((240, 240), dtype=np.int32)
        arena_map_int = arena_map.astype('int32')
        arena_map_with_border[1:-1, 1:-1] = arena_map_int

        waypoints = []
        block_release_pos = None
        path_mask = np.zeros((arena_map.shape), dtype=np.int32)

        if robot_color == "green":
            block_pos_matrix = (
                int(self.green_dropoff[self.num_green_returned][0] * 100.0 + 0.5) + 120,
                int(self.green_dropoff[self.num_green_returned][1] * 100.0 + 0.5) + 120)
            dist, waypoints, block_release_pos, path_mask = self.calculate_route(
                arena_map_with_border, self.red_path_mask, robot_pos_matrix, block_pos_matrix)
        elif robot_color == "red":
            block_pos_matrix = (
                int(self.red_dropoff[self.num_red_returned][0] * 100.0 + 0.5) + 120,
                int(self.red_dropoff[self.num_red_returned][1] * 100.0 + 0.5) + 120)
            dist, waypoints, block_release_pos, path_mask = self.calculate_route(
                arena_map_with_border, self.green_path_mask, robot_pos_matrix, block_pos_matrix)

        return waypoints, block_release_pos
