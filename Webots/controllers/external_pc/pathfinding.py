from common import util

import numpy as np
from queue import PriorityQueue


class PathfindingController:
    def __init__(self, arena_shape=(240, 240)):
        self.green_dropoff = [(-0.08, -0.47), (-0.03, -0.47), (0.03, -0.47), (0.08, -0.47)]
        self.red_dropoff = [(-0.16, 0.25), (-0.06, 0.25), (0.04, 0.25), (0.14, 0.25)]

        self.green_spawn = (0.0, -0.3)
        self.red_spawn = (0.0, 0.3)

        self.num_green_returned = 0
        self.num_red_returned = 0

        self.green_path_mask = np.zeros(arena_shape, dtype=np.bool)
        self.red_path_mask = np.zeros(arena_shape, dtype=np.bool)
        self.dropoff_mask = np.zeros(arena_shape, dtype=np.bool)

        self.arena_shape = arena_shape

    def grid_to_world_coord(self, coords):
        coords = ((coords[0] - self.arena_shape[0] / 2.0) / 100.0,
                  (coords[1] - self.arena_shape[1] / 2.0) / 100.0)
        return coords

    def world_to_grid_coords(self, coords):
        coords = (int(coords[0] * 100.0 + 0.5 + self.arena_shape[0] / 2.0),
                  int(coords[1] * 100.0 + 0.5 + self.arena_shape[1] / 2.0))
        return coords

    def dilate(self, grid, iterations):
        """
            Create buffer area around obstacles to avoid robot collision.
        """
        grid = grid.copy()
        for i in range(iterations):
            grid[:-1, :] = grid[1:, :] | grid[:-1, :]
            grid[:, :-1] = grid[:, 1:] | grid[:, :-1]
            grid[1:, :] = grid[:-1, :] | grid[1:, :]
            grid[:, 1:] = grid[:, :-1] | grid[:, 1:]

        return grid

    def mask_delivered_blocks(self, arena_map):
        """
            Returns mask of starting area based on number of blocks delivered
        """
        delivered_blocks_mask = np.zeros((arena_map.shape), dtype=np.bool)

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
        path_mask = np.zeros((arena_map.shape), dtype=np.bool)
        distances[start] = 0
        # PriorityQueue to order squares to visit by their distance + manhatte_distance to the goal
        # meaning that it can find the optimal route without exploring the whole map
        # Format: (distance from start + manhattan distance to goal, (x, z))
        to_explore = PriorityQueue()
        to_explore.put((0, start))

        # create a bool map, where an obstacle is True and free space is False
        arena_map = np.invert(arena_map)
        arena_map = arena_map | self.dilate(arena_map, 10)

        # remove the area around the current target location, to eliminate the block to pick
        # up from the mask of areas not to be traversed
        arena_map[max(0, goal[0] - 10): min(arena_map.shape[0] - 1, goal[0] + 10),
                  max(0, goal[1] - 10): min(arena_map.shape[1] - 1, goal[1] + 10)] = False

        # add the paths of the other robot to the "Do Not Travel" areas
        arena_map = arena_map | self.dilate(mask_path, 15)

        # add the locations of already delivered blocks to the "Do Not Travel" areas
        arena_map = arena_map | self.mask_delivered_blocks(arena_map)

        # explore the map with added heuristic until it is all explored or you have reached the goal
        while not to_explore.empty():
            _, (cur_x, cur_z) = to_explore.get()
            if (cur_x, cur_z) == goal:
                break
            for direction, (x, z, dist) in enumerate(walks):
                # check if movement is in map
                if not (0 <= cur_x + x < arena_map.shape[0] and 0 <= cur_z + z < arena_map.shape[1]):
                    if arena_map[cur_x + x, cur_z + z]:  # check if movement is in obstacle
                        continue  # Movement is outside of map or inside an obstacle

                # if already processed and too expensive, skip reevaluation
                if distances[cur_x + x, cur_z + z] != -1:
                    if distances[cur_x, cur_z] + dist >= distances[cur_x + x, cur_z + z]:
                        continue

                directions[cur_x + x, cur_z + z] = direction
                manhattan_dist = util.get_distance((cur_x + x, cur_z + z), goal)
                distances[cur_x + x, cur_z + z] = distances[cur_x, cur_z] + dist
                # store the newly reached square ready to explore its neighbours
                to_explore.put((
                    distances[cur_x + x, cur_z + z] + manhattan_dist,
                    (cur_x + x, cur_z + z)
                ))
        else:
            return float('inf'), [], None, path_mask.copy()  # No path found

        # Create a list of waypoints for the robot to navigate along
        waypoints = []

        # Store location where robot needs to stop in order to pick up or drop off
        # block at desired location. Required due to robot position being a few cm
        # back compared to block position
        release_pos = None

        cur_x, cur_z = goal
        path_mask[goal] = 1
        direction = directions[goal]

        # backtrack from the goal following the saved directions used to reach each square
        while (cur_x, cur_z) != start:
            cur_x -= walks[direction][0]
            cur_z -= walks[direction][1]

            goal_dist = distances[goal] - distances[cur_x, cur_z]
            if 2 < goal_dist < 4:
                release_pos = self.grid_to_world_coord((cur_x, cur_z))
            if 15.0 < goal_dist < 17:
                waypoints = [self.grid_to_world_coord((cur_x, cur_z))]
            path_mask[cur_x, cur_z] = 1
            if directions[cur_x, cur_z] != direction:
                waypoints.append(self.grid_to_world_coord((cur_x, cur_z)))
                direction = directions[cur_x, cur_z]

        return distances[goal], waypoints, release_pos, path_mask.copy()

    def get_nearest_block_path(self, arena_map, block_positions, robot_pos, robot_name):
        """
            Finds paths for each known block from current robot and returns path to nearest, together with location
            for BlockCollection
        """
        print("calculating path to nearest block")
        robot_pos_matrix = self.world_to_grid_coords(robot_pos)
        print(robot_pos_matrix)
        arena_map_with_border = np.ones(self.arena_shape, dtype=np.bool)
        arena_map_with_border[1:-1, 1:-1] = arena_map

        shortest_path = float('inf')
        waypoint_path = []
        block_release_pos = None
        path_mask = np.zeros((self.arena_shape), dtype=np.bool)
        for block_pos in block_positions:
            block_pos_matrix = self.world_to_grid_coords(block_pos)
            if robot_name == "Fluffy":
                dist, waypoints, release_pos, path_mask = self.calculate_route(
                    arena_map_with_border, self.red_path_mask, robot_pos_matrix, block_pos_matrix)
            elif robot_name == "Small":
                dist, waypoints, release_pos, path_mask = self.calculate_route(
                    arena_map_with_border, self.green_path_mask, robot_pos_matrix, block_pos_matrix)
            if dist < shortest_path:
                shortest_path = dist
                waypoint_path = waypoints
                block_release_pos = release_pos
                if robot_name == "Fluffy":
                    self.green_path_mask = path_mask
                elif robot_name == "Small":
                    self.red_path_mask = path_mask

        return waypoint_path[::-1], block_release_pos

    def get_dropoff_path(self, arena_map, robot_pos, robot_name):
        """
            Returns a list of tuples (x, z) in world coordinate system for robot path back to base, together with
            location for BlockCollection release.
        """
        print("calculating path back to base")
        robot_pos_matrix = self.world_to_grid_coords(robot_pos)
        arena_map_with_border = np.ones(self.arena_shape, dtype=np.bool)
        arena_map_with_border[1:-1, 1:-1] = arena_map

        waypoints = []
        block_release_pos = None
        path_mask = np.zeros((self.arena_shape), dtype=np.bool)

        if robot_name == "Fluffy":
            green_spawn_matrix = self.world_to_grid_coords(self.green_spawn)
            dist, waypoints, block_release_pos, path_mask = self.calculate_route(
                arena_map_with_border, self.red_path_mask, robot_pos_matrix, green_spawn_matrix)
            waypoints = waypoints[::-1] + [self.green_spawn]
            block_release_pos = self.green_dropoff[self.num_green_returned]

        elif robot_name == "Small":
            red_spawn_matrix = self.world_to_grid_coords(self.red_spawn)
            dist, waypoints, block_release_pos, path_mask = self.calculate_route(
                arena_map_with_border, self.green_path_mask, robot_pos_matrix, red_spawn_matrix)
            waypoints = waypoints[::-1] + [self.red_spawn]
            block_release_pos = self.red_dropoff[self.num_red_returned]

        return waypoints, block_release_pos
