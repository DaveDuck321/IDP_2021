from common import util
from controller import Display
import mapping

import numpy as np
from queue import PriorityQueue
from collections import defaultdict, namedtuple


def dilate(grid, iterations):
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


class PathfindingController:
    def __init__(self, display_pathfinding, arena_shape=(121, 121)):

        self.robots_spawn = {
            "Fluffy": (0.0, -0.28),
            "Small": (0.0, 0.28),
        }
        self.dropoff_locations = {
            "Fluffy": [(-0.105, -0.47), (-0.035, -0.47), (0.035, -0.47), (0.105, -0.47)],
            "Small": [(-0.105, 0.47), (-0.035, 0.47), (0.035, 0.47), (0.105, 0.47)],
        }

        self.number_returned = defaultdict(int)

        self.path_masks = {}
        # self.green_path_mask = np.zeros(arena_shape, dtype=np.bool)
        # self.red_path_mask = np.zeros(arena_shape, dtype=np.bool)
        self.dropoff_mask = np.zeros(arena_shape, dtype=np.bool)

        self.arena_shape = arena_shape

        self._display_pathfinding = display_pathfinding

    def send_to_display(self, grid):
        np_image = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
        np_image[:, :, 2] = np.where(grid, 255, 0)
        if "Small" in self.path_masks:
            np_image[:, :, 0] = np.where(self.path_masks["Small"], 255, 0)
        if "Fluffy" in self.path_masks:
            np_image[:, :, 1] = np.where(self.path_masks["Fluffy"], 255, 0)
        np_image = np.swapaxes(np_image, 0, 1)

        image = self._display_pathfinding.imageNew(
            np_image.tobytes(),
            Display.RGB,
            np_image.shape[0],
            np_image.shape[1]
        )
        self._display_pathfinding.imagePaste(image, 0, 0, False)
        self._display_pathfinding.imageDelete(image)

    def grid_to_world_coord(self, coords):
        coords = ((coords[0] - self.arena_shape[0] / 2.0) / mapping.WORLD_RESOLUTION,
                  (coords[1] - self.arena_shape[1] / 2.0) / mapping.WORLD_RESOLUTION)
        return coords

    def world_to_grid_coords(self, coords):
        coords = (int(coords[0] * mapping.WORLD_RESOLUTION + 0.5 + self.arena_shape[0] / 2.0),
                  int(coords[1] * mapping.WORLD_RESOLUTION + 0.5 + self.arena_shape[1] / 2.0))
        return coords

    def mask_delivered_blocks(self, arena_map):
        """
            Returns mask of starting area based on number of blocks delivered
        """
        delivered_blocks_mask = np.zeros((arena_map.shape), dtype=np.bool)
        for robot_name in self.dropoff_locations:
            for block_pos in self.dropoff_locations[robot_name][:self.number_returned[robot_name]]:
                (x, z) = self.world_to_grid_coords(block_pos)
                delivered_blocks_mask[x - 2:x + 3, z - 2:z + 3] = True

        return delivered_blocks_mask

    def calculate_route(self, robot_name, arena_map, start, goal):
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
        death_mask = np.zeros((arena_map.shape), dtype=np.bool)
        distances[start] = 0
        # PriorityQueue to order squares to visit by their distance + manhatte_distance to the goal
        # meaning that it can find the optimal route without exploring the whole map
        # Format: (distance from start + manhattan distance to goal, (x, z))
        to_explore = PriorityQueue()
        to_explore.put((0, start))

        # create a bool map, where an obstacle is True and free space is False
        arena_map = np.invert(arena_map)

        # add the locations of already delivered blocks to the "Do Not Travel" areas
        death_mask = death_mask | dilate(self.mask_delivered_blocks(arena_map), 7)

        # leave a safety margin around areas of "Do Not Travel"
        arena_map = arena_map | dilate(arena_map, 8)

        # remove the area around the current target location, to eliminate the block to pick
        # up from the mask of areas not to be traversed
        arena_map[max(0, goal[0] - 10): min(arena_map.shape[0] - 1, goal[0] + 10),
                  max(0, goal[1] - 10): min(arena_map.shape[1] - 1, goal[1] + 10)] = False

        # add the paths of the other robot to the "Do Not Travel" areas
        for other_name in self.path_masks:
            # Dilate all other robot paths
            if other_name == robot_name:
                continue
            death_mask = death_mask | dilate(self.path_masks[other_name], 8)

        self.send_to_display(arena_map)
        # explore the map with added heuristic until it is all explored or you have reached the goal
        while not to_explore.empty():
            _, (cur_x, cur_z) = to_explore.get()
            if (cur_x, cur_z) == goal:
                break
            for direction, (x, z, dist) in enumerate(walks):
                # check if movement is in map
                if not (0 <= cur_x + x < arena_map.shape[0] and 0 <= cur_z + z < arena_map.shape[1]):
                    continue
                if death_mask[cur_x + x, cur_z + z]:
                    continue
                if arena_map[cur_x + x, cur_z + z] == 1:  # check if movement is in obstacle
                    dist *= 1000

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
            if 3 < goal_dist < 5:
                release_pos = self.grid_to_world_coord((cur_x, cur_z))
            if 9 < goal_dist < 11:
                waypoints = [self.grid_to_world_coord((cur_x, cur_z))]
            path_mask[cur_x, cur_z] = 1
            if directions[cur_x, cur_z] != direction:
                waypoints.append(self.grid_to_world_coord((cur_x, cur_z)))
                direction = directions[cur_x, cur_z]

        return distances[goal], waypoints, release_pos, path_mask.copy()

    def get_nearest_block_path(self, robot_name, arena_map, clusters, robot_pos):
        """
            Finds paths for each known block from current robot.
            Returns: namedtuple("WaypointPath", ["distance", "waypoint_path", "release_pos", "goal_pos"])

        """
        # print("calculating path to nearest block")
        arena_map = np.swapaxes(arena_map, 0, 1)
        robot_pos_matrix = self.world_to_grid_coords(robot_pos)
        arena_map_with_border = np.zeros(self.arena_shape, dtype=np.bool)
        arena_map_with_border[1:-1, 1:-1] = arena_map

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

    def get_dropoff_path(self, arena_map, robot_pos, robot_name):
        """
            Returns a list of tuples (x, z) in world coordinate system for robot path back to base, together with
            location for BlockCollection release.

            Note: ONLY CALL ONCE PER BLOCK
        """
        # print("calculating path back to base")
        arena_map = np.swapaxes(arena_map, 0, 1)
        arena_map_with_border = np.zeros(self.arena_shape, dtype=np.bool)
        arena_map_with_border[1:-1, 1:-1] = arena_map

        waypoints = []
        block_release_pos = None
        path_mask = np.zeros((self.arena_shape), dtype=np.bool)

        robot_grid_coords = self.world_to_grid_coords(robot_pos)
        spawn_grid_coords = self.world_to_grid_coords(self.robots_spawn[robot_name])

        dist, waypoints, block_release_pos, path_mask = self.calculate_route(
            robot_name, arena_map_with_border, robot_grid_coords, spawn_grid_coords
        )
        waypoints = waypoints[::-1] + [self.robots_spawn[robot_name]]
        block_release_pos = self.dropoff_locations[robot_name][self.number_returned[robot_name]]
        self.path_masks[robot_name] = path_mask

        self.number_returned[robot_name] += 1
        return waypoints, block_release_pos
