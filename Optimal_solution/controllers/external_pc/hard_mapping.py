"""
    Mapping utilities to store exact coordinates such as confirmed block locations
"""

from common import util
import mapping

import numpy as np

from collections import namedtuple
from functools import reduce
import itertools


def get_cluster_average(cluster_candidates, weights):
    """
        Groups together True pixels and returns a list of ClusterLocation objects.
        Output is entirely in world coordinates.
    """

    # Err, comment to explain this absolute mess of a line
    intense_coords = dict(zip(zip(*np.where(cluster_candidates)), itertools.repeat(False)))
    strides = np.array([(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)])

    # Almost functional iterator
    def find_cluster_coords(coord):
        # Needs to be a tuple for indexing
        index = tuple(coord)
        if not util.tuple_in_bound(index, mapping.MAP_RESULTION) or \
                not cluster_candidates[index] or intense_coords[index]:
            # Block has already been counted or shouldn't be included
            return []

        # Mark this pixel to avoid repeated counting
        intense_coords[index] = True
        return list(reduce(
            lambda old, stride: np.concatenate((old, find_cluster_coords(coord + stride))),
            strides, coord
        ))

    # Now finally, group the clusters
    clusters_coords = []
    for coord in intense_coords:
        if intense_coords[coord] or len(coord) == 0:
            continue  # Already processed
        cluster_coords = np.reshape(find_cluster_coords(np.array(coord)), (-1, 2)).astype(int)
        clusters_coords.append(cluster_coords)

    # Now do some statistics to find centers
    cluster_locations = []  # list of BlockLocation
    for cluster_coords in clusters_coords:
        if len(cluster_coords) < mapping.CLUSTER_THRESHOLD:
            continue

        total_weight = 0
        cluster_sum = np.array((0.0, 0.0))
        for coord in cluster_coords:
            total_weight += weights[tuple(coord)]
            cluster_sum += coord * weights[tuple(coord)]

        # Create a cluster representing the average properties of these coordinates
        # Convert everything to worldspace
        average_position = mapping._to_worldspace(cluster_sum / total_weight)

        radius = 0
        for coord in cluster_coords:
            world_coord = mapping._to_worldspace(coord)
            radius = max(radius, util.get_distance(average_position, world_coord))

        cluster_locations.append(ClusterLocation(average_position, len(cluster_coords), total_weight, radius))

    # Merge nearby clusters
    merged_clusters = []
    for cluster in cluster_locations:
        for large_cluster in merged_clusters:
            cluster_distance = util.get_distance(large_cluster.coord, cluster.coord)

            # Test if clusters are close
            if cluster_distance < mapping.CLUSTER_OVERLAP_THRESHOLD * (large_cluster.radius + cluster.radius):
                # Clusters overlap, take a weighted average and change the original cluster
                large_cluster.merge_cluster(cluster)
                break
        else:
            # Cluster could not be merged into another cluster, add it to merged_clusters
            merged_clusters.append(cluster)

    return merged_clusters


class ClusterLocation:
    """
        Represents the location and statistical certainty of a block reading.
        This can be merged with nearby block locations to remove noise.
    """

    def __init__(self, coord, area, total_weight, radius):
        self.coord = coord
        self.area = area
        self.total_weight = total_weight
        self.radius = radius
        self.color = None
        self.known_block = None

    def assign_known_color(self, known_block):
        self.known_block = known_block
        self.color = known_block[1]

    def merge_cluster(self, other):
        """
            Takes a weighted average of cluster coordinates in order to merge two clusters.
            This causes the cluster to grow in weight and area.
        """
        combined_weight = self.total_weight + other.total_weight
        self.coord = (
            self.total_weight * self.coord + other.total_weight * other.coord
        ) / combined_weight
        self.total_weight = combined_weight
        self.area += other.area


class ExactMapping:
    def __init__(self):
        self.confirmed_blocks = []  # (coords, color)
        self.dropped_blocks = []

    def update_with_color_reading(self, block_location, block_color):
        """
            A robot has just read a block color, log this block position to the world map.
        """
        self.confirmed_blocks.append((block_location, block_color))

    def invalid_region(self, block_locations, position, invalidation_size):
        """
            Invalidates the region surrounding a single block.
            This should be used after the robot changes the environment for an reason.
        """

        # Remove each block in a known position
        for cluster in block_locations:
            invalidation_radius = max(invalidation_size, cluster.radius)

            if util.get_distance(cluster.coord, position) < invalidation_radius:
                if(cluster.known_block is not None):
                    print("[info] Block in invalidation cluster, removing")
                    self.confirmed_blocks.remove(cluster.known_block)

        # Also delete overlapping blocks if they are not in a cluster
        for index in range(len(self.confirmed_blocks) - 1, 0, -1):
            block_pos, _ = self.confirmed_blocks[index]
            if util.get_distance(block_pos, position) < invalidation_size:
                # Purge this block
                print("[info] Block in invalidation region, removing")
                self.confirmed_blocks.pop(index)

    def add_drop_off_region(self, position):
        """
            The block has been dropped off. Block should be excluded from all future scans.
        """
        self.dropped_blocks.append(position)

    def predict_block_locations(self, cluster_candidates, occupancy_map):

        # Smooth clusters into single block
        clusters = get_cluster_average(cluster_candidates, occupancy_map)

        # Some block colors are known, mark these in the clusters object
        for block in self.confirmed_blocks:
            block_location, block_color = block
            # Check if block is in radius of any clusters
            color_consumed = False

            # NOTE: this is a back hackfix
            ClosestCluster = namedtuple("ClosestBlock", ["distance", "cluster"])
            closest_cluster = ClosestCluster(np.inf, None)

            for cluster in clusters:
                cluster_distance = util.get_distance(cluster.coord, block_location)

                if cluster_distance < mapping.CLUSTER_PROXIMITY_THRESHOLD:
                    if (cluster.color is not None):
                        print("[Warning] Multiple blocks identified in the same cluster")

                    # if color_consumed:
                    #    print("[Warning] The same blocks has been assigned to multiple clusters")

                    color_consumed = True
                    cluster.assign_known_color(block)

                if cluster_distance < closest_cluster.distance:
                    closest_cluster = ClosestCluster(
                        cluster_distance,
                        cluster
                    )

            if not color_consumed:
                # Hackfix, no cluster identified properly, assign to closest one
                # print("[Warning] Block color has been identified but does not exist on map")
                # closest_cluster.cluster.assign_known_color(index, block_color)
                new_cluster = ClusterLocation(
                    block_location, 20, 10, mapping.BLOCK_OBSTACLE_WIDTH
                )
                new_cluster.assign_known_color(block)
                clusters.append(new_cluster)

        return clusters
