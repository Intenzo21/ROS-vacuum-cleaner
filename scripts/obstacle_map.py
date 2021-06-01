#!/usr/bin/env python

""" 
An implementation of an obstacle map.
Used in laser scan updating.
"""

import numpy as np
from sklearn.neighbors import NearestNeighbors


class ObstacleMap:
    """ 
    Generate an obstacle map from a given input map.  
    Adopt it to return the distance to the closest
    obstacle for any coordinate in the map
    """

    def __init__(self, map):

        self.map = map  # nav_msgs/OccupancyGrid map where the localization occurs
        X = np.zeros(
            (self.map.info.width * self.map.info.height, 2))  # Numpy array of all grid cell coordinates of the map

        # Count the number of occupied cells
        tot_occ = 0
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                ind = i + j * self.map.info.width
                if self.map.data[ind] > 0:
                    tot_occ += 1
                X[curr, 0] = float(i)
                X[curr, 1] = float(j)
                curr += 1

        # Create a numpy array of the coordinates of each occupied grid cell in the map
        occ_arr = np.zeros((tot_occ, 2))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                ind = i + j * self.map.info.width
                if self.map.data[ind] > 0:
                    occ_arr[curr, 0] = float(i)
                    occ_arr[curr, 1] = float(j)
                    curr += 1

        # Use super fast scikit learn nearest neighbor algorithm
        nbrs = NearestNeighbors(n_neighbors=1, algorithm="ball_tree").fit(occ_arr)
        dist, indices = nbrs.kneighbors(X)

        # Create dictionary to hold the closes obstacle to every point in the map
        self.closest_occ = {}
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                ind = i + j * self.map.info.width
                self.closest_occ[ind] = dist[curr][0] * self.map.info.resolution
                curr += 1

    def get_dist_to_obstacle(self, x, y):
        """ 
        Compute the closest obstacle to the specified (x,y) coordinate in the map.  
        Return nan if out of bounds.
        """
        x_coord = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        y_coord = int((y - self.map.info.origin.position.y) / self.map.info.resolution)

        # check if we are in bounds
        if x_coord > self.map.info.width or x_coord < 0:
            return float('nan')
        if y_coord > self.map.info.height or y_coord < 0:
            return float('nan')

        ind = x_coord + y_coord * self.map.info.width

        # Additional check
        if ind >= self.map.info.width * self.map.info.height or ind < 0:
            return float('nan')

        return self.closest_occ[ind]
