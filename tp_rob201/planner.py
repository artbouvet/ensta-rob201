"""
Planner class
Implementation of A*
"""

import numpy as np
import heapq

from occupancy_grid import OccupancyGrid


class Planner:
    """Simple occupancy grid Planner"""

    def __init__(self, occupancy_grid: OccupancyGrid):
        self.grid = occupancy_grid


        # Origin of the odom frame in the map frame
        self.odom_pose_ref = np.array([0, 0, 0])

    def get_neighbors(self, current_cell):
        """
        pour une position courante, renvoie ses 8 voisins sur la carte
        """
        neighbors = []
        x, y = current_cell
        directions = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
        for dx,dy in directions:
            n_x, n_y = x+dx, y+dy
            if 0 <= n_x < self.grid.x_max_map and 0<= n_y < self.grid.y_max_map:
                if self.grid.occupancy_map[n_x, n_y] == 0:  # 0 si position libre
                    neighbors.append((n_x, n_y))
        return neighbors
    
    def heuristic(self, cell1, cell2):
        """
        renvoie la distance euclidienne entre 2 points sur la carte
        """
        return np.linalg.norm(np.array(cell1)-np.array(cell2))


    def plan(self, start, goal):
        """
        Compute a path using A*, recompute plan if start or goal change
        start : [x, y, theta] nparray, start pose in world coordinates (theta unused)
        goal : [x, y, theta] nparray, goal pose in world coordinates (theta unused)
        """
        # TODO for TP5

        path = [start, goal]  # list of poses
        return path

    def explore_frontiers(self):
        """ Frontier based exploration """
        goal = np.array([0, 0, 0])  # frontier to reach for exploration
        return goal
