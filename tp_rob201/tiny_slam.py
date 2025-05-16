""" A simple robotics navigation code including SLAM, exploration, planning"""

import cv2
import numpy as np
from occupancy_grid import OccupancyGrid


class TinySlam:
    """Simple occupancy grid SLAM"""

    def __init__(self, occupancy_grid: OccupancyGrid):
        self.grid = occupancy_grid

        # Origin of the odom frame in the map frame
        self.odom_pose_ref = np.array([0, 0, 0])

    def _score(self, lidar, pose):
        """
        Computes the sum of log probabilities of laser end points in the map
        lidar : placebot object with lidar data
        pose : [x, y, theta] nparray, position of the robot to evaluate, in world coordinates
        """
        # TODO for TP4

        distances = lidar.get_sensor_values()
        angles = lidar.get_ray_angles()

        # supprimer les points à distance maximale
        valides = distances<lidar.max_range
        distances = distances[valides]
        angles = angles[valides]

        # transformation des points lidar dans le repère monde
        theta = pose[2]
        x_lidar = pose[0]+distances*np.cos(angles + theta)
        y_lidar = pose[1]+distances*np.sin(angles + theta)

        # conversion en indices de grille
        x_map, y_map = self.grid.conv_world_to_map(x_lidar, y_lidar)

        # verification des indices dans les bornes de la carte
        valid = (
            (x_map >= 0) & (x_map < self.grid.occupancy_map.shape[0]) &
            (y_map >= 0) & (y_map < self.grid.occupancy_map.shape[1])
        )
        x_map = x_map[valid]
        y_map = y_map[valid]

        # somme des valeurs dans la carte aux points correspondants
        score = np.sum(self.grid.occupancy_map[x_map, y_map])

        return score

    def get_corrected_pose(self, odom_pose, odom_pose_ref=None):
        """
        Compute corrected pose in map frame from raw odom pose + odom frame pose,
        either given as second param or using the ref from the object
        odom : raw odometry position
        odom_pose_ref : optional, origin of the odom frame if given, use self.odom_pose_ref if not given
        """
        # TODO for TP4
        if odom_pose_ref is None:
            odom_pose_ref = self.odom_pose_ref

        x, y, theta = odom_pose
        x_ref, y_ref, theta_ref = odom_pose_ref

        # rotation and translation
        x_corr = x_ref+np.cos(theta_ref)*x - np.sin(theta_ref)*y
        y_corr = y_ref+np.sin(theta_ref)*x + np.cos(theta_ref)*y
        theta_corr = theta_ref+theta

        corrected_pose = np.array([x_corr, y_corr, theta_corr])
        return corrected_pose

    def localise(self, lidar, raw_odom_pose):
        """
        Compute the robot position wrt the map, and updates the odometry reference
        lidar : placebot object with lidar data
        odom : [x, y, theta] nparray, raw odometry position
        """
        # TODO for TP4

        best_score = 0

        return best_score

    def update_map(self, lidar, pose):
        """
        Bayesian map update with new observation
        lidar : placebot object with lidar data
        pose : [x, y, theta] nparray, corrected pose in world coordinates
        """
        # TODO for TP3
        dist = lidar.get_sensor_values()
        angle = lidar.get_ray_angles()

        p_faible = np.log(0.01/0.99)
        p_fort = np.log(0.95/0.05)

        x = pose[0] + dist * np.cos(angle + pose[2])
        y = pose[1] + dist * np.sin(angle + pose[2])

        for i in range(0,len(dist)):
            xi = x[i]
            yi = y[i]

            self.grid.add_value_along_line(pose[0], pose[1], xi, yi, p_faible)
        
        self.grid.add_map_points(x, y, p_fort-p_faible)
        self.grid.occupancy_map = np.clip(self.grid.occupancy_map, -40, 40)
