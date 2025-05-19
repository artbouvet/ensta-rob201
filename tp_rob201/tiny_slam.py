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

        dist = np.array(lidar.get_sensor_values())
        angles = np.array(lidar.get_ray_angles())

        x_lidar = pose[0] + dist *np.cos(pose[2]+angles)
        y_lidar = pose[1] + dist *np.sin(pose[2]+angles)

        i_max = np.where(dist == np.max(dist))
        x_lidar = np.delete(x_lidar, i_max)
        y_lidar = np.delete(y_lidar, i_max)

        # conversion des coordonnées
        x_map, y_map = self.grid.conv_world_to_map(x_lidar, y_lidar)
        
        # suppression des points en dehors de la carte
        valid = ((x_map >= 0) & (x_map <self.grid.x_max_map) &(y_map >= 0)& (y_map< self.grid.y_max_map))
        x_map = x_map[valid]
        y_map = y_map[valid]

        # calcul du score
        score = np.sum(self.grid.occupancy_map[x_map, y_map])

        return score

    def get_corrected_pose(self, odom_pose, odom_pose_ref=None):
        """
        Compute corrected pose in map frame from raw odom pose + odom frame pose,
        either given as second param or using the ref from the object
        odom : raw odometry position
        odom_pose_ref : optional, origin of the odom frame if given,
                        use self.odom_pose_ref if not given
        """
        # TODO for TP4
        if odom_pose_ref is None:
            odom_pose_ref = self.odom_pose_ref

        # Décomposition
        x_o, y_o, theta_o = odom_pose
        x_r, y_r, theta_r = odom_pose_ref

        # Transformation rigide : on applique l'odométrie dans la base de la référence
        corrected_x = x_r + x_o *np.cos(theta_r)- y_o *np.sin(theta_r)
        corrected_y = y_r + x_o *np.sin(theta_r)+ y_o *np.cos(theta_r)
        corrected_theta = theta_r + theta_o

        return np.array([corrected_x, corrected_y, corrected_theta])

    def localise(self, lidar, raw_odom_pose):
        """
        Compute the robot position wrt the map, and updates the odometry reference
        lidar : placebot object with lidar data
        odom : [x, y, theta] nparray, raw odometry position
        """
        # TODO for TP4

        best_score = self._score(lidar, self.get_corrected_pose(raw_odom_pose))
        best_ref = self.odom_pose_ref.copy()
        sigma = 1
        for i in range(150):
            offset = np.random.normal(0, sigma, 3)
            test_ref = self.odom_pose_ref + offset
            new_score = self._score(lidar, self.get_corrected_pose(raw_odom_pose, test_ref))

            if new_score > best_score:
                best_score = new_score
                best_ref = test_ref  # Mémorise la meilleure ref trouvée
            i += 1
        self.odom_pose_ref = best_ref
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

        i_max = np.where(dist >= np.max(dist)-1e-2)

        x_lidar = pose[0] + dist*np.cos(angle + pose[2])
        y_lidar = pose[1] + dist*np.sin(angle + pose[2])

        x = pose[0] + 0.9 *dist*np.cos(angle + pose[2])
        y = pose[1] + 0.9*dist*np.sin(angle + pose[2])

        for i in range(0,150):             
            xi = x[i]
            yi = y[i]

            self.grid.add_value_along_line(pose[0],pose[1],xi,yi,-2)

        x_lidar = np.delete(x_lidar,i_max)
        y_lidar = np.delete(y_lidar,i_max)

        self.grid.add_map_points(x, y, 4)
        self.grid.occupancy_map = np.clip(self.grid.occupancy_map, -40, 40)
