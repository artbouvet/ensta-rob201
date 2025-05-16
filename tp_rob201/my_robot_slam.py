"""
Robot controller definition
Complete controller including SLAM, planning, path following
"""
import numpy as np

from place_bot.entities.robot_abstract import RobotAbstract
from place_bot.entities.odometer import OdometerParams
from place_bot.entities.lidar import LidarParams

from tiny_slam import TinySlam

from control import potential_field_control, reactive_obst_avoid
from occupancy_grid import OccupancyGrid
from planner import Planner


# Definition of our robot controller
class MyRobotSlam(RobotAbstract):
    """A robot controller including SLAM, path planning and path following"""

    def __init__(self,
                 lidar_params: LidarParams = LidarParams(),
                 odometer_params: OdometerParams = OdometerParams()):
        # Passing parameter to parent class
        super().__init__(should_display_lidar=False,
                         lidar_params=lidar_params,
                         odometer_params=odometer_params)

        # step counter to deal with init and display
        self.counter = 0

        # Init SLAM object
        # Here we cheat to get an occupancy grid size that's not too large, by using the
        # robot's starting position and the maximum map size that we shouldn't know.
        size_area = (1400, 1000)
        robot_position = (100.0, 100)
        x_min=-(size_area[0] / 2 + robot_position[0])
        x_max=size_area[0] / 2 - robot_position[0]
        y_min=-(size_area[1] / 2 + robot_position[1])
        y_max=size_area[1] / 2 - robot_position[1]
        self.occupancy_grid = OccupancyGrid(x_min=x_min, x_max=x_max, y_min=y_min, y_max=y_max, resolution=2)

        self.tiny_slam = TinySlam(self.occupancy_grid)
        self.planner = Planner(self.occupancy_grid)

        # storage for pose after localization
        self.corrected_pose = np.array([0, 0, 0])

        #initialization of the objective
        self.goal_pose = np.array([50.0, 50.0, 0.0])  #initial objective
        self.goal_reached_threshold = 30.0  #threshold to consider the goal reached


    def control(self):
        """
        Main control function executed at each time step
        """
        self.tiny_slam.update_map(self.lidar(), self.odometer_values())
        self.occupancy_grid.display_cv(self.odometer_values())
        return self.control_tp1() 

    def control_tp1(self):
        """
        Control function for TP1
        Control funtion with minimal random motion
        """
        # Compute new command speed to perform obstacle avoidance
        command = reactive_obst_avoid(self.lidar())
        return command

    def control_tp2(self):
        """
        Control function for TP2
        Main control function with full SLAM, random exploration and path planning
        """
        pose = self.odometer_values()
        goal = self.goal_pose
        dist_to_goal = np.linalg.norm(goal[:2] - pose[:2])

        if (dist_to_goal<self.goal_reached_threshold):
            print("Goal reached.")

            size_area = (1400, 1000)
            robot_position = (100.0, 100)

            x_min=-(size_area[0] / 2 + robot_position[0])
            x_max=size_area[0] / 2 - robot_position[0]
            y_min=-(size_area[1] / 2 + robot_position[1])
            y_max=size_area[1] / 2 - robot_position[1]

            new_x = np.random.uniform(x_min+100, x_max-100)
            new_y = np.random.uniform(y_min+100, y_max-100)

            self.goal_pose = np.array([new_x, new_y, 0.0])
            print("New goal: ", self.goal_pose)

        # Compute new command speed to perform obstacle avoidance
        command = potential_field_control(self.lidar(), pose, goal)

        return command
