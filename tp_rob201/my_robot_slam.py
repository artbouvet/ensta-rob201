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

        self.first_update_done = False

        

    def control(self):
        """
        Main control function executed at each time step
        """

        
               
        return self.control_tp3()


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
            # si objectif atteint : on fixe un nouvel objectif aléatoire
            x_min=-(size_area[0] /2+robot_position[0])
            x_max=size_area[0]/2-robot_position[0]
            y_min=-(size_area[1] /2+robot_position[1])
            y_max=size_area[1] /2-robot_position[1]

            new_x = np.random.uniform(x_min+100, x_max-100)
            new_y = np.random.uniform(y_min+100, y_max-100)

            self.goal_pose = np.array([new_x, new_y, 0.0])
            print("New goal: ", self.goal_pose)

        command = potential_field_control(self.lidar(), pose, goal)

        return command
    
    def control_tp3(self):
        command = self.control_tp1()
        self.tiny_slam.update_map(self.lidar(), self.odometer_values())
        self.occupancy_grid.display_cv(self.odometer_values())
        return command

    def control_tp4(self):
        raw_pose = self.odometer_values()

        if not self.first_update_done:
            # Première mise à jour forcée
            self.corrected_pose = self.tiny_slam.get_corrected_pose(raw_pose)
            self.tiny_slam.update_map(self.lidar(), self.corrected_pose)
            self.occupancy_grid.display_cv(self.corrected_pose)
            self.first_update_done = True
            print("Première mise à jour de la carte forcee")
            return self.control_tp1()

        # SLAM standard ensuite
        score = self.tiny_slam.localise(self.lidar(), raw_pose)
        print(f"score: {score}")

        if score > 100:
            self.corrected_pose = self.tiny_slam.get_corrected_pose(raw_pose)
            self.tiny_slam.update_map(self.lidar(), self.corrected_pose)
            self.occupancy_grid.display_cv(self.corrected_pose)
        else:
            self.occupancy_grid.display_cv(raw_pose)
        command = self.control_tp1()
        
        return command

    """
    NON FONCTIONNEL
    def control_tp5(self):
    

        # nbre d'iterations pour cartographie
        NB_ITER = 100

        lidar = self.lidar()
        odo = self.odometer_values()
        self.tiny_slam.update_map(lidar, odo)
        score = self.tiny_slam.localise(lidar, odo)

        self.counter += 1
        print("iteration :", self.counter)

        self.corrected_pose = self.tiny_slam.get_corrected_pose(odo)

        # on détermine l'objectif en fonction de la phase d'exploration ou de retour
        if self.counter < NB_ITER:
            # Phase d'exploration : objectif fixe
            goal = [-800, 0, 0]
        else:
            # Phase de retour : planification et suivi du chemin
            if self.counter == NB_ITER:
                start = self.corrected_pose[:2]
                goal = [0, 0,0]  #retour à la position initiale
                path = self.planner.plan(start, goal)
                if path:
                    print("Chemin trouvé :", path)
                    self.path = path
                    self.occupancy_grid.display_cv(odo, goal=goal, traj=path)

            # Suivi du chemin planifié
            if hasattr(self, 'path') and self.path:
                goal = self.path.pop(0)  # prochain point à atteindre


        if self.counter % 2 == 0:
            self.occupancy_grid.display_cv(odo, goal=goal)

        return self.control_tp2()
        """