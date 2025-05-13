""" A set of robotics control functions """

import random
import numpy as np

last_turn = 1  # Mémoire courte
def reactive_obst_avoid(lidar):
    """
    Simple obstacle avoidance
    lidar : placebot object with lidar data
    management of blocking situations : add of a small random biais
    """
    # TODO for TP1

    global last_turn

    laser_dist = lidar.get_sensor_values()
    speed = 0.0
    rotation_speed = 0.0

    # On regarde si un obstacle est détecté dans un cone
    cone = laser_dist[140:220]

    if(min(cone) < 50):
        # Sens de rotation selon angle de l'obstacle
        left = np.mean(laser_dist[140:160])
        right = np.mean(laser_dist[200:220])
        if(left < right):
            # On tourne à droite
            rotation_speed = 0.3
            last_turn = 1
        else:
            # On tourne à gauche
            rotation_speed = -0.3
            last_turn = -1
    else:
        # On avance si absence d'obstacle en face
        speed = 0.4

        # ajout d'un biais aléatoire pour l’exploration
        if (np.random.rand()<0.01):
            rotation_speed = (last_turn * 0.2)  # tourne légèrement vers la dernière direction utilisée



    command = {"forward": speed,
               "rotation": rotation_speed}

    return command


def potential_field_control(lidar, current_pose, goal_pose):
    """
    Control using potential field for goal reaching and obstacle avoidance
    lidar : placebot object with lidar data
    current_pose : [x, y, theta] nparray, current pose in odom or world frame
    goal_pose : [x, y, theta] nparray, target pose in odom or world frame
    Notes: As lidar and odom are local only data, goal and gradient will be defined either in
    robot (x,y) frame (centered on robot, x forward, y on left) or in odom (centered / aligned
    on initial pose, x forward, y on left)
    """


    K_goal = 0.4  # Gain for goal attraction
    K_rep = 400000   # Gain for repulsion
    d = np.sqrt((goal_pose[0] - current_pose[0])**2 + (goal_pose[1] - current_pose[1])**2)

    d_obs = lidar.get_sensor_values()[160:200].min()

    if d > 3:
        grad_f_att = K_goal * (goal_pose - current_pose) / d

        d_safe = 50
        if d_obs>d_safe:
            grad_f_rep = np.zeros(3)
            r_speed = 0.0
        else:
            grad_f_rep = K_rep/d**3 * (1/d_obs - 1/d_safe) * (goal_pose - current_pose)

            repulsive_direction = np.arctan2(grad_f_rep[0], grad_f_rep[1])
            r_speed = K_rep * (repulsive_direction - current_pose[2])
        print(grad_f_att, grad_f_rep)
        grad_f = grad_f_att + grad_f_rep

        speed = np.linalg.norm(grad_f[0:1]) 
        speed = max(min(speed, 1.0), -1.0)


        # Normalize the rotation speed to the range [-pi, pi]
        r_speed = (r_speed + np.pi) % (2 * np.pi) - np.pi

        # Ensure the rotation speed is within the range [-1, 1]
        r_speed = max(min(r_speed, 1.0), -1.0)

    else:
        speed = 0.0
        r_speed = 0.0

    print(speed, r_speed)
    speed = round(speed, 2)
    r_speed = round(r_speed, 2)

    command = {"forward": speed,
               "rotation": r_speed}

    return command
