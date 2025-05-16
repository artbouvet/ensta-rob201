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

    if max(cone)<50:
        return {"forward": -0.5, "rotation": last_turn*0.5}  # recule si obstacle trop proche

    if(min(cone) < 50):
        # Sens de rotation selon angle de l'obstacle
        left = np.mean(laser_dist[140:160])
        right = np.mean(laser_dist[200:220])
        if(left < right):
            # On tourne à droite
            rotation_speed = 0.3
            speed = 0.1
            last_turn = 1
        else:
            # On tourne à gauche
            rotation_speed = -0.3
            speed = 0.1
            last_turn = -1
    else:
        # On avance si absence d'obstacle en face
        speed = 0.4

        # ajout d'un biais aléatoire pour l’exploration
        rdm = np.random.rand()
        if (rdm<0.3):
            if(rdm<0.15):
                rotation_speed = (last_turn * 0.2)  # tourne légèrement vers la dernière direction utilisée
            else:
                rotation_speed = -(last_turn * 0.2)

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
    K_rep = 1500   # Gain for repulsion
    d_safe = 100  # Safe distance to obstacle
    d_stop = 20  # Stop distance to objective


    pos_q = current_pose[:2]
    pos_goal = goal_pose[:2]
    theta = current_pose[2]

    delta = pos_goal-pos_q
    d = np.linalg.norm(delta)

    if(d<1e-5):
        d = 1e-5 # to avoid division by zero

    # Attractive potential
    if(d<d_stop):
        grad_f_att = K_goal*delta #quadtratic potential
    else:
        grad_f_att = K_goal*delta/d #linear potential

    # Repulsive potential
    laser_dist = lidar.get_sensor_values()
    laser_angles = lidar.get_ray_angles()

    grad_f_rep = np.zeros(2)

    for dist,angle in zip(laser_dist, laser_angles):
        if 1.0 < dist < d_safe:
            # Obstacle position in robot frame
            x_obs = dist*np.cos(angle)
            y_obs = dist*np.sin(angle)
            q_obs = np.array([x_obs, y_obs])
            q_robot = np.array([0.0, 0.0])  # robot centered

            d_q_qobs = np.linalg.norm(q_obs-q_robot)
            if d_q_qobs < 1e-5:
                d_q_qobs= 1e-5  # éviter division par zéro

            repulsion = K_rep*((1.0/d_q_qobs)-(1.0/d_safe))/(d_q_qobs**3)*(q_robot - q_obs)
            grad_f_rep +=repulsion

    # Global potential
    grad_f = grad_f_att + grad_f_rep

    angle_to_grad = np.arctan2(grad_f[1], grad_f[0])
    angle_diff = angle_to_grad - theta

    # Angle normalization
    angle_diff = (angle_diff+np.pi)%(2*np.pi)-np.pi

    speed = np.clip(np.linalg.norm(grad_f), -1.0, 1.0)
    r_speed = np.clip(angle_diff, -1.0, 1.0)

    if d <d_stop:
        speed = 0.0
        r_speed = 0.0

    speed = round(speed, 2)
    r_speed = round(r_speed, 2)

    # debug
    #print("grad_att:", grad_f_att, "grad_rep:", grad_f_rep)
    #print("cmd:", speed, r_speed)

    command = {"forward": speed,
               "rotation": r_speed}

    return command

