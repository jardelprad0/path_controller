#! /usr/bin/env python3
import rospy 
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import random
import math
from gazebo_msgs.msg import *
import numpy as np
import csv
import rospkg
import matplotlib.pyplot as plt
from matplotlib import cm
import time
from environment import Env


if __name__ == "__main__": 
    rospy.init_node("path_controller_node", anonymous=False)
    
    env = Env()
    state_scan = env.reset()
    action = np.zeros(2)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    
    r = rospy.Rate(5) # 10hz
    velocity = Twist()
    while not rospy.is_shutdown():
        # Calcular o erro de orientação (diferença de ângulo) entre a direção atual do robô e o ângulo do objetivo
        goal_angle = math.atan2(env.goal_y - env.position.y, env.goal_x - env.position.x)
        orientation_error = goal_angle - env.heading

        # Normalizar o erro de orientação para estar no intervalo [-pi, pi]
        if orientation_error > math.pi:
            orientation_error -= 2 * math.pi
        elif orientation_error < -math.pi:
            orientation_error += 2 * math.pi

        # Calcular a velocidade angular com base no erro de orientação
        angular_velocity = 2.0 * orientation_error
        
        # Limitar a velocidade angular a um intervalo razoável
        angular_velocity = np.clip(angular_velocity, -1.0, 1.0)

        # Definir a ação para controlar apenas a velocidade angular
        action[0] = 0.0  # Sem movimento linear
        action[1] = angular_velocity
        
        state_scan = env.step(action)
                
        r.sleep()
