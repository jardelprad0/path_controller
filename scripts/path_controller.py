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

def alinha_robo(heading):
    # Calcular o erro de orientação usando o valor de heading do ambiente
    orientation_error = heading

    # Normalizar o erro de orientação para estar no intervalo [-pi, pi]
    if orientation_error > math.pi:
        orientation_error -= 2 * math.pi
    elif orientation_error < -math.pi:
        orientation_error += 2 * math.pi

    # Calcular a velocidade angular com base no erro de orientação
    angular_velocity = 2.0 * orientation_error
    
    # Limitar a velocidade angular a uma faixa razoável
    angular_velocity = np.clip(angular_velocity, -1.0, 1.0)

    return angular_velocity  # Retorna apenas a velocidade angular

if __name__ == "__main__": 
    rospy.init_node("path_controller_node", anonymous=False)
    
    env = Env()
    state_scan = env.reset()
    action = np.zeros(2)
    min_distance = min(state_scan[:20])

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    
    r = rospy.Rate(5) # 10hz
    velocity = Twist()
while not rospy.is_shutdown():
    min_distance = min(state_scan[:20])
    
    if (min(state_scan[:20])) > 0.5: # Se não tiver obstáculo na frente
        angular_velocity = alinha_robo(env.heading)
        action = np.array([0.1, angular_velocity])  # Definir a ação com a velocidade angular
   

        
    state_scan = env.step(action)
            
    r.sleep()

