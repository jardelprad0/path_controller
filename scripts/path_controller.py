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

def girar_robo(angle_degrees):
    # Converter o ângulo de graus para radianos
    angle_rad = math.radians(angle_degrees)
    
    # Configurar a velocidade angular de acordo com o ângulo desejado
    angular_velocity = angle_rad / 2.0  # Ajuste conforme necessário
    
    # Girar o robô por um certo tempo para alcançar o ângulo desejado
    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration.from_sec(math.abs(angle_rad / angular_velocity)):
        velocity.angular.z = angular_velocity
        pub.publish(velocity)
        r.sleep()
    
    # Parar o movimento após atingir o ângulo desejado
    velocity.angular.z = 0.0
    pub.publish(velocity)

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
    ## separando laser scan ##
    objeto_frente_1 = min(state_scan[:22])
    objeto_frente_2 = min(state_scan[337:])
    
    objeto_frente = objeto_frente_1 + objeto_frente_2
    objeto_frente_esquerda = min(state_scan[22:67])
    objeto_esquerda = min(state_scan[68:111])
    objeto_tras_esquerda = min(state_scan[113:157])
    objeto_atras = min(state_scan[158:202])
    objeto_tras_direita = min(state_scan[203:247])
    objeto_direita = min(state_scan[248:292])
    objeto_frente_direita = min(state_scan[293:337])
    
    # min_distance = min(state_scan[:20])
    if objeto_frente > 0.5:
        angular_velocity = alinha_robo(env.heading)
        action = np.array([0.1, angular_velocity])  # Definir a ação com a velocidade angular
    else:
        action[0] = 0.0
        action[1] = 1.5
        # if objeto_direita < 0.5: # Se não tiver obstáculo na frente
        #     action[0] = 0.1
        #     action[1] = 0.0
        #     rospy.loginfo("objeto na Direita")
            
        # elif objeto_esquerda < 0.5:
        #     action[0] = 0.1
        #     action[1] = 0.0
        #     rospy.loginfo("objeto na Esquerda")
            
        # elif objeto_atras < 0.5:
        #     action[0] = 0.3
        #     action[1] = 0.0
        #     rospy.loginfo("objeto atrás")
              
  
    state_scan = env.step(action)
            
    r.sleep()
    state_scan = env.step(action)
            
    r.sleep()

