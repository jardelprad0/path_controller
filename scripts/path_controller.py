#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random
import math
import numpy as np
import time
from environment import Env

def alinha_robo(heading):
    orientation_error = heading

    if orientation_error > math.pi:
        orientation_error -= 2 * math.pi
    elif orientation_error < -math.pi:
        orientation_error += 2 * math.pi

    angular_velocity = 2.0 * orientation_error
    angular_velocity = np.clip(angular_velocity, -1.0, 1.0)

    return angular_velocity

def girar_E_e_parar(pub, r):
    angular_speed = 1.0  # Velocidade de rotação em rad/s
    target_angle = (math.pi) / 2  # Ângulo de rotação desejado em radianos
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = (current_time - start_time).to_sec()
        
        if elapsed_time >= target_angle / angular_speed:
            # Parar o robô
            velocity = Twist()
            pub.publish(velocity)
            break
        
        # Girar o robô
        velocity = Twist()
        velocity.angular.z = angular_speed
        pub.publish(velocity)
        r.sleep()

def girar_D_e_parar(pub, r):
    angular_speed = -1.0  # Velocidade de rotação em rad/s
    target_angle = -(math.pi) / 2  # Ângulo de rotação desejado em radianos
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = (current_time - start_time).to_sec()
        
        if elapsed_time >= target_angle / angular_speed:
            # Parar o robô
            velocity = Twist()
            pub.publish(velocity)
            break
        
        # Girar o robô
        velocity = Twist()
        velocity.angular.z = angular_speed
        pub.publish(velocity)
        r.sleep()

if __name__ == "__main__":
    rospy.init_node("path_controller_node", anonymous=False)

    env = Env()
    state_scan = env.reset()
    action = np.zeros(2)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    r = rospy.Rate(5)  # 5Hz

    while not rospy.is_shutdown():
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
            
        

        if objeto_frente > 0.5 and objeto_esquerda > 0.5 and objeto_direita > 0.5:
            angular_velocity = alinha_robo(env.heading)
            action = np.array([0.1, angular_velocity])
            
        elif objeto_frente < 0.5  and objeto_frente_esquerda:
            action[0] = 0
            girar_D_e_parar(pub, r)    
        elif objeto_frente < 0.5  and objeto_frente_direita:
            action[0] = 0
            girar_E_e_parar(pub, r)
                    
        if objeto_direita <= 0.50 and objeto_frente_direita <= 0.50 and objeto_tras_direita <= 0.50:
            action[0] = 0.1
            action[1] = 0.005
            if objeto_direita >= 0.5 and objeto_frente_direita >= 0.5 and objeto_tras_direita >= 0.5:
                action[0] = 0.1
                girar_E_e_parar(pub, r)
            elif objeto_direita <= 0.4 and objeto_frente_direita <= 0.3 and objeto_tras_direita <= 0.4:
                action[0] = 0.1
                action[1] = 0.1
            rospy.loginfo("objeto direita")
            
        elif objeto_esquerda <= 0.50 and objeto_frente_esquerda <= 0.50 and objeto_tras_esquerda <= 0.50:
            action[0] = 0.1
            action[1] = -0.005
            if objeto_esquerda >= 0.5 and objeto_frente_esquerda >= 0.5 and objeto_tras_esquerda >= 0.5:
                action[0] = 0.1
                girar_D_e_parar(pub, r)
            elif objeto_esquerda <= 0.4 and objeto_frente_esquerda <= 0.4 and objeto_tras_esquerda <= 0.4:
                action[0] = 0.1
                action[1] = -0.1
            rospy.loginfo("objeto esquerda")
    

        state_scan = env.step(action)

        r.sleep()
