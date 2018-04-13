#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def scaneou(dado,velocidade_saida):
    #print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    #print("Leituras:")
    global dists
    global encontrou_algo

    dists = (np.array(dado.ranges).round(decimals=2))

#distâncias a frenta direita
    for distancia in dists[330:]:
        if distancia < 0.5 and distancia != 0.0:
            print(distancia)
            print("Gira pra direita")
            encontrou_algo = True
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))

#distâncias a frente esquerda
    for distancia in dists[:30]:
        if distancia < 0.5 and distancia != 0.0:
            print(distancia)
            print("Gira pra esquerda")
            encontrou_algo = True
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -1))

#distâncias a esquerda
    for distancia in dists[30:90]:
        if distancia < 0.5 and distancia != 0.0:
            print(distancia)
            print("Gira pra direita forte")
            encontrou_algo = True
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -1.5)

#distâncias a direita
    for distancia in dists[270:330]:
        if distancia < 0.5 and distancia != 0.0:
            print(distancia)
            print("Gira pra esquerda forte")
            encontrou_algo = True
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1.5))

#distâncias atras
    for distancia in dists[90:270]:
        if distancia < 0.5 and distancia != 0.0:
            print(distancia)
            print("vai pra frente")
            encontrou_algo = True
            vel = Twist(Vector3(1, 0, 0), Vector3(0, 0,0))
    velocidade_saida.publish(vel)


if __name__=="__main__":

    rospy.init_node("le_scan")

    #velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)




    while not rospy.is_shutdown():
        print("Oeee")
        #velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        #velocidade_saida.publish(velocidade)
        rospy.sleep(2)
