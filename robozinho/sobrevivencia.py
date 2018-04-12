#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def scaneou(dado):
    #print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    #print("Leituras:")
    dists = (np.array(dado.ranges).round(decimals=2))

#distâncias a frenta direita
    for distancia in dists[330:]:
        if distancia < 0.5 and distancia != 0.0:
            print("Gira pra direita")
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))
        velocidade_saida.publish(vel)

#distâncias a frente esquerda
    for distancia in dists[:30]:
        if distancia < 0.5 and distancia != 0.0:
            print("Gira pra esquerda")
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -1))
        velocidade_saida.publish(vel)

#distâncias a esquerda
    for distancia in dists[30:90]:
        if distancia < 0.5 and distancia != 0.0:
            print("Gira pra esquerda")
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -3))
        velocidade_saida.publish(vel)

#distâncias a direita
    for distancia in dists[270:330]:
        if distancia < 0.5 and distancia != 0.0:
            print("Gira pra esquerda")
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 3))
        velocidade_saida.publish(vel)

#distâncias atras
    for distancia in dists[90:270]:
        if distancia < 0.5 and distancia != 0.0:
            print("Gira pra esquerda")
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
