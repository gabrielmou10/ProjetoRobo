#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]

#Importando os módulos necessários
import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros

import cormodule
import corinthians
import sobreviencia

bridge = CvBridge()

cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = []
centro = []
area = 0.0



tolerancia_x = 50
tolerancia_y = 20
ang_speed = 0.4
area_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 20000

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 1.1E9
check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados
#Variaveis do corinthians.py
N_MATCH_COUNT=10

detector= cv2.xfeatures2d.SIFT_create()


FLANN_INDEX_KDITREE=0
flannParam=dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
flann=cv2.FlannBasedMatcher(flannParam,{})

trainImg=cv2.imread("coringao.png",0)
trainKP,trainDesc=detector.detectAndCompute(trainImg,None)



#Rodandos os frames
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global area
    global coringao
    global caixa_cor

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime
    delay = lag.secs
    if delay > atraso and check_delay==True:
        return
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        media, centro, area, caixa_cor = cormodule.identifica_cor(cv_image)
        media_corin, coringao = corinthians.procuracor(cv_image)
        if media[0] == 0:
            media = media_corin
        depois = time.clock()
        cv2.imshow("Camera", cv_image)
    except CvBridgeError as e:
        print('ex', e)






## Classes - estados

#Robo girando(balançando) até alinhar com a cor

class Girando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhou', 'girando'])

    def execute(self, userdata):
        global velocidade_saida

        if media is None or len(media)==0:
            return 'girando'

        if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
            velocidade_saida.publish(vel)
            rospy.sleep(0.5)
            return 'girando'
        if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
            velocidade_saida.publish(vel)
            rospy.sleep(0.5)
            return 'girando'
        else:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            velocidade_saida.publish(vel)
            rospy.sleep(0.5)
            return 'alinhou'

# Se o valor da média(centro de todos os pontos) está entre o valor do centro da visão do robo e a tolerancia então o robo alinha

# Robo centralizando a cor com a sua visão e seguindo em direção

class Centralizado(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhando', 'alinhado'])

    def execute(self, userdata):
        global velocidade_saida

        #Aqui verificamos qual objeto esta na tela, dando prioridade para o simbolo do corinthians
        #A velocidade muda de acordo com o objeto na tela
        if coringao == True:
            velocidade_reta = Vector3(1,0,0)
        elif caixa_cor == True:
            velocidade_reta = Vector3(-1,0,0)
        else:
            velocidade_reta = Vector3(0,0,0)

        if media is None:
            return 'alinhou'
        if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
            return 'alinhando'
        if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
            return 'alinhando'
        else:
            vel = Twist(velocidade_reta, Vector3(0, 0, 0))
            velocidade_saida.publish(vel)
            rospy.sleep(0.5)
            return 'alinhado'

# main - executa tudo
def main():
    global velocidade_saida
    global buffer
    rospy.init_node('cor_estados')

    # Para usar a webcam
    #recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
    recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['terminei'])

    # Open the container
    with sm:
        # Add states to the container
        #smach.StateMachine.add('LONGE', Longe(),
        #                       transitions={'ainda_longe':'ANDANDO',
        #                                    'perto':'terminei'})
        #smach.StateMachine.add('ANDANDO', Andando(),
        #                       transitions={'ainda_longe':'LONGE'})
        smach.StateMachine.add('GIRANDO', Girando(),
                                transitions={'girando': 'GIRANDO',
                                'alinhou':'CENTRO'})
        smach.StateMachine.add('CENTRO', Centralizado(),
                                transitions={'alinhando': 'GIRANDO',
                                'alinhado':'CENTRO'})


    # Execute SMACH plan
    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    main()
