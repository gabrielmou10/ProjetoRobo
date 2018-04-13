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
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros

import cormodule
import corinthians
import sobrevivencia

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

encontrou_algo = False 
coringao = False 
caixa_cor = False

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
        if media_corin[0] != 0:
            media = media_corin
        depois = time.clock()
        cv2.imshow("Camera", cv_image)
    except CvBridgeError as e:
        print('ex', e)

def roda_scan(dado):
    global dists
    global encontrou_algo
    print("chegoulaser")
    dists = np.array(dado.ranges).round(decimals=2)
    for distancia in dists[:30]:
        if distancia == None:
            encontrou_algo = False

        elif distancia < 0.4 and distancia > 0.1:
            print(distancia)
            print
            encontrou_algo = True
            rospy.sleep(0.05);
        else:
            print("nao achou")
            encontrou_algo = False
            rospy.sleep(0.05)

def scaneia(dists,velocidade_saida):
    #print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    #print("Leituras:")z
    global encontrou_algo
    #global velocidade_saida
    global distancia

    #dists = (np.array(dado.ranges).round(decimals=2))
    vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

#distâncias a frenta direita
    for distancia in dists[330:]:
        if distancia < 0.5 and distancia > 0.11:
            print(distancia)
            print("Gira pra direita")
            encontrou_algo = True
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))

#distâncias a frente esquerda
    for distancia in dists[:30]:
        if distancia < 0.5 and distancia > 0.11:
            print(distancia)
            print("Gira pra esquerda")
            encontrou_algo = True
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -1))
    velocidade_saida.publish(vel)




## Classes - estados

#Robo girando(balançando) até alinhar com a cor

class Girando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhou', 'girando'])

    def execute(self, userdata):

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

class Sobrevivendo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['achou', 'naoachou'])

    def execute(self, userdata):
        if encontrou_algo == True:
            print("encontrou")
            scaneia(dists, velocidade_saida)
            return 'achou'
        else:
            return 'naoachou'

class Andando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sobrevivendo','alinhando','andando'])

    def execute(self, userdata):
        global velocidade_saida

        #Aqui verificamos qual objeto esta na tela, dando prioridade para o simbolo do corinthians
        #A velocidade muda de acordo com o objeto na tela
        if encontrou_algo == True:
            return 'sobrevivendo'
        elif coringao == True:
            velocidade_reta = Vector3(-0.4,0,0)
        elif caixa_cor == True:
            velocidade_reta = Vector3(0.4,0,0)
        else:
            velocidade_reta = Vector3(0,0,0)
            #return 'alinhando'

        if media is None:
            return 'andando'
        if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
            return 'alinhando'
        if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
            return 'alinhando'
        else:
            vel = Twist(velocidade_reta, Vector3(0, 0, 0))
            velocidade_saida.publish(vel)
            rospy.sleep(0.5)
            return 'andando'


# main - executa tudo
def main():
    global velocidade_saida
    global buffer
    rospy.init_node('cor_estados')

    # Para usar a webcam
    #recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, roda_scan)
    recebe_cam = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)
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
        smach.StateMachine.add('SOBREVIVE', Sobrevivendo(),
                                transitions={'achou': 'SOBREVIVE',
                                'naoachou':'CENTRO'})
        smach.StateMachine.add('CENTRO', Andando(),
                                transitions={'alinhando': 'GIRANDO',
                                'andando':'CENTRO','sobrevivendo':'SOBREVIVE'})
    # Execute SMACH plan
    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    main()