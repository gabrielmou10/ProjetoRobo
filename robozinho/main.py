#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda","Guilherme Aliperti", "Gabriel Moura"]
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

bridge = CvBridge()

# Varáveis Booleanas de Estados
coringao = False
caixa_cor = False
encontrou_algo = False
velzero= Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = (0,0)
centro = (0,0)
area = 0.0

# Tolerâncias  
tolerancia_x = 50
tolerancia_y = 20
ang_speed = 0.4
area_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 20000

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 0.9E9
check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

# Contador do roda frame
f = 0

#Rodandos os frames
def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro
    global area
    global coringao
    global caixa_cor
    global f
    f +=1


    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime
    delay = lag.nsecs
    # tratando o delay
    if delay > atraso and check_delay==True:
        print("delay {:.2f}".format(delay/1.0E9))
        return

    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # identifica o símbolo do corinthians
        media, centro, area, caixa_cor = cormodule.identifica_cor(cv_image)
        # 1 frame a cada 3
        if f%3 == 0:
            media_corin, coringao = corinthians.procuracor(cv_image)
        media_corin = (0,0)

        if media_corin[0] != 0:
            media = media_corin

        depois = time.clock()
        cv2.imshow("Camera", cv_image)

    except CvBridgeError as e:
        print('ex', e)

#Rodando o Scan
def roda_scan(dado):
    global dists
    global encontrou_algo
    dists = np.array(dado.ranges).round(decimals=2)
    # Distancias da frente da direita
    for distancia in dists[:30]:
        if distancia == None:
            encontrou_algo = False
        elif distancia < 0.3 and distancia > 0.11:
            encontrou_algo = True
            break
       	else:
            encontrou_algo = False
    # Distancias da frente da esquerda
    for distancia in dists[330:]:
        if distancia == None:
            encontrou_algo = False

        elif distancia < 0.3 and distancia > 0.11:
            encontrou_algo = True
            break
        else:
            encontrou_algo = False

#Reação de Sobreviencia - desvia se encontra algo
def scaneia(dists,velocidade_saida):
    global encontrou_algo
    global distancia
    #dists = (np.array(dado.ranges).round(decimals=2))
    vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

#distâncias a frenta direita
    for distancia in dists[330:]:
        if distancia < 0.3 and distancia > 0.11:
            print("Gira pra direita")
            encontrou_algo = True
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))

#distâncias a frente esquerda
    for distancia in dists[:30]:
        if distancia < 0.3 and distancia > 0.11:
            print("Gira pra esquerda")
            encontrou_algo = True
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -1))
    velocidade_saida.publish(vel)




## Classes - estados de máquina

#Classe Inicial - Parado
class Parando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['andando', 'sobrevivendo','girando'])

    def execute(self, userdata):

        velocidade_saida.publish(velzero)

        if encontrou_algo == True:
            return 'sobrevivendo'

        elif coringao == True:
            return 'andando'

        elif caixa_cor== True:
            return 'andando'

        else:
            return'girando'

#Classe Robo girando - balançando até alinhar com a cor / corinthians

class Girando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['andando', 'sobrevivendo','girando'])

    def execute(self, userdata):
        if encontrou_algo == True:
            return 'sobrevivendo'

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
            return 'andando'

# Classe de Sobreviencia - se identificar perigo roda a função de sobrevivencia (scaneou)

class Sobrevivendo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sobrevivendo', 'parando'])

    def execute(self, userdata):
        if encontrou_algo == True:
            print("encontrou")
            scaneia(dists, velocidade_saida)
            return 'sobrevivendo'
        else:
            return 'parando'

# Classe de Movimentação - seguindo a cor e se afastando do símbolo

class Andando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sobrevivendo','andando','parando'])

    def execute(self, userdata):
        global velocidade_saida

        #Aqui verificamos qual objeto esta na tela, dando prioridade para o simbolo do corinthians
        #A velocidade muda de acordo com o objeto na tela
        if encontrou_algo == True:
            return 'sobrevivendo'
        elif coringao == True:
            velocidade_reta = Vector3(-0.2,0,0)
            vel = Twist(velocidade_reta, Vector3(0,0,0))
            velocidade_saida.publish(vel)
            rospy.sleep(0.5)
            print("VAICORINTHIANS")
            return 'andando'
        elif caixa_cor == True:
            velocidade_reta = Vector3(0.1,0,0)
            vel = Twist(velocidade_reta, Vector3(0,0,0))
            velocidade_saida.publish(vel)
            rospy.sleep(0.5)
            print("VERMELHAO")
            return 'andando'
        else:
            return 'parando'


# Main - executa tudo

def main():
    global velocidade_saida
    global buffer
    rospy.init_node('cor_estados')

 # Recebe Scan e Câmera
    recebe_scan = rospy.Subscriber("/scan", LaserScan, roda_scan)
    recebe_cam = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    # Criando a SMACH state machine
    sm = smach.StateMachine(outcomes=['terminei'])

    with sm:
        smach.StateMachine.add('PARANDO', Parando(),
                                transitions={'andando': 'ANDANDO',
                                'girando':'GIRANDO','sobrevivendo':'SOBREVIVENDO'})
        smach.StateMachine.add('GIRANDO', Girando(),
                                transitions={'girando': 'GIRANDO',
                                'sobrevivendo':'SOBREVIVENDO','andando':'ANDANDO'})
        smach.StateMachine.add('SOBREVIVENDO', Sobrevivendo(),
                                transitions={'sobrevivendo': 'SOBREVIVENDO',
                                'parando':'PARANDO'})
        smach.StateMachine.add('ANDANDO', Andando(),
                                transitions={'sobrevivendo':'SOBREVIVENDO',
                                'andando':'ANDANDO','parando':'PARANDO'})
    # Executando a SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()