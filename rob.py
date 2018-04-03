import rospy
import numpy as np
import tf
import math
import cv2
import cv2 as cv
import time
from matplotlib import pyplot as plt
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros

import cormodule

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
atraso = 1.5
check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados




def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global area

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, area = cormodule.identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)

## Classes - estados

class Girando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhou', 'girando'])

    #identifica a maior cor na faixa escolhida
    def identifica_cor(frame):
	'''
	Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
	'''

	# No OpenCV, o canal H vai de 0 até 179, logo cores similares ao
	# vermelho puro (H=0) estão entre H=-8 e H=8.
	# Precisamos dividir o inRange em duas partes para fazer a detecção
	# do vermelho:
	frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	cor_menor = np.array([0, 50, 50])
	cor_maior = np.array([8, 255, 255])
	segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

	cor_menor = np.array([172, 50, 50])
	cor_maior = np.array([180, 255, 255])
	segmentado_cor += cv2.inRange(frame_hsv, cor_menor, cor_maior)


	# A operação MORPH_CLOSE fecha todos os buracos na máscara menores
	# que um quadrado 7x7. É muito útil para juntar vários
	# pequenos contornos muito próximos em um só.
	segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

	# Encontramos os contornos na máscara e selecionamos o de maior área
	#contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	img_out, contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	maior_contorno = None
	maior_contorno_area = 0

	for cnt in contornos:
	    area = cv2.contourArea(cnt)
	    if area > maior_contorno_area:
	        maior_contorno = cnt
	        maior_contorno_area = area

	# Encontramos o centro do contorno fazendo a média de todos seus pontos.
	if not maior_contorno is None :
	    cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
	    maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
	    media = maior_contorno.mean(axis=0)
	    media = media.astype(np.int32)
	    cv2.circle(frame, tuple(media), 5, [0, 255, 0])
	else:
	    media = (0, 0)

    centro = (frame.shape[0]//2, frame.shape[1]//2)
    return media, centro

    def execute(self, userdata):
		global velocidade_saida

        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

        media, centro = identifica_cor(cv_image)[0] #encontra o ponto medio central da imagem no intervalo de cor definido
    #com base na media alinha o robo com o objeto
		if media is None or len(media)==0:
			return 'girando'

		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
			velocidade_saida.publish(vel)
			return 'girando'
		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
			velocidade_saida.publish(vel)
			return 'girando'
		else: #com o robo alinhado passa para o proximo estado, o "Centralizado"
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'alinhou'


class Centralizado(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhando', 'alinhado'])

    def execute(self, userdata):
		global velocidade_saida

		if media is None:
			return 'alinhou'
		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
			return 'alinhando'
		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			return 'alinhando'
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'alinhado'

# main
def main():
	global velocidade_saida
	global buffer
	rospy.init_node('cor_estados')

    #ap = cv2.VideoCapture(0)

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
