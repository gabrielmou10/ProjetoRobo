import cv2
import numpy as np

#Mínimo para o Match
MIN_MATCH_COUNT=30

#Detectando SIFT
detector= cv2.xfeatures2d.SIFT_create()

FLANN_INDEX_KDITREE=0
flannParam=dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
flann=cv2.FlannBasedMatcher(flannParam,{})

trainImg=cv2.imread("coringao.png",0)
trainKP,trainDesc=detector.detectAndCompute(trainImg,None)


# Função que Procura o Símbolo do corinthians
def procuracor(QueryImgBGR):
        QueryImg=cv2.cvtColor(QueryImgBGR,cv2.COLOR_BGR2GRAY)
        queryKP,queryDesc=detector.detectAndCompute(QueryImg,None)
        matches=flann.knnMatch(queryDesc,trainDesc,k=2)
        # Varável de estado
        coringao = False
        # Matches
        goodMatch=[]
        for m,n in matches:
            if(m.distance < 0.75 * n.distance):
                goodMatch.append(m)
        if(len(goodMatch) > MIN_MATCH_COUNT):
            tp=[]
            qp=[]
            for m in goodMatch:
                tp.append(trainKP[m.trainIdx].pt)
                qp.append(queryKP[m.queryIdx].pt)
            tp,qp=np.float32((tp,qp))
            H,status=cv2.findHomography(tp,qp,cv2.RANSAC,3.0)
            if H is None:
                return (0,0) , coringao
            h,w=trainImg.shape
            trainBorder=np.float32([[[0,0],[0,h-1],[w-1,h-1],[w-1,0]]])
            queryBorder=cv2.perspectiveTransform(trainBorder,H)
            cv2.polylines(QueryImgBGR,[np.int32(queryBorder)],True,(0,255,0),5)
            #encontrando as coordenas medias de X para poder utilizar no cor_estados_gira
            #pegamos as coordenas dos 4 pontos do quadrado em volta da figura
            x00 = queryBorder[0][0][0]
            x01 = queryBorder[0][1][0]
            x10 = queryBorder[0][2][0]
            x11 = queryBorder[0][3][0]
            PontomedioX = (x00+x01+x10+x11)/4
            media = (PontomedioX, 0)
            coringao = True
        else:
            media = (0,0)
        cv2.imshow('result',QueryImgBGR)
        return media, coringao