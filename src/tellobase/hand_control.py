#!/usr/bin/env python3

import cv2
import mediapipe as mp
import time
from TELLO import TELLO
import rospy
import numpy as np

DEBUG = True
DRONE = True

class interactive_tello_control:

    def __init__(self):
        if DRONE:
            rospy.init_node("interactive_tello_control")
            self.tello = TELLO("robinho pequeno")
            image_message = self.tello.image
            self.img = np.frombuffer(image_message.data, dtype=np.uint8).reshape(image_message.height, image_message.width, 3)
        else:
            self.cap = cv2.VideoCapture(0)  #captures default webcam
            sucess, self.img = self.cap.read()
        self.hands_detector = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.75, min_tracking_confidence=0.75)

    def detection_loop(self):
        results = self.hands_detector.process(self.img)
        if results.multi_hand_landmarks:
            hands = results.multi_hand_landmarks
            finger_points = self.fingerPositions(hands)
            handedness = self.determine_handedness(finger_points)

            if DEBUG:
                mp.solutions.drawing_utils.draw_landmarks(self.img, hands[0], mp.solutions.hands.HAND_CONNECTIONS)

            return handedness, finger_points
        return 0, 0

    def fingerPositions(self,hands):
        #Transforma a posicao dos dedos em (x,y) da imagem
        finger_list = []
        for id, lm in enumerate(hands[0].landmark):
            height, width, useless = self.img.shape
            x_point, y_point = int(lm.x * width), int(lm.y * height)
            finger_list.append([x_point, y_point])
        return finger_list

    def determine_handedness(self,finger_points): #what about upside-down or backhands ?
        #Determina qual a mao que esta sendo utilizada, a esquerda ou a direita
        if finger_points[4][0] < finger_points[17][0]:
            return "right_hand"
        else:
            return "left_hand"

    def fingers_up(self, handedness, finger_list):
        #Interpreta as posicoes dos dedos para ver quais estao abertos e quais estao fechados, devolve uma lista do GESTO feito
        gesto = []
        if finger_list[5][1] < finger_list[8][1]:
            indicador = False
        else:
            gesto.append("indicador")
        if finger_list[9][1] < finger_list[12][1]:
            meio = False
        else:
            gesto.append("meio")
        if finger_list[13][1] < finger_list[16][1]:
            anelar = False
        else:
            gesto.append("anelar")
        if finger_list[17][1] < finger_list[20][1]:
            mindinho = False
        else:
            gesto.append("mindinho")
        if finger_list[2][0] < finger_list[4][0] and handedness == "right_hand":
            dedao = False
        elif finger_list[2][0] > finger_list[4][0] and handedness == "right_hand":
            gesto.append("dedao")
        if finger_list[2][0] > finger_list[4][0] and handedness == "left_hand":
            dedao = False
        if finger_list[2][0] < finger_list[4][0] and handedness == "left_hand":
            gesto.append("dedao")

        return gesto


    def hand_interface(self):
        queue_modo = [0]
        while True:
            if DRONE:
                image_message = self.tello.image
                self.img = np.frombuffer(image_message.data, dtype=np.uint8).reshape(image_message.height, image_message.width, -1)
            else:
                sucess, self.img = self.cap.read()

            handedness, finger_points = self.detection_loop()

            if DEBUG:
                cv2.imshow("image", self.img)
                cv2.waitKey(1)

            if handedness == 0: #detection_loop() found nothing
                continue
            gesto = self.fingers_up(handedness, finger_points)

            ########## Colocar os gestos abaixo! ##########
            if gesto == ['indicador']: #Modo 1
                if queue_modo[0] != 1:
                    queue_modo.clear()
                    queue_modo.append(1)
                else:
                    queue_modo.append(1)

            elif gesto == ['indicador', 'meio']: #Modo 2
                if queue_modo[0] != 2:
                    queue_modo.clear()
                    queue_modo.append(2)
                else:
                    queue_modo.append(2)

            elif gesto == ['indicador', 'meio', 'anelar']: #Modo 3
                if queue_modo[0] != 3:
                    queue_modo.clear()
                    queue_modo.append(3)
                else:
                    queue_modo.append(3)


            else:
                queue_modo.clear()
                queue_modo.append(0)
            ########## Colocar os gestos acima! ##########


            if len(queue_modo) >= 20:
                mode = queue_modo[0]
                return mode

    def main(self):
        while not rospy.is_shutdown():
            mode = self.hand_interface()
            self.run_hand_mode(mode)

    def run_hand_mode(self,mode):
        print("rodando modo " + str(mode))
        if mode == 1 and DRONE:
            self.takeoff_finger()
        print("modo " + str(mode) + " encerrado, manda proximo gesto!")

    def takeoff_finger(self):
        self.tello.takeoff()

c = interactive_tello_control()
c.main()