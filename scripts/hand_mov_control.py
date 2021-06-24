#!/usr/bin/env python3

import numpy as np
import cv2
import mediapipe as mp
import time
import TELLO from TELLO 

TOL = 0.1

class interactive_tello_control:

    def __init__(self):
        self.cap = cv2.VideoCapture(0)  #captures default webcam
        sucess, self.img = self.cap.read()
        self.hands_detector = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.75, min_tracking_confidence=0.75)

    def detection_loop(self):
        results_hands = self.hands_detector.process(self.img)

        if results_hands.multi_hand_landmarks:
            hands = results_hands.multi_hand_landmarks
            x_0 = hands[0].landmark[0].x
            x_9 = hands[0].landmark[9].x
            x_12 = hands[0].landmark[12].x
            
            if x_12 - x_0 < TOL:
                print("Inclinado pra esquerda")
                #set_velocity(0.001, 0, 0)
                
            elif x_12 - x_0 > TOL:
                print("Inclinado pra direita")
                #set_velocity(-0.001, 0, 0)

            else:
                print("Não está inclinado")
                
            mp.solutions.drawing_utils.draw_landmarks(self.img, hands[0], mp.solutions.hands.HAND_CONNECTIONS)

    def main_interface(self):
        queue_modo = [0]
        modo_atual = 0
        while True:
            sucess, self.img = self.cap.read()
            self.detection_loop()
            cv2.imshow("image", self.img)
            cv2.waitKey(1)

c = interactive_tello_control()
c.main_interface()
