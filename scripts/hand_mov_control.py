#!/usr/bin/env python3

import numpy as np
import cv2
import mediapipe as mp
import time



class interactive_tello_control:

    def __init__(self):
        self.cap = cv2.VideoCapture(0)  #captures default webcam
        sucess, self.img = self.cap.read()
        self.hands_detector = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.75, min_tracking_confidence=0.75)
        self.face_detector = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.5)
    def detection_loop(self):
        results_hands = self.hands_detector.process(self.img)
        results_face = self.face_detector.process(self.img)

        if results_hands.multi_hand_landmarks:
            hands = results_hands.multi_hand_landmarks
            # Mao inclinada pra frente o z (12) fica negativo independente da mão
            # mao inclinada pra traz o z (12) fica positivo
            # proximo encontro vamos tentar fazer funções sem tello
            print("0:",hands[0].landmark[0]) # Numeração de 0 a 21 
            print("12:",hands[0].landmark[12])
            mp.solutions.drawing_utils.draw_landmarks(self.img, hands[0], mp.solutions.hands.HAND_CONNECTIONS)
        if results_face.detections:
            for detection in results_face.detections:
                mp.solutions.drawing_utils.draw_detection(self.img, detection)
            
        



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
