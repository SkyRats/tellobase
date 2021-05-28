#!/usr/bin/env python

import cv2
import mediapipe as mp
import time
#from TELLO import TELLO
import rospy
import numpy as np
import random

DEBUG = True
DRONE = False

class interactive_tello_control:

    def __init__(self):
        self.cap = cv2.VideoCapture(0)  #captures default webcam
        sucess, self.img = self.cap.read()
        rospy.init_node('face_game')
        if DRONE:
            self.tello = TELLO("Robin ;-;")
        self.face_detector = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.75)
        self.rate = rospy.Rate(60)
        self.rate.sleep()
        
    def face_detection_loop(self):
        results = self.face_detector.process(self.img)
        if results.detections:
            for id,detection in enumerate(results.detections):
                if DEBUG:
                    mp.solutions.drawing_utils.draw_detection(self.img, detection)
        return results.detections

    def face_interface(self): #problemas com multiplas pessoas -> guardar rostos (?)
        detection_queue = []
        first_detection = False
        while True:
            if DRONE:
                image_message = self.tello.image
                self.img = np.frombuffer(image_message.data, dtype=np.uint8).reshape(image_message.height, image_message.width, -1)
            else:
                sucess, self.img = self.cap.read()
            
            detected = self.face_detection_loop()

            if DEBUG:
                cv2.imshow("image", self.img)
                cv2.waitKey(1)

            if not first_detection:
                if detected:
                    first_detection = True
                    continue
                else:
                    continue

            if detected:
                if(self.pessoa +1 == self.escolhido):
                    return True
                detection_queue.clear()
                continue
            else:
                if len(detection_queue) >= 10:
                    return True
                else:
                    detection_queue.append(False)

    def run_game(self):
        num_participantes = 4 #Numero de participantes ainda aberto -> fazer com as maos ?
        self.escolhido = random.randint(1,num_participantes)
        print("Participante escolhido:", self.escolhido)
        if DRONE:
            for i in range(80):
                self.tello.takeoff()
                for i in range(45):
                    self.tello.set_velocity(0, 0, 0.5, 0, 0, 0)
                    rospy.rate.sleep()

        self.pessoa = 0
        if DRONE:
            self.tello.set_velocity(0, 0, 0, 0, 0, 0.5) # vai girar no sentido anti-horarios
        while self.pessoa < self.escolhido:
            if self.face_interface():
                self.pessoa += 1
                print("foto de agora", self.pessoa)
        print("Escolhi voce - Clarice Falcao")
                
        if DRONE:
            self.tello.set_velocity(0, 0, 0, 0, 0, 0) # para o tello
        
        
        
if __name__ == "__main__":
    c = interactive_tello_control()
    c.run_game()
    
