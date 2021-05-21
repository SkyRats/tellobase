import cv2
import mediapipe as mp
import time
import rospy
import numpy as np
import random

DEBUG = True

class interactive_tello_control:

    def __init__(self):
        self.cap = cv2.VideoCapture(0)  #captures default webcam
        sucess, self.img = self.cap.read()
        self.face_detector = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.75)
    
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
                detection_queue.clear()
                continue
            else:
                if len(detection_queue) >= 10:
                    break
                else:
                    detection_queue.append(False)

    def run_game(self):
        num_participantes = 10 #Número de participantes ainda aberto -> fazer com as mãos ?
        escolhido = random.randint(1,num_participantes)

c = interactive_tello_control()
c.run_game()
    
