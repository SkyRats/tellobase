import cv2,time
from matplotlib.pyplot import show
import mediapipe as mp
import pygame as pg
from djitellopy import Tello


class hand_tello_control:
    def __init__(self):

        self.action = " "
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        self.action_done = True
        self.ffoward = 1
        self.fback = 1
        self.fright = 1
        self.fleft = 1
        self.fsquare = 1
        self.fmiddle = 1
        self.fno = 1
        self.fland = 1
        self.tricks = True

    def tello_startup(self):
        # For Tello input:
        self.tello = Tello()  # Starts the tello object
        self.tello.connect()  # Connects to the drone


    def define_orientation(self, results):

        if results.multi_hand_landmarks[0].landmark[4].x < results.multi_hand_landmarks[0].landmark[17].x:
            orientation = "right hand"
        else:
            orientation = "left hand"
        return orientation

    def follow_hand(self, results):
        normalizedLandmark = results.multi_hand_landmarks[0].landmark[9] # Normalizes the lowest middle-finger coordinate for hand tracking
        pixelCoordinatesLandmark = self.mp_drawing._normalized_to_pixel_coordinates(normalizedLandmark.x, normalizedLandmark.y, 255, 255) #Tracks the coordinates of the same landmark in a 255x255 grid
        print(pixelCoordinatesLandmark)
        
        if pixelCoordinatesLandmark == None: # If hand goes out of frame, stop following
            self.tello.send_rc_control(0,0,0,0)
            return
        
        
        centerRange = [12,12] #Range for detecting commands in the x and y axis.
        centerPoint = [128,128] #Theoretical center of the image
        xCorrection = pixelCoordinatesLandmark[0] - centerPoint[0] 
        yCorrection = pixelCoordinatesLandmark[1] - centerPoint[1]
        xSpeed = 0
        ySpeed = 0

        if xCorrection > centerRange[0] or xCorrection < -centerRange[0]: #If the hand is outside the acceptable range, changes the current speed to compensate.
            xSpeed = xCorrection//3
        if yCorrection > centerRange[1] or yCorrection < -centerRange[1]:
            ySpeed = -yCorrection//3


        self.tello.send_rc_control(xSpeed,0,ySpeed,0)
        time.sleep(0.5)
        self.tello.send_rc_control(0,0,0,0)
        
        
    
    def action_to_do(self, fingers, orientation, results): #use the variable results for the hand tracking control
        
        if self.action_done == True:
            self.ffoward = 1
            self.fback = 1
            self.fright = 1
            self.fleft = 1
            self.fsquare = 1
            self.fmiddle = 1
            self.fno = 1
            self.fland = 1
            self.action_done = False
        #Left hand controls tricks, right hand controls movement
        if orientation == "left hand":   #Thumb on the left = left hand!
            if fingers == [0, 1, 0, 0, 0]:
                if self.ffoward >= 15:
                    self.action = "flip forward"
                    self.tello.flip_forward() 
                    self.action_done = True
                self.ffoward = self.ffoward + 1

            elif fingers == [0, 1, 1, 0, 0] and self.tricks ==  True:
                if self.fback >= 15:
                    self.action = "flip back"
                    self.tello.flip_back()
                    self.action_done = True
                self.fback = self.fback + 1

            elif fingers == [1, 0, 0, 0, 0] and self.tricks ==  True:
                if self.fright >= 15:
                    self.action = "flip right"
                    self.tello.flip_right()
                    self.action_done = True
                self.fright = self.fright + 1

            elif fingers == [0, 0, 0, 0, 1] and self.tricks ==  True:
                if self.fleft >= 15:
                    self.action = "flip left"
                    self.tello.flip_left()
                    self.action_done = True
                self.fleft = self.fleft + 1

            elif fingers == [0, 1, 1, 1, 0]:
                if self.fsquare >= 15:
                    self.action = "Square"
                    self.tello.move_left(20)
                    self.tello.move_up(40)
                    self.tello.move_right(40)
                    self.tello.move_down(40)
                    self.tello.move_left(20)
                    self.action_done = True
                self.fsquare = self.fsquare + 1

            elif fingers == [0, 0, 1, 0, 0]:
                if self.fmiddle >= 15:
                    self.action = " :( "
                    self.tello.land()
                    self.action_done = True
                self.fmiddle = self.fmiddle + 1  
                
            elif ((self.tricks == False) and (fingers == [1, 0, 0, 0, 0] or fingers == [0, 1, 0, 0, 0] or fingers == [0, 0, 0, 0, 1])): #not avaiable to do tricks
                if self.fno >= 15:
                    self.tello.rotate_clockwise(45)
                    self.tello.rotate_counter_clockwise(90)
                    self.tello.rotate_clockwise(45)
                    self.action_done = True
                self.fno = self.fno + 1      
            else:
                self.action = " "

        elif orientation == "right hand":  #Thumb on the right = right hand!
            if fingers == [1, 1, 1, 1, 1]:
                self.action = "Follow"
                self.follow_hand(results)
            elif fingers == [1, 0, 0, 0, 0]:
                if self.fland >= 15:
                    self.action = "Land"
                    self.tello.land()
                    self.action_done = True
                self.fland = self.fland + 1           
            else:
                self.action = " "
                


    def fingers_position(self, results, orientation):

        # [thumb, index, middle finger, ring finger, pinky]
        # 0 for closed, 1 for open
        fingers = [0, 0, 0, 0, 0]

        if (results.multi_hand_landmarks[0].landmark[4].x > results.multi_hand_landmarks[0].landmark[
            3].x) and orientation == "right hand":
            fingers[0] = 0
        if (results.multi_hand_landmarks[0].landmark[4].x < results.multi_hand_landmarks[0].landmark[
            3].x) and orientation == "right hand":
            fingers[0] = 1

        if (results.multi_hand_landmarks[0].landmark[4].x > results.multi_hand_landmarks[0].landmark[
            3].x) and orientation == "left hand":
            fingers[0] = 1
        if (results.multi_hand_landmarks[0].landmark[4].x < results.multi_hand_landmarks[0].landmark[
            3].x) and orientation == "left hand":
            fingers[0] = 0

        fingermarkList = [8, 12, 16, 20]
        i = 1
        for k in fingermarkList:
            if results.multi_hand_landmarks[0].landmark[k].y > results.multi_hand_landmarks[0].landmark[k - 2].y:
                fingers[i] = 0
            else:
                fingers[i] = 1

            i = i + 1

        return fingers

    def detection_loop(self):
        with self.mp_hands.Hands(model_complexity=0, min_detection_confidence=0.75,
                                 min_tracking_confidence=0.5) as hands:
            self.tello.streamoff()  # Ends the current stream, in case it's still opened
            self.tello.streamon()  # Starts a new stream
            while True:
                frame_read = self.tello.get_frame_read()  # Stores the current streamed frame
                image = frame_read.frame
                self.battery = self.tello.get_battery()
                if self.battery <= 50:

                    self.tricks = False

                else:
                    
                    self.tricks = True

                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = hands.process(image)

                # Draw the hand annotations on the image.
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)


                if results.multi_hand_landmarks:
                    action = " "
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS,
                            self.mp_drawing_styles.get_default_hand_landmarks_style(),
                            self.mp_drawing_styles.get_default_hand_connections_style())
                    
                    orientation = self.define_orientation(results)
                    fingers = self.fingers_position(results, orientation)
                    #print(fingers)
                    self.action_to_do(fingers, orientation, results)
                    
                for event in pg.event.get():
                    if event.type == pg.KEYDOWN:
                        if event.key == pg.K_l:
                            self.tello.land()
                        if event.key == pg.K_t:
                            self.tello.takeoff()
                        if event.key == pg.K_b:
                            print("A bateria esta em ", self.tello.get_battery(), "%")
                        if event.key == pg.K_m:
                            return 0

                cv2.imshow("image", image)
                if cv2.waitKey(5) & 0xFF == 27:
                    break
                
    def key_control(self):

        self.tello.streamoff()  # Ends the current stream, in case it's still opened
        self.tello.streamon()  # Starts a new stream
        while True:
            frame_read = self.tello.get_frame_read()  # Stores the current streamed frame
            image = frame_read.frame

            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Draw the hand annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            for event in pg.event.get():
                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_w:
                        self.tello.move_forward(20)
                    if event.key == pg.K_a:
                        self.tello.move_left(20)
                    if event.key == pg.K_s:
                        self.tello.move_back(20)
                    if event.key == pg.K_d:
                        self.tello.move_right(20)
                    if event.key == pg.K_q:
                        self.tello.rotate_counter_clockwise(20)
                    if event.key == pg.K_e:
                        self.tello.rotate_clockwise(20)
                    if event.key == pg.K_SPACE:
                        self.tello.move_up(20)
                    if event.key == pg.K_LCTRL:
                        self.tello.move_down(20)
                    if event.key == pg.K_b:
                        print("A bateria esta em ", self.tello.get_battery(), "%")
                    if event.key == pg.K_m:
                        return 0
    
            cv2.imshow("image", image)
            if cv2.waitKey(5) & 0xFF == 27:
                break
    
                                
        

    def main_interface(self):
        telloMode = -1
        self.tello_startup()
        pg.init()
        win = pg.display.set_mode((500,500))
        pg.display.set_caption("Test")
        #self.tello.takeoff()
        
        print("Para controlar pelo teclado, digite 1")
        print("Para controlar com a mao, digite 2")
        print("Para sair, digite 0")

        self.tello.streamoff()  # Ends the current stream, in case it's still opened
        self.tello.streamon()  # Starts a new stream
        while telloMode != 0:
            frame_read = self.tello.get_frame_read()  # Stores the current streamed frame
            image = frame_read.frame

            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Draw the hand annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  
            
            cv2.imshow("image", image)
            if cv2.waitKey(5) & 0xFF == 27:
                break
        
            if telloMode == 1:
                self.key_control()
                telloMode = -1
                print("Para controlar pelo teclado, digite 1")
                print("Para controlar com a mao, digite 2")
                print("Para sair, digite 0")
            elif telloMode == 2:
                self.detection_loop()
                telloMode = -1
                print("Para controlar pelo teclado, digite 1")
                print("Para controlar com a mao, digite 2")
                print("Para sair, digite 0")
            elif telloMode == 0:
                self.tello.land()
                telloMode = -1
                print("Obrigado por voar hoje")
            elif telloMode != -1 and telloMode != 1 and telloMode != 2:
                print("valor invalido!")
            for event in pg.event.get():
                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_1:
                        telloMode = 1
                    if event.key == pg.K_2:
                        telloMode = 2
                    if event.key == pg.K_0:
                        telloMode = 0
                    if event.key == pg.K_l:
                        self.tello.land()
                    if event.key == pg.K_t:
                        self.tello.takeoff()
                    if event.key == pg.K_b:
                        print("A bateria esta em ", self.tello.get_battery(), "%")
        

tello_control = hand_tello_control()
tello_control.main_interface()