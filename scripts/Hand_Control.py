import cv2, time
import mediapipe as mp
from djitellopy import Tello


class hand_tello_control:
    def __init__(self):
        self.action = " "
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        self.action_done = False
        self.tricks = [1,1,1,1,1,1,1,1] #forward flip, back flip, right flip, left flip, square, :(, battery, land 

    def tello_startup(self):
        # For Tello input:
        self.tello = Tello()  # Starts the tello object
        self.tello.connect()  # Connects to the drone


    def define_orientation(self, results):

        if results.multi_hand_landmarks[0].landmark[4].x < results.multi_hand_landmarks[0].landmark[17].x:
            orientation = "left hand"
        else:
            orientation = "right hand"
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
        
        
    def commandReset(self, trick): #Resets command counters in action_to_do function, excluding the current count
        if trick == -1:
            self.tricks = [1,1,1,1,1,1,1,1]
            return
        for i in range(0,7):
            if i != trick:
                self.tricks[i] = 1
        return

    def action_to_do(self, fingers, orientation, results): #use the variable results for the hand tracking control
        
        if self.action_done == True: #Resets counters when a trick was performed
            self.commandReset(-1)
            self.action_done = False

        #Left hand controls tricks, right hand controls movement
        if orientation == "left hand":   #Thumb on the left = left hand!
            if fingers == [0, 1, 0, 0, 0] and self.battery ==  True:
                if self.tricks[0] >= 15:
                    self.action = "flip forward"
                    self.tello.flip_forward() 
                    self.action_done = True
                self.tricks[0] = self.tricks[0] + 1
                self.commandReset(0)
 
            elif fingers == [0, 1, 1, 0, 0] and self.battery ==  True:
                if self.tricks[1] >= 15:
                    self.action = "flip back"
                    self.tello.flip_back()
                    self.action_done = True
                self.tricks[1] = self.tricks[1] + 1
                self.commandReset(1)
            elif fingers == [0, 0, 0, 0, 1] and self.battery ==  True:
                if self.tricks[2] >= 15:
                    self.action = "flip right"
                    self.tello.flip_right()
                    self.action_done = True
                self.tricks[2] = self.tricks[2] + 1
                self.commandReset(2)
            elif fingers == [1, 0, 0, 0, 0] and self.battery ==  True:
                if self.tricks[3] >= 15:
                    self.action = "flip left"
                    self.tello.flip_left()
                    self.action_done = True
                self.tricks[3] = self.tricks[3] + 1
                self.commandReset(3)
            elif fingers == [0, 1, 1, 1, 0]:
                if self.tricks[4] >= 15:
                    self.action = "Square"
                    self.tello.move_left(20)
                    self.tello.move_up(40)
                    self.tello.move_right(40)
                    self.tello.move_down(40)
                    self.tello.move_left(20)
                    self.action_done = True
                self.tricks[4] = self.tricks[4] + 1
                self.commandReset(4)
            elif fingers == [0, 0, 1, 0, 0]:
                if self.tricks[5] >= 30:
                    self.action = " :( "
                    self.tello.land()
                    self.action_done = True
                self.tricks[5] = self.tricks[5] + 1 
                self.commandReset(5) 
            elif ((self.battery == False) and (fingers == [1, 0, 0, 0, 0] or fingers == [0, 1, 0, 0, 0] or fingers == [0, 0, 0, 0, 1])): #not avaiable to do tricks
                if self.tricks[6] >= 15:
                    self.tello.rotate_clockwise(45)
                    self.tello.rotate_counter_clockwise(90)
                    self.tello.rotate_clockwise(45)
                    self.action_done = True
                self.tricks[6] = self.tricks[6] + 1
                self.commandReset(6)      
            else:
                self.action = " "
                self.commandReset(-1)

        elif orientation == "right hand":  #Thumb on the right = right hand!
            if fingers == [1, 1, 1, 1, 1]:
                self.action = "Follow"
                self.follow_hand(results)
                self.commandReset(-1)
            elif fingers == [1, 0, 0, 0, 0]:
                if self.tricks[7] >= 30:
                    self.action = "Land"
                    self.tello.land()
                    self.action_done = True
                self.tricks[7] = self.tricks[7] + 1 
                self.commandReset(7)          
            else:
                self.action = " "
                self.commandReset(-1)
                
    def batteryCheck(self):#Checks battery levels, returns bool based on trick ability
        btLevel = self.tello.get_battery()
        if btLevel <= 50:
            return False
        else:
            return True


    def fingers_position(self, results, orientation):

        # [thumb, index, middle finger, ring finger, pinky]
        # 0 for closed, 1 for open
        fingers = [0, 0, 0, 0, 0]

        if (results.multi_hand_landmarks[0].landmark[4].x > results.multi_hand_landmarks[0].landmark[
            3].x) and orientation == "left hand":
            fingers[0] = 0
        if (results.multi_hand_landmarks[0].landmark[4].x < results.multi_hand_landmarks[0].landmark[
            3].x) and orientation == "left hand":
            fingers[0] = 1

        if (results.multi_hand_landmarks[0].landmark[4].x > results.multi_hand_landmarks[0].landmark[
            3].x) and orientation == "right hand":
            fingers[0] = 1
        if (results.multi_hand_landmarks[0].landmark[4].x < results.multi_hand_landmarks[0].landmark[
            3].x) and orientation == "right hand":
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
                self.battery = self.batteryCheck()

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

                cv2.putText(image, f'Action: {str(self.action)}', (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (100, 100, 255),
                            3, )


                cv2.imshow("image", image)
                if cv2.waitKey(5) & 0xFF == 27:
                    break

    def main_interface(self):
        self.tello_startup()
        self.tello.takeoff()
        self.detection_loop()
        


tello_control = hand_tello_control()
tello_control.main_interface()