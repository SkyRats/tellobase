import cv2
import mediapipe as mp
from djitellopy import Tello


class hand_tello_control:
    def __init__(self):
        self.action = " "
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands

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
        centerRange = [10,10] #Range for detecting commands in the x and y axis.
        centerPoint = [128,128] #Theoretical center of the image
        if pixelCoordinatesLandmark == None: # If hand goes out of frame, stop following
            return

        
        elif pixelCoordinatesLandmark[0] > centerPoint[0] + centerRange[0] and pixelCoordinatesLandmark[1] > centerPoint[1] + centerRange[1]:  # Drone vai para baixo + esquerda (x e y positivo)
            xCorrection = (pixelCoordinatesLandmark[0] - centerPoint[0])
            yCorrection = (pixelCoordinatesLandmark[1] - centerPoint[1])
            ratioX = xCorrection//yCorrection
            ratioy = yCorrection//xCorrection
            print(ratioX,ratioy)
            self.tello.go_xyz_speed(ratioX,ratioy,0,10)

        elif pixelCoordinatesLandmark[0] < centerPoint[0] - centerRange[0] and pixelCoordinatesLandmark[1] < centerPoint[1] - centerRange[1]: # Drone vai para cima + direita (x e y negativo)
            xCorrection = (pixelCoordinatesLandmark[0] - centerPoint[0])
            yCorrection = (pixelCoordinatesLandmark[1] - centerPoint[1])
            ratioX = xCorrection//yCorrection
            ratioy = yCorrection//xCorrection
            print(ratioX,ratioy)
            self.tello.go_xyz_speed(ratioX,ratioy,0,10)
            
        elif pixelCoordinatesLandmark[0] > centerPoint[0] + centerRange[0] and pixelCoordinatesLandmark[1] < centerPoint[1] - centerRange[1]: #Drone vai para baixo + direita (x positivo, y negativo)
            xCorrection = (pixelCoordinatesLandmark[0] - centerPoint[0])
            yCorrection = (pixelCoordinatesLandmark[1] - centerPoint[1])
            ratioX = xCorrection//yCorrection
            ratioy = yCorrection//xCorrection
            print(ratioX,ratioy)
            self.tello.go_xyz_speed(ratioX,ratioy,0,10)

        elif pixelCoordinatesLandmark[0] < centerPoint[0] - centerRange[0] and pixelCoordinatesLandmark[1] > centerPoint[1] + centerRange[1]: #Drone vai para cima e esquerda (x negativo, y positivo)
            xCorrection = (pixelCoordinatesLandmark[0] - centerPoint[0])
            yCorrection = (pixelCoordinatesLandmark[1] - centerPoint[1])
            ratioX = xCorrection//yCorrection
            ratioy = yCorrection//xCorrection
            print(ratioX,ratioy)
            self.tello.go_xyz_speed(ratioX,ratioy,0,10)
        
        
    
    def action_to_do(self, fingers, results): #use the variable results for the hand tracking control

        if fingers == [0, 1, 0, 0, 0]:
            self.action = "One flip"
            #self.tello.flip_forward() #Flips once - Working!

        elif fingers == [0, 1, 1, 0, 0]: 
            self.action = "Two flips"
            #self.tello.flip_forward() #check if there is a time between two tello commands
            #It does, but it also queues up more flips while you wait.(Also, freezes the camera ;-;)
            #self.tello.flip_back()

        elif fingers == [0, 1, 1, 1, 0]:
            self.action = "Square"
    
        elif fingers == [1, 1, 1, 1, 1]:
            self.action = "Follow"
            self.follow_hand(results)
        elif fingers == [0, 0, 1, 0, 0]:
            self.action = " :( "
            self.tello.land()
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
                    self.action_to_do(fingers, results)

                cv2.putText(image, f'Action: {str(self.action)}', (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (100, 100, 255),
                            3, )
                cv2.imshow("image", image)
                if cv2.waitKey(5) & 0xFF == 27:
                    break

    def main_interface(self):
        self.tello_startup()
        self.tello.takeoff()
        self.detection_loop()
        #self.tello.land() 


tello_control = hand_tello_control()
tello_control.main_interface()
