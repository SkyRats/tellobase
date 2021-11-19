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

    def action_to_do(self, fingers, results): #use the variable results for the hand tracking control

        if fingers == [0, 1, 0, 0, 0]:
            self.action = "One flip"
            #self.tello.flip_forward() #check if its working

        elif fingers == [0, 1, 1, 0, 0]: 
            self.action = "Two flips"
            #self.tello.flip_forward() #check if there is a time between two tello commands

            #self.tello.flip_forward()

        elif fingers == [0, 1, 1, 1, 0]:
            self.action = "Square"
    
        elif fingers == [0, 0, 1, 0, 0]:
            self.action = " :( "
            #self.tello.land()
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
                    print(fingers)
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
        self.tello.land()


tello_control = hand_tello_control()
tello_control.main_interface()
