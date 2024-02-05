from multiprocessing import Process, Pipe
from time import sleep
import time
import cv2
from random import randint
import os

#seconds until picture is taken
PICTURE_TIME = 3

def simulate_move():
    sleep(randint(2,5))
    print("Tello is moving...")

    if randint(0, 9) == 0:
        raise ConnectionError("Command was not properly relayed to DJI Tello.")

def get_frame():
    return cv2.imread("/home/guilherme/Pictures/bruh.png")

def state_visualizer(tello_state, pipe_output):

    counter = 0

    cap = cv2.VideoCapture(0)
    sleep(1)

    while(tello_state != b'stop'):
        _, frame = cap.read()
        cv2.imshow("Game", frame)
        
        if tello_state == b'picture':
            print("Taking picture!")
            cv2.imwrite(f"{path}/{counter}.png", frame)
            counter += 1
            tello_state = b'idle'

        if pipe_output.poll(): 
            tello_state = pipe_output.recv()
        
def command_issuer(pipe_input):

    while(True):
        try:
            pipe_input.send(b'idle')

            simulate_move()

            pipe_input.send(b'picture')
            sleep(PICTURE_TIME)

        except:
            print("Tello error! Closing processes...")
            break
    
    pipe_input.send(b'stop')

if __name__ == '__main__':

    tello = b'placeholder'

    pipe_output, pipe_input = Pipe(False)

    path = f"{os.path.dirname}/images/"
    os.makedirs(path)

    input("Type anything to start!")

    visualizer_process = Process(target=state_visualizer, args=(tello, pipe_output, ) )
    visualizer_process.start()

    issuer_process = Process(target=command_issuer, args=(pipe_input, ))
    issuer_process.start()

    visualizer_process.join()
    issuer_process.join()