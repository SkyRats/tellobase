from multiprocessing import Process, Pipe
from time import sleep
import time
import cv2
from random import randint
import os

#seconds until picture is taken
PICTURE_TIME = 3
FLASH_TIME = 0.8

#simulates a Tello movement action
def simulate_move():
    sleep(randint(2,5))
    print("Tello is moving...")

    if randint(0, 9) == 0:
        raise ConnectionError("Command was not properly relayed to DJI Tello.")

def state_visualizer(tello_state, pipe_output):
    cap = cv2.VideoCapture(0)
    counter = 0

    while(tello_state != b'stop'):
        _, frame = cap.read()
        cv2.imshow("Game", frame)
        cv2.waitKey(1)
        
        if tello_state == b'picture':
            print("Taking picture!")
            start_time = time.time()
            elapsed = 0
            while(elapsed < FLASH_TIME):
                _, frame = cap.read()
                elapsed = time.time() - start_time
                
                flash_percentage = (elapsed/FLASH_TIME)
                frame = cv2.multiply(frame, (flash_percentage, flash_percentage, flash_percentage, 1))
                cv2.imshow("Game", frame)
                cv2.waitKey(1)
            
            cv2.imwrite(f"{path}/{counter}.png", frame)
            counter += 1
            tello_state = b'idle'

        if pipe_output.poll(): 
            tello_state = pipe_output.recv()
        
def command_issuer(pipe_input):
    while(True):
        try:
            simulate_move()

            sleep(PICTURE_TIME)
            pipe_input.send(b'picture')

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