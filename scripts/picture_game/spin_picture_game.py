from time import sleep
import time
import cv2
import random
import os
from djitellopy import Tello

PICTURES_UNTILL_CHANGE = 3
DOWN_LIMIT = 80
UP_LIMIT = 230
SLEEP_FOR = 2

path = os.getcwd()
dir_name = "spingame_1"
dir_num = 1

for dir in os.listdir(path):
    try:
        print(dir)
        n = dir.split("_")[1]
        if int(n) >= dir_num:
            dir_num = int(n) + 1
    except:
        pass

dir_name = f"spingame_{dir_num}"
os.mkdir(dir_name)

tello = Tello()
tello.connect()
tello.streamon()
cap = tello.get_frame_read()

sleep(0.2)

tello.takeoff()
tello.move_up(100)

index = 0
pictures_til_change = 0

dist = tello.get_distance_tof()
print(os.getcwd())

while(True):
    try:
        if(abs(tello.get_distance_tof() - dist) > 60):
            tello.emergency()

        if(pictures_til_change == 3):
            pictures_til_change = 0
            tello.move_down(random.randint(20, 40))
            dist = tello.get_distance_tof()

        else:

            if(random.randint(0, 1) == 0):
                tello.rotate_clockwise(random.randint(40, 360))
            else:
                tello.rotate_counter_clockwise(random.randint(40, 360))
            
            start_time = time.time()

            sleep(1)
            
            frame = cap.frame
            cv2.imwrite(f"{path}/{dir_name}/picture_{index}.png", frame)

            index += 1
            pictures_til_change += 1

    except KeyboardInterrupt:
        tello.streamoff()
        break

tello.streamoff()
