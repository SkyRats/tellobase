import time
from time import sleep
import cv2
import random
import os
from djitellopy import Tello

# -v --> Verbose

verbose = False
if '-v' or '--v' or '--verbose' in sys.argv:
    print("Verbose mode enabled")

PICTURES_UNTILL_CHANGE = 3
DOWN_LIMIT = 80
UP_LIMIT = 230
SLEEP_FOR = 2

def get_parent(n): # n-grand parent of directory
    result = os.path.abspath(__file__)
    for _ in range(n):
        result = os.path.dirname(result)
    return result

media_dir = os.path.join(get_parent(3), "media")

# Check if the media directory exists
if os.path.exists(media_dir):
    print("Media will be stored at:", media_dir)
else:
    print("Creating 'media' folder in tellobase!")
    os.mkdir(media_dir)

timestamp = time.strftime("%Y%m%d%H%M%S")
dir_name = f"spingame_{timestamp}"

os.mkdir(os.path.join(media_dir, dir_name))

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
            cv2.imwrite(f"{media_dir}/{dir_name}/picture_{index}.png", frame)

            index += 1
            pictures_til_change += 1

    except KeyboardInterrupt:
        tello.streamoff()
        exit
        break

tello.streamoff()
