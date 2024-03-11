import time
from time import sleep
import sys
import cv2
import random
import os
from djitellopy import Tello

# -r --> Number of rounds

DEFAULT_ROUNDS_NUMBER = 3
PICTURES_UNTILL_CHANGE = 3

DOWN_LIMIT = 80
UP_LIMIT = 230
SLEEP_FOR = 2

def get_rounds_arg():
    index = -1

    if '-r' in sys.argv:
        index = sys.argv.index("-r")

        if (index + 1 >= len(sys.argv)):
            print("No number of rounds provided after '-r', default used")
            return DEFAULT_ROUNDS_NUMBER
        else:
            try:
                result = int(sys.argv[index + 1])
                print("Number of rounds (-r):", result)
                return result
            except ValueError:
                print("Couldn't undestand rounds number, default used")
            return DEFAULT_ROUNDS_NUMBER
    else:
        print("Number of rounds (default):", DEFAULT_ROUNDS_NUMBER)
        return DEFAULT_ROUNDS_NUMBER
    
def get_parent(n): # n-grand parent of directory
    result = os.path.abspath(__file__)
    for _ in range(n):
        result = os.path.dirname(result)
    return result

rounds = get_rounds_arg()
media_dir = os.path.join(get_parent(3), "media")

# Check if the media directory exists
if os.path.exists(media_dir):
    print("Media will be stored at:", media_dir)
else:
    print("Creating 'media' folder in tellobase!")
    os.mkdir(media_dir)

timestamp = time.strftime("%Y%m%d%H%M%S")
session_dir = os.path.join(media_dir, f"spingame_{timestamp}")

os.mkdir(session_dir) # TODO: Move somewhere else so it doesn't create without photos

tello = Tello()
tello.connect()
tello.streamon()

cap = tello.get_frame_read()

sleep(0.2)

tello.takeoff()
tello.move_up(100)

index = 0
round_now = 1
pictures_til_change = 0

dist = tello.get_distance_tof()

print(f"Round {round_now}/{rounds}")

while(True):
    try:
        if(abs(tello.get_distance_tof() - dist) > 60):
            tello.emergency()

        if(pictures_til_change == PICTURES_UNTILL_CHANGE):
            if (round_now <= rounds): 
                round_now += 1
                print(f"Round {round_now}/{rounds}")

                pictures_til_change = 0

                tello.move_down(random.randint(20, 40))
                dist = tello.get_distance_tof()
            else:
                break
            
        else:

            if(random.randint(0, 1) == 0):
                tello.rotate_clockwise(random.randint(40, 360))
            else:
                tello.rotate_counter_clockwise(random.randint(40, 360))
            
            start_time = time.time()

            sleep(1)
            
            # frame = cv2.cvtColor(cap.frame, cv2.COLOR_RGB2BGR)
            frame = cap.frame
            cv2.imwrite(f"{media_dir}/{dir_name}/picture_{index}.png", frame)

            index += 1
            pictures_til_change += 1

    except KeyboardInterrupt:
        tello.streamoff()
        break

tello.streamoff()
