# tellobase
## Before using

### 1. Set up your Linux environment

Set up a Ubuntu 18.04 box or VM. This should include ffmpeg 3.4.4.
~~~
ffmpeg -version
~~~

### 2. Set up your Python environment

Use your favorite Python package manager to set up Python 2.7 and the following packages:

* opencv-python 
* mediapipe as mp
* from datetime: datetime, date, timedelta
* pygame
* time
* random
* os
* beepy

### 3. Install DJITellopy

#### Install using pip

```pip install djitellopy```

For Linux distributions with both python2 and python3 (e.g. Debian, Ubuntu, ...) you need to run

```pip3 install djitellopy```

#### Install in developer mode
Using the commands below you can install the repository in an editable way. This allows you to modify the library and use the modified version as if you had installed it regularly.

```bash
git clone https://github.com/damiafuentes/DJITelloPy.git
cd DJITelloPy
pip install -e .
```

## Using ```tello_integration.py```

1. Automatic takeoff in both modes (1 and 2). Press 1 to "hand and key control" and 2 to play the camera game.

2. Control the drone with the keyboard:

- w : move forward 20 cm
- s : move back 20 cm
- a : move left 20 cm
- d :> move right 20 cm
- SPACE : move up 20 cm
- SHIFT : move down 20 cm
- t : takeoff
- l : land
- b : prints battery
- i : prints tello current state (info)
- LEFT (<-) : flip left
- RIGHT (->) : flip right
- UP : flip forward
- DOWN : flip back
- e : sends the drone to have speed 0
- BACKSPACE : stop all motors (emergency)
- k : checks tello connection
- ESCAPE : leaves the loop

3. In hand and key control, the mediapipe accepted commands are:
- High Five -> follow the hand
- Thumb -> side flips
- Indicator -> front flips
- Hang-loose, V and rock -> take a picture
- L -> land
- 3 -> rotate
- 4 -> square

4. In camera game, follow the instructions printed in the terminal.

## Using ``` ttt.py```

1.
