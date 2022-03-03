# Tellobase 
## Introduction
Tellobase is a project that aims to control a Tello drone using visual commands, such as hand movements. This specific branch runs using only python itself, as well as necessary packages. 

## Before using

### 1. Set up your Linux environment

This project was created using Ubuntu 20.04, however, there's little to no errors that should appear as a result of utilizing a different OS, as all of the tools we used are available in other comparable OS. That being said, testing on other sistems has been minimal, and as such **Ubuntu 20.04 is strongly recommended**.  

### 2. Set up your Python environment

Use your favorite Python package manager to set up Python 3 and the following packages:

* opencv-python 4.5.5.62
* mediapipe
* djitellopy 2.4.0


## Running tellobase
Utilizing this repository should be as easy as running any python script. Just make sure to connect to your Tello drone over Wi-Fi first.

Currently, there's two scripts available: **Hand_Control.py** and **Hand_Control(No_Follow).py**. The main difference between the two is that **Hand_Control(No_Follow).py** has no hand following function. Choose one and run it as any other python 3 script.

# User guide
## For context
Both scripts contained in this branch use mediapipe in order to recognize the user's hands utilizing the drone's camera. As such, all commands given to the drone are dependant on what hand is being used, as well as what fingers are up or down. Currently, the way the code distinguishes the left and right hand is by looking at the direction of the thumb. **Therefore, for the time being, you can just change to showing the drone the back of your hand in order to change what commands are being given.**

Another thing to keep in mind is that **some tricks are only available with good battery levels**. When such a thing happens, the drone should "shake it's head" at the user (by rotating 45 degrees clockwise, then 90 counter clockwise, and finaly 45 clockwise, back to the starting position ) 

## Right hand commands
The right hand is mainly used in order to execute the "follow hand" command. **Simply extend all five fingers and move it in front of the drone's camera to have it follow**. 

It does this by trying to keep your hand at a certain range in the center of it's camera, and moving if it's outside of this range. If the hand goes outside of the camera range entirely, the drone will stop.

The only other command is the **"land"** function. Simply **lift your index finger alone** to make the drone land.

## Left Hand commands
The left hand is responsible for **tricks**, here's a list of the ones currently available:

* Flip Forward -- Lift only your index finger.
* Flip Backwards -- Lift both your index and middle fingers.
* Flip Right -- Lift only your thumb.
* Flip  Left -- Lift only your pinky.
* Make a square -- Lift your index, middle and ring fingers.

## Before using
Tellobase is still a **work-in-progress**, and as such, some features aren't completely done just yet, and some issues are very much present. 

For the best experience, it's recommended only one person be in the drone's field of vision at a time, and that you make sure only one hand is showing at a time. Issuing the commands somewhat slowly also helps.

As development continues, many of this issues will certainly go away, and this guide will change accordingly.