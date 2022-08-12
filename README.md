# tellobase
## Before using

### 1. Set up your Linux environment

Set up a Ubuntu 18.04 box or VM. This should include ffmpeg 3.4.4.
~~~
ffmpeg -version
~~~

### 2. Set up your Python environment

Use your favorite Python package manager to set up Python 2.7 and the following packages:

* numpy 1.15.2
* av 0.5.2
* opencv-python 3.4.3.18
* opencv-contrib-python 3.4.3.18
* [codec-image-transport](https://github.com/yoshito-n-students/codec_image_transport.git)


### 3. Install ROS

[Install ROS Melodic](http://wiki.ros.org/Installation/Ubuntu) with the `ros-melodic-desktop-full` option.
This will install Gazebo 9 and OpenCV 3.2, among other things.

### 4. Install tello_driver from source

* ```$ cd <CATKIN_WS/SRC>```
* ```$ git clone --recursive https://github.com/appie-17/tello_driver.git```
* ```$ cd ..```  
With python-catkin-tools:
* ```$ catkin build tello_driver```
* ```$ source devel/setup.bash```



#### Running the driver

* turn on drone and wait for its front lights to blink amber
* connect WiFi to drone's access point (e.g. `TELLO_######`)
* `$ roslaunch tello_driver launch/tello_node.launch`


## Using ```HandAndKey_control.py```

1. Takeoff
- Press ```t``` for the drone to takeoff.
2. Control the drone with the keyboard
- Press ```1``` to start the control with the keyboard
- Use ```w``` to go forward, ```s``` to go backwards, ```d``` to go right and ```a``` to go left
- Use ```q``` to rotate anticlockwise and ```e``` to rotate clockwise
3. Control the drone using your *left* hand
- Press ```m``` to go back to the initial menu
- Press ```2``` to start the hand control
- The drone will follow your left hand

