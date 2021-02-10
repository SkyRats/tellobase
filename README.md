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
* tellopy 0.5.0

### 3. Install ROS

[Install ROS Melodic](http://wiki.ros.org/Installation/Ubuntu) with the `ros-melodic-desktop-full` option.
This will install Gazebo 9 and OpenCV 3.2, among other things.

### 4. Install tello_driver

* `$ cd <CATKIN_WS/SRC>`
* `$ git clone https://github.com/anqixu/TelloPy.git`
* `$ cd TelloPy`
* `$ sudo -H pip2 install -e .`
* `$ cd ..`
* `$ git clone https://github.com/anqixu/h264_image_transport.git`
* `$ git clone https://github.com/anqixu/tello_driver.git`
* `$ cd ..`
* `$ rosdep install h264_image_transport`
* skip this step: `$ # rosdep install tello_driver # not working currently`
* `$ catkin build tello_driver`


#### Running the driver

* turn on drone and wait for its front lights to blink amber
* connect WiFi to drone's access point (e.g. `TELLO_######`)
* `$ roslaunch tello_driver launch/tello_node.launch`