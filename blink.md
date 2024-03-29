## Using ```blink.py```
This is a code developed to connect with the DJITello drone as part of the 2023 version of the "Tello Lúdico" project. In this game, you can control the drone through blinks!   
* Make sure you have installed the following libraries:
  > OpenCV (python) -  https://docs.opencv.org/4.x/d2/de6/tutorial_py_setup_in_ubuntu.html
  ```bash
  $ python -m pip install mediapipe
  ```

  > Mediapipe - https://developers.google.com/mediapipe/solutions/setup_python
  ```bash
  $ sudo apt-get install python3-opencv
  ```

  > DJITelloPy - https://djitellopy.readthedocs.io/en/latest/ 

  ```bash
  pip3 install djitellopy
  ```
1. Make sure you are in a well lit environment, and connect your computer to the Tello through WiFi;
   
2. After connecting to the Tello, run the program 'blink.py' and wait for the AI inicialization. When everything is ready two tabs must be opened, one with the image coming directly from the drone (im1) and the other being the AI processed image (im2) - This tab will only work if there is a face being detected;
   
3. Now to control the drone through your eyes get close to it (1 m) and try the following commands:
* Back flip: blink both eyes
* Flip to your left: blink your left eye
* Flip to your right: blink your right eye
* Land: open your eyes (STRONGLY)
  
  >If you're having trouble with the blink detection try blinking slower than usual, make sure you're close enough, check if the environment brightness is good enough for the camera, and also make sure the battery is >= 50% (the drone will not flip if that is not the case) 

4. Another fun feature to try is the facetracking algorithm:
* Try crouching and getting up while looking at the drone, it should go down and up as well;
* Try moving slowly arrond the drone while looking at it, the drone should follow you;

   
