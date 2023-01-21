### turtlebot-controller
#### semester project for tasfrs labs (student id: 144661)

**description**: the robot moves backwards or forwards depending on a) whether an aruco marker is detected in the top part or the bottom part of the frame OR whether a pointer is clicked in the top part or the bottom part of the camera frame window.
the aruco marker detection is by default set to the 4x4 class. 

**launching**: `./run_gazebo.sh` and then `./start.sh`

![demo: whether marker is detected in the top part or the bottom part of the frame determines if the robot moves backwards or forwards](demo.gif)

#### acknowledgements:
these repositories were a point of reference in the creation of this project:
- [usb_cam package examples and demos by flynneva and lucasw](https://github.com/ros-drivers/usb_cam)
- [aruco detection example by niconielsen32](https://github.com/niconielsen32/ComputerVision/blob/master/ArUco/arucoDetection.py)
