#!/bin/bash
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ./cam_params.yml & \
ros2 run turtlebot_controller camera_node