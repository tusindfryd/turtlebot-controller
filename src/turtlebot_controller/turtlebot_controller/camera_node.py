#!/usr/bin/env python3

# This node uses in parts code shared by Evan Flynn and Lucas Walter
# with the following license.
#
# Copyright 2021 Evan Flynn, Lucas Walter
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Evan Flynn nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # ROS2 package to convert between ROS and OpenCV Images
import cv2  # Python OpenCV library
import numpy as np


class ExamineImage(Node):
    def __init__(self):
        super().__init__("examine_image")
        self.window_name = "camera"
        self.subscription = self.create_subscription(
            Image, "image_raw", self.image_callback, 100
        )
        self.subscription
        self.point = None
        self.mat = None

    def image_callback(self, image_data):
        image_size = (image_data.height, image_data.width)

        if image_data.encoding == "rgb8":
            dirty = (
                self.mat is None
                or image_data.width != self.mat.shape[1]
                or image_data.height != self.mat.shape[0]
                or len(self.mat.shape) < 2
                or self.mat.shape[2] != 3
            )

            if dirty:
                self.mat = np.zeros(
                    [image_data.height, image_data.width, 3], dtype=np.uint8
                )

            self.mat[:, :, 2] = np.array(image_data.data[0::3]).reshape(image_size)
            self.mat[:, :, 1] = np.array(image_data.data[1::3]).reshape(image_size)
            self.mat[:, :, 0] = np.array(image_data.data[2::3]).reshape(image_size)

        elif image_data.encoding == "mono8":
            self.mat = np.array(image_data.data).reshape(image_size)

        if self.point is not None:
            cv2.rectangle(
                self.mat,
                (self.point[0] - 100, self.point[1] - 100),
                (self.point[0] + 100, self.point[1] + 100),
                (0, 255, 0),
                3,
            )

        cv2.setMouseCallback(self.window_name, self.draw_rectangle)

        if self.mat is not None:
            cv2.imshow(self.window_name, self.mat)
            cv2.waitKey(5)

    def draw_rectangle(self, event, x, y, flags, param):
        self.point = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            print(x, y)  # check if mouse event is click
        


def main(args=None):
    rclpy.init(args=args)
    examine_image = ExamineImage()
    rclpy.spin(examine_image)
    examine_image.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
