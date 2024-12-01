#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
import cv2
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import os

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__("img_capture")
        self.log = self.get_logger()
        self.log.info("Image Capture Node Online")

        # Global Variables 
        self.path = "/home/jhsrobo/corews/src/img_capture/img"
        self.count = 1
        self.cropped_count = 1
        self.bridge = CvBridge()
        self.cropping = False

        # Cropping Parameter
        self.create_timer(0.1, self.update_parameters)
        self.declare_parameter("cropping", self.cropping)
        
        self.camera_feed_subscriber = self.create_subscription(Image, "camera_feed", self.img_callback, 10)

    def update_parameters(self):
        change = (self.cropping != self.get_parameter("cropping").value)
        self.cropping = self.get_parameter("cropping").value

        if change:
            self.log.info(f"Cropping: {self.cropping}")

    def img_callback(self, screenshot):
        img = self.bridge.imgmsg_to_cv2(screenshot, desired_encoding="passthrough")
        if self.cropping:
            img = img[42:720, 0:1150]
            cv2.imwrite(f"{self.path}/cropped/{self.cropped_count}.png", img)
            self.cropped_count += 1 
        else:
            cv2.imwrite(f"{self.path}/full/{self.count}.png", img)
            self.count += 1 

def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureNode()

    # Delete prior images
    os.system("rm -rf /home/jhsrobo/corews/src/img_capture/cropped/*.png")
    os.system("rm -rf /home/jhsrobo/corews/src/img_capture/full/*.png")

    rclpy.spin(node)

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
