#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from collections import deque
import cv2
import numpy as np


CAMERA_TOPIC = "/camera"

class MyNode(Node):
    def __init__(self):
        node_name="player"
        super().__init__(node_name)
        self.cv_br = CvBridge()
        self.img_sub = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_handler,
            qos_profile=qos_profile_sensor_data,
        )

        self.queue = deque(maxlen=2)
        self.get_logger().info("Hello ROS2")

    def calc_orb(self, frame1, frame2):
        orb = cv2.ORB_create()
        kp1, des1 = orb.detectAndCompute(frame1, None)
        kp2, des2 = orb.detectAndCompute(frame2, None)
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)
        # Extract the matched keypoints' coordinates
        points1 = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        points2 = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # Estimate the optical flow using matched points
        for i, (pt1, pt2) in enumerate(zip(points1, points2)):
            x1, y1 = pt1.ravel()
            x2, y2 = pt2.ravel()
            print(f"Motion vector {i}: ({x1}, {y1}) -> ({x2}, {y2})")
        img3 = cv2.drawMatches(frame1, kp1, frame2, kp2, matches[:50], None, flags=2)
        cv2.imshow("ORB", img3)
        cv2.waitKey(10)

    def image_handler(self, msg: Image):
        """_summary_
        https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html

        """
        frame = self.cv_br.imgmsg_to_cv2(msg)
        
        self.queue.append(frame)
        if len(self.queue) == 2:
            self.calc_orb(self.queue[0], self.queue[1])
        # cv2.imshow("debug", frame)
        

        

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()