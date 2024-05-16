#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from image_pros_2.srv import HoleCenter
import cv2
import numpy as np


class CircleDetectionNode(Node):
    def __init__(self):
        super().__init__('circle_2D_detection')
        self.subscription = self.create_subscription(
            Image,
            '/zivid/color/image_color',  # Replace 'image_topic' with your actual topic name
            self.image_callback,
            10)
         # Create a service server
        self.srv = self.create_service(HoleCenter, 'hole_center', self.center_holes_callback)



        self.subscription  # prevent unused variable warning
        self.cv_bridge = CvBridge()
        self._raw_image = None


    def perform_circle_detection(self):
        if self._raw_image is None:
            self.get_logger().warning("No raw image received yet.")
            return

        gray = cv2.cvtColor(self._raw_image, cv2.COLOR_BGR2GRAY)
        img = cv2.medianBlur(gray, 5)
        cimg = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 120, param1=100, param2=30, minRadius=20, maxRadius=30)

        self.get_logger().info("Number of detected circles: %d" %len(circles[0]))

        if circles is not None:
            
            
            circles = np.uint16(np.around(circles))
            circle_values = []  # Array to store computed values for each circle

            for i in circles[0, :]:
                # outer circle
                cv2.circle(self._raw_image, (i[0], i[1]), i[2], (255, 0, 0), 2)
                # center of the circle
                cv2.circle(self._raw_image, (i[0], i[1]), 2, (0, 255, 0), 3)

                # Compute value and append to array
                value = i[0] + (i[1] * 1920)  # Assuming image width is 1920
                circle_values.append(value)

                # Print radius, center position, and computed value
                self.get_logger().info("Radius of circle: %d" % i[2])
                self.get_logger().info("Center position: (%d, %d)" % (i[0], i[1]))

            #self.get_logger().info("Computed value: %s" % circle_values)
        else:
            self.get_logger().warning("No circles detected in the image.")


    def center_holes_callback(self, request, response):
        self.get_logger().info("service called !")
        if self._raw_image is None:
            self.get_logger().warning("No raw image received yet.")
            return response

        gray = cv2.cvtColor(self._raw_image, cv2.COLOR_BGR2GRAY)
        img = cv2.medianBlur(gray, 5)
        cimg = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 120, param1=100, param2=30, minRadius=20, maxRadius=30)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            hole_centers = circles[0, :, :2].astype(np.int16)  # Extract centers and cast to int16
            response.hole_centers_x = hole_centers[:, 0].tolist()  # Convert to list of ints
            response.hole_centers_y = hole_centers[:, 1].tolist()  # Convert to list of ints
            self.get_logger().info("Detected hole centers: \n %s" % str(hole_centers))
        else:
            self.get_logger().warning("No circles detected in the image.")

        return response



    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # Save the image into a variable of cv2 (OpenCV)
        self._raw_image = cv_image
        self.get_logger().info("Image receved.")




def main(args=None):
    rclpy.init(args=args)
    node = CircleDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
