#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from image_pros_2.msg import HoleCenterMsg2
import cv2
import numpy as np
import time
from std_msgs.msg import Float64MultiArray, MultiArrayDimension



class CircleDetectionNode(Node):
    def __init__(self):
        super().__init__('circle_2D_detection')
        #subscription to zivid camera data
        self.subscription = self.create_subscription(
            Image,
            '/zivid/color/image_color',
            self.image_callback,
            10)
        #publisher
        self.publisher_ = self.create_publisher(HoleCenterMsg2, 'hole_center_2D', 10)
        self.publisher_2_= self.create_publisher(Float64MultiArray, '/hole_color', 10)

        self.color = Float64MultiArray()
        self.color.layout.dim.append(MultiArrayDimension())
        self.color.layout.dim.append(MultiArrayDimension())
        self.color.layout.dim[0].label = "height"
        self.color.layout.dim[1].label = "width"
        self.color.layout.dim[0].size = 4
        self.color.layout.dim[1].size = 4
        self.color.layout.dim[0].stride = 4*4
        self.color.layout.dim[1].stride = 4


        self.subscription  # prevent unused variable warning
        self.cv_bridge = CvBridge()
        self._raw_image = None

    # from the raw image
    # create a mask to filter the desired color --> white, green, red
    # inside of the masked areas, find the circles using Hough Transform function
    # return: the pixel coordinate of each circles
    #        the radius of each circles
    #        the amount of each green, red, and white circles
    def detect_circle(self):
        # Define range of green color in HSV
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
    
        # Define range of red color in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # Define range of white color in HSV
        lower_white = np.array([110,50,50])
        upper_white = np.array([130,255,255])

        hsv = cv2.cvtColor(self._raw_image, cv2.COLOR_BGR2HSV)

        #create the mask for each color
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_red  = cv2.inRange(hsv, lower_red , upper_red )
        mask_white  = cv2.inRange(hsv, lower_white , upper_white )
        # cv2.imshow("mask_green", mask_green)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # cv2.imshow("mask_red", mask_red)

        #inside of the areas defined by the masks, using the HoughTransform find the circles --> markers on the box
        circles_green = cv2.HoughCircles(mask_green, cv2.HOUGH_GRADIENT, dp=1, minDist=70, param1=100, param2=10, minRadius=20, maxRadius=50)
        circles_red = cv2.HoughCircles(mask_red, cv2.HOUGH_GRADIENT, dp=1, minDist=70, param1=100, param2=10, minRadius=20, maxRadius=50)
        circles_white = cv2.HoughCircles(mask_white, cv2.HOUGH_GRADIENT, dp=1, minDist=70, param1=100, param2=10, minRadius=20, maxRadius=50)
        
        # print('circle_green',circles_green)
        
        #save how many circles for each color
        if circles_green is None:
            green_number = 0
        else:
            green_number = circles_green.shape[1]
        # print("------------------------green number",green_number)
        


        red_number = circles_red.shape[1]
        white_number = circles_white.shape[1]

        self.get_logger().info("Number of circles detected: %d" % (green_number+red_number+white_number))
        self.get_logger().info("Green circles detected: %d" % green_number)
        self.get_logger().info("Red circles detected [1]: %d" % red_number)
        self.get_logger().info("Blue circles detected [1]: %d" % white_number)



        if green_number == 0:
            circles = np.concatenate((circles_red, circles_white), axis=1)
        else:
            circles = np.concatenate((circles_green, circles_red, circles_white), axis=1)

        return circles, green_number, red_number, white_number

    # subscriber callback
    # 
    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # Save the image into a variable of cv2 (OpenCV)
        self._raw_image = cv_image
        self.get_logger().info("Image received.")
        cv2.imwrite('image.jpg', self._raw_image)

        #process the image --> circle extraction
        # self.get_logger().info("service called !")
        if self._raw_image is None:
            self.get_logger().warning("No raw image received yet.")
        else:
            #circle extraction process
            circles, green_number, red_number, white_number = self.detect_circle()
            #create the message and pubblish on the topic --> hole_center_2D
            if circles is not None:
                circles = np.uint16(np.around(circles))
                hole_centers = circles[0, :, :2].astype(np.int16)

                msg = HoleCenterMsg2()
                msg.hole_center_x = hole_centers[:, 0].tolist()
                msg.hole_center_y = hole_centers[:, 1].tolist()
                #compute the index of the circle
                circle_values = []  # Array to store computed values for each circle
                for i in circles[0, :]:
                    value = i[0] + (i[1] * 1920)
                    circle_values.append(value)
                tmp_index = np.array(circle_values, dtype=np.int32)
                msg.point_index = tmp_index.tolist()

                #circle color
                circle_color = np.column_stack((np.full((1, green_number), 0, dtype=np.int16), 
                                               np.full((1, red_number), 1, dtype=np.int16),
                                                np.full((1, white_number), 3, dtype=np.int16)))
                
                circle_color_float = np.column_stack((np.full((1, green_number), 0, dtype=np.float32), 
                                                     np.full((1, red_number), 1, dtype=np.float32), 
                                                     np.full((1, white_number), 3, dtype=np.float32)))
                
                # print('circle_color', circle_color)
                #print('circle_color', circle_color_float, type(circle_color_float))

                msg.circle_color = circle_color[0].tolist()
                self.color.data = circle_color_float[0].tolist()
                

                # self.get_logger().info("Detected hole centers: \n %s" % str(hole_centers))
                wait_time = 5
                self.get_logger().info("waiting %s second" % wait_time)
                time.sleep(wait_time)

                #publish the resust of the process
                self.publisher_.publish(msg)
                self.publisher_2_.publish(self.color)
                self.get_logger().info('Publishing HoleCenterMsg & hole_color')
            else:
                self.get_logger().warning("No circles detected in the image.")




def main(args=None):
    rclpy.init(args=args)
    node = CircleDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()