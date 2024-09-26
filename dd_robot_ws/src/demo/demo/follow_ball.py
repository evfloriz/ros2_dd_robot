import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage

import cv2
from cv_bridge import CvBridge
import numpy as np


class MovementHandler(Node):

    def __init__(self):
        super().__init__('movement_handler')
        self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

        self.subsciber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.cv_pub = self.create_publisher(Image, '/cv_compressed', 10)
        self.bridge = CvBridge()

        # target in terms of horizontal pixel location
        self.target = -1
        
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        twist = Twist()
        
        if self.target == -1:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.5

            # angle is between -1 and 1
            angle = (320 - self.target) / 320
            twist.angular.z = angle
        
        self.publisher_.publish(twist)
        self.get_logger().info(f'Publishing: Linear {twist.linear.x} Angular {twist.angular.z}')

    def image_callback(self, image):
        self.get_logger().info(f'Received: Image')

        cv_image = self.bridge.imgmsg_to_cv2(image)

        # convert for gz camera sensor image
        # TODO: add env variable maybe?
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        
        # Color tracking taken from cmput312 example
        # ------------------------------------------

        blurred = cv2.medianBlur(cv_image,11)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Red Tracking
        redLowMask = (0,5,50)
        redHighMask = (10, 255, 255)
        mask = cv2.inRange(hsv, redLowMask, redHighMask)
        
        # Perform erosion and dilation in the image (in 11x11 pixels squares) in order to reduce the "blips" on the mask
        mask = cv2.erode(mask, np.ones((11, 11),np.uint8), iterations=2)
        mask = cv2.dilate(mask, np.ones((11, 11),np.uint8), iterations=5)
        # Mask the blurred image so that we only consider the areas with the desired colour
        masked_blurred = cv2.bitwise_and(blurred,blurred, mask= mask)
        # masked_blurred = cv2.bitwise_and(frame,frame, mask= mask)
        # Convert the masked image to gray scale (Required by HoughCircles routine)
        result = cv2.cvtColor(masked_blurred, cv2.COLOR_BGR2GRAY)
        # Detect circles in the image using Canny edge and Hough transform
        circles = cv2.HoughCircles(result, cv2.HOUGH_GRADIENT, 1.5, 300, param1=100, param2=20, minRadius=20, maxRadius=200)

        dotColor = (0, 255, 0)
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                #print("Circle: " + "("+str(x)+","+str(y)+")")
                # draw the circle in the output image, then draw a rectangle corresponding to the center of the circle
                # The circles and rectangles are drawn on the original image.
                cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(cv_image, (x - 5, y - 5), (x + 5, y + 5), dotColor, -1)

                self.target = x
        else:
            self.target = -1

        # ---------------------
        # End of color tracking

        # convert to compressed image and publish
        #compressed = self.bridge.cv2_to_compressed_imgmsg(cv_image)
        compressed = self.bridge.cv2_to_imgmsg(cv_image)
        self.cv_pub.publish(compressed)



def main(args=None):
    rclpy.init(args=args)

    movement_handler = MovementHandler()

    rclpy.spin(movement_handler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    movement_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
