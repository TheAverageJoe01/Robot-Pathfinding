#!/usr/bin/env python

# An example of TurtleBot 3 subscribe to camera topic, mask colours, find and display contours, and move robot to center the object in image frame
# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

class ColourChaser(Node):
    def __init__(self):
        super().__init__('colourChaser')
        
        self.turn_vel = 0.0

        # publish cmd_vel topic to move the robot
        self.pub_vel = self.create_publisher(Twist, 'colour_vel', 10)

        # create timer to publish cmd_vel topic
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # subscribe to the camera topic
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def camera_callback(self, data):
        #self.get_logger().info("camera_callback")

        cv2.namedWindow("Image window", 1)

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Convert image to HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        # Create mask for range of colours (HSV low values, HSV high values)
        #current_frame_mask = cv2.inRange(current_frame_hsv,(70, 0, 50), (150, 255, 255))
        current_frame_maskYellow = cv2.inRange(current_frame_hsv,(22, 100, 100), (30, 255, 255)) # Yellow
        current_frame_maskGreen = cv2.inRange(current_frame_hsv,(34,100,0), (86,255,255)) # Green
        current_frame_maskBlue = cv2.inRange(current_frame_hsv,(100,100,0), (130,255,255)) # Blue
        current_frame_maskRed = cv2.inRange(current_frame_hsv,(170,100,0), (180,255,255)) # Red
        current_frame_maskRed2 = cv2.inRange(current_frame_hsv,(0,100,0), (5,255,255)) # Red


        contoursYellow, hierarchy = cv2.findContours(current_frame_maskYellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursGreen, hierarchy = cv2.findContours(current_frame_maskGreen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursBlue, hierarchy = cv2.findContours(current_frame_maskBlue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursRed, hierarchy = cv2.findContours(current_frame_maskRed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursRed2, hierarchy = cv2.findContours(current_frame_maskRed2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Add together all colour contours
        contours = contoursYellow + contoursGreen + contoursBlue + contoursRed + contoursRed2
        

        # Sort by area (keep only the biggest one)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

        # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        current_frame_contours = cv2.drawContours(current_frame, contours, 0, (0, 255, 0), 20)
        
        if len(contours) > 0:
            if (len(contours[0]) > 10): # if largest contour is larger than 200  
                # find the centre of the contour: https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
                M = cv2.moments(contours[0]) # only select the largest controur
                if M['m00'] > 0:
                    # find the centroid of the contour
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    #print("Centroid of the biggest area: ({}, {})".format(cx, cy))

                    # Draw a circle centered at centroid coordinates
                    # cv2.circle(image, center_coordinates, radius, color, thickness) -1 px will fill the circle
                    cv2.circle(current_frame, (round(cx), round(cy)), 50, (0, 255, 0), -1)
                                
                    # find height/width of robot camera image from ros2 topic echo /camera/image_raw height: 1080 width: 1920

                    # if center of object is to the left of image center move left
                    if cx < 900:
                        self.turn_vel = 0.3
                    # else if center of object is to the right of image center move right
                    elif cx >= 1200:
                        self.turn_vel = -0.3
                    else: # center of object is in a 100 px range in the center of the image so dont turn
                        #print("object in the center of image")
                        self.turn_vel = 0.0
            else:
                print("No Centroid Large Enougth Found")
                # turn until we can see a coloured object
                self.turn_vel = 0.3
                        
        else:
            print("No Centroid Found")
            # turn until we can see a coloured object
            self.turn_vel = 0.3

        # show the cv images
        current_frame_contours_small = cv2.resize(current_frame_contours, (0,0), fx=0.4, fy=0.4) # reduce image size
        cv2.imshow("Image window", current_frame_contours_small)
        cv2.waitKey(1)

    def timer_callback(self):
        #print('entered timer_callback')

        self.tw=Twist() # twist message to publish

        self.tw.angular.z = self.turn_vel

        self.tw.linear.x = 0.25 # Changed to move robot forwards

        self.pub_vel.publish(self.tw)

def main(args=None):
    print('Starting colourChaser.py.')

    rclpy.init(args=args)

    colourChaser = ColourChaser()

    rclpy.spin(colourChaser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colourChaser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




## sudo apt update
## install it 
      # sudo apt install ros-humble-twist-mux
## move launch and config folder over
# Add path into the data files
# in launch change twist_mux/cmd_vel to /cmd_vel



# Change stuff in yaml files