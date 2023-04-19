#!/usr/bin/env python

#twist mux is used to help with ros topics for requires downloading to root on every launch 
# download guide below 
# sudo apt update
# sudo apt install ros-humble-twist-mux

# This code was adapted from scripts found on this github page 
#https://github.com/LCAS/teaching/tree/lcas_humble/cmp3103m_ros2_code_fragments


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class collisionAvoidance(Node):
    def __init__(self):

        super().__init__('collisionAvoidance')
        # Create a publisher for the twist message
        self.twist_pub = self.create_publisher(Twist, 'collision_vel', 10)
        # Create a subscriber for the laser scan message
        self.laser_sub = self.create_subscription(LaserScan, 'laser_scan', self.collisionCallback, 10)

    def collisionCallback(self, msg):
        """
        This callback is called for every LaserScan received. 
        If it detects obstacles within the segment of the LaserScan it turns, 
        if the space is clear, it moves forward.
        """
        collisionDetected = False 
        robotMove = Twist()

        forwardView = msg.ranges[0] > 0.8
        #Check the range values at indices 1 to 60 and -1 to -60
        for empty in range(60):
            # If all the range values are greater than 0.8, set forwardView to True
            forwardView = forwardView and msg.ranges[empty] > 0.8
            forwardView = forwardView and msg.ranges[-empty] > 0.8

        for rot in range(0, len(msg.ranges), 30):

            if (msg.ranges[rot] < 0.8 and forwardView):
                #print out message to show that the front of the robot is clear
                print(f"Seems to be clear in front")
                robotMove.linear.x = 0.25
                robotMove.angular.z = 0.0
                collisionDetected = True


            elif (msg.ranges[rot] < 0.8 or forwardView == False):
                #print out message to show that the front of the robot is not clear
                print(f"Collision detected at {rot}")
                #stops the robot
                robotMove.linear.x = 0.0
                collisionDetected = True

                # decides which side of the robot is clear and turns the robot to the right or left
                if rot >= 180:
                    robotMove.angular.z = -0.3
                else:
                    robotMove.angular.z = 0.3

        # output message 
        if collisionDetected == True:
            self.twist_pub.publish(robotMove)

 



def main(args=None):
    print('Starting collision.py.')

    rclpy.init(args=args)

    collision = collisionAvoidance()

    rclpy.spin(collision)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    collision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()