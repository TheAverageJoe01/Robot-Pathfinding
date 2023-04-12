## Collision package utilising the laser scanner
## Can only detect objects that are on the floor
## Will not deal with object propped up, like benches, tables etc

import rclpy
from rclpy.node import Node

# Gets twist method to move robot
from geometry_msgs.msg import Twist
# sensor_msgs/msg/LaserScan - Laser scanner for collision detection
from sensor_msgs.msg import LaserScan

class stopColl(Node):
    def __init__(self):
        super().__init__('stop_collisions') # calls super's initialiser (so Node init method) and names node
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan', # subscribes to the scan topic
            self.callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # creates publisher


    def callback(self, msg):
        move = Twist()
        ## msg contains scan data
        # range list contains all groups of degrees, index has group of degrees

        # This is forwards
        if (msg.ranges[0] < 1):
            print("0")
            move.linear.x = 0.0
            move.angular.z = 0.0
        # This is left
        elif (msg.ranges[90] < 0.5):
            print("90")
            move.linear.x = 0.0
            move.angular.z = 0.0
        elif (msg.ranges[180] < 1):
            print("180")
            move.linear.x = 0.0
            move.angular.z = 0.0
        # Right
        elif (msg.ranges[270] < 0.5):
            print("270")
            move.linear.x = 0.0
            move.angular.z = 0.0


        # output message when both condition and !condition 
        self.publisher_.publish(move)
    


def main(args=None):
    rclpy.init(args=args)

    node_ = stopColl()

    rclpy.spin(node_)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()