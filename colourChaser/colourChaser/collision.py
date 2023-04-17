import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class collisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance_node')
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, 'laser_scan', self.scan_callback, 10)
        self.twist_msg_ = Twist()
        self.collision_detected_ = False

    def scan_callback(self, msg):
        # Implement your collision avoidance logic here
        # Assume that if any of the laser scan ranges are less than a certain threshold, collision is detected
        for range_val in msg.ranges:
            if range_val < 0.2:
                self.collision_detected_ = True
                break
        else:
            self.collision_detected_ = False

        if self.collision_detected_:
            # If collision is detected, stop the robot
            self.twist_msg_.linear.x = 0.0
            self.twist_msg_.angular.z = 0.0
        else:
            # If no collision, continue moving forward
            self.twist_msg_.linear.x = 0.2
            self.twist_msg_.angular.z = 0.0

        self.twist_pub.publish(self.twist_msg_)

def main(args=None):
    rclpy.init(args=args)
    node = collisionAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()