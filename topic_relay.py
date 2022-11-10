#!/usr/bin/env python3

import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.clock import Clock
from nav_msgs.msg import Odometry

class realsense_bridge(Node):

    def __init__(self):
        super().__init__('realsense_bridge')
        self.odometry_sub = self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.odometry_callback, qos.qos_profile_sensor_data)
        self.odometry_pub = self.create_publisher(Odometry,  '/bitch', 5)
        self.odom_output = Odometry()
    
    def odometry_callback(self, data):
        self.odom_output = data
        self.odom_output.header.frame_id = data.header.frame_id
        self.odom_output.child_frame_id = data.child_frame_id 
        self.odometry_pub.publish(self.odom_output)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = realsense_bridge()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

