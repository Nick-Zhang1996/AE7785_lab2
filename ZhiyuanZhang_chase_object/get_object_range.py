# subscribe to 
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from math import degrees,radians

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class GetObjectRange(Node):
    def __init__(self, publish_debug=False):
        self.publish_debug = publish_debug
        self.object_location_angle_rad = 0

        super().__init__('get_object_range')
        self.get_logger().info(f' publish_debug: {self.publish_debug}')

        self.subscription = self.create_subscription(Point,'/object_location',self.objectLocationCallback,5)
        # x: horizontal location, in degrees
        # y: distance, in meters
        self.publisher = self.create_publisher(Point, '/object_info', 5)

        #Set up QoS Profiles for passing images over WiFi
        lidar_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )

        self.subscription = self.create_subscription(LaserScan,'scan',self.newScanCallback,lidar_qos_profile)

    # /object_location: only x matters, left positive,right negative
    def objectLocationCallback(self,msg):
        self.get_logger().info(f'object detected at {msg.x} deg')
        self.object_location_angle_rad = radians(msg.x)

    # /object_location: only x matters, left positive,right negative
    def newScanCallback(self,msg):
        angle_inc = (msg.angle_max-msg.angle_min) / len(msg.ranges)
        self.get_logger().info(f'angle_inc = {angle_inc} rad = {degrees(angle_inc)} deg')
        # index at angle
        index = int((self.object_location_angle_rad - msg.angle_min)/angle_inc)
        mean_range = np.mean( msg.ranges[index] )
        self.get_logger().info(f'index = {index} range: {mean_range}')
        #self.publisher.publish(out_msg)



def main(args=None):
    rclpy.init(args=args)
    node = GetObjectRange()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
