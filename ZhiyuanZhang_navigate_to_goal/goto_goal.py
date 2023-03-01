# all in one
import rclpy
from rclpy.node import Node
from math import degrees,radians
import numpy as np

from geometry_msgs.msg import Point,Twist,Pose,PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from time import sleep,time

from threading import Event,Thread

class GotoGoal(Node):
    def __init__(self):
        super().__init__('goto_goal')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 5)
        #Set up QoS Profiles for passing images over WiFi
        lidar_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )

        self.sub_scan = self.create_subscription(LaserScan,'scan',self.newScanCallback,lidar_qos_profile)
        self.sub_odom = self.create_subscription(Odometry,'/odom',self.newOdomCallback,5)

        self.enable_scan = Event()
        self.enable_scan.set()

    def newScanCallback(self,msg):
        # report direct obstacle
        # after report, scan obstacle pose , report
        #self.get_logger().info(f'got scan message')
        if (not self.enable_scan.is_set()):
            return

    def newOdomCallback(self,msg):
        self.get_logger().info(f'{msg.pose}')

        return

    def main(self):
        try:
            self.get_logger().info(f'running sequence')
            self.sequence()
        except KeyboardInterrupt:
            self.get_logger().info(f'halting')
            msg = Twist()
            self.publisher.publish(msg)
            sleep(1)
            exit(0)


    def sequence(self):
        dt_linear = 0.9
        dt_turn = 0.3
        v_linear = 0.15
        dt = 1
        # go to first node
        self.get_logger().info(f'starting')
        sleep(dt)

        #1.5 0
        self.get_logger().info(f'going to first node 1.5,0')
        msg = Twist()
        msg.linear.x = v_linear
        self.publisher.publish(msg)
        sleep(1.5/v_linear+dt_linear)
        msg.linear.x = 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f'arrived at first node 1.5,0')
        sleep(10)

        #1.5 1.4
        self.get_logger().info(f'going to second node 1.5,1.4')
        # move to 1.5+0.45, 0
        self.get_logger().info(f'going to intermediate node 1.5+0.45,0')
        msg.linear.x = v_linear
        self.publisher.publish(msg)

        sleep(0.45/v_linear+dt_linear)

        msg = Twist()
        msg.linear.x = 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f'arrived at intermediate node 1.5+0.45,0')
        sleep(dt)

        self.get_logger().info(f'turning 90 deg ccw')
        msg = Twist()
        msg.angular.z = 0.3
        self.publisher.publish(msg)

        sleep(radians(90)/0.3+dt_turn)

        msg = Twist()
        self.publisher.publish(msg)
        self.get_logger().info(f'done turning')
        sleep(dt)

        self.get_logger().info(f'going to intermediate node 1.5+0.45,1.4')
        msg = Twist()
        msg.linear.x = v_linear
        self.publisher.publish(msg)

        sleep(1.4/v_linear)
        msg = Twist()
        self.publisher.publish(msg)
        self.get_logger().info(f'arrived at intermediate node 1.5+0.45,1.4')
        sleep(dt)

        self.get_logger().info(f'turning 90 deg ccw')
        msg = Twist()
        msg.angular.z = 0.3
        self.publisher.publish(msg)

        sleep(radians(90)/0.3+dt_turn)

        msg = Twist()
        self.publisher.publish(msg)
        self.get_logger().info(f'done turning')
        sleep(dt)

        self.get_logger().info(f'going to second node 1.5,1.4')
        msg = Twist()
        msg.linear.x = 0.2
        self.publisher.publish(msg)

        sleep(0.45/0.2+dt_linear)

        msg = Twist()
        msg.linear.x = 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f'arrived at second node 1.5,1.4')
        sleep(dt)

        #0 1.4
        self.get_logger().info(f'going to third node 0,1.4')
        # wait for obstacle to appear
        # wait while obstacle becomes stationary
        # find obstacle bounding box 
        # replan


def main(args=None):
    rclpy.init(args=args)
    node = GotoGoal()
    thread = Thread(target=rclpy.spin, args=(node,),daemon=True)
    thread.start()

    node.main()

    thread.join()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
