# all in one
import rclpy
from rclpy.node import Node
from math import degrees,radians
import numpy as np
from time import sleep,time
from threading import Event,Thread

from geometry_msgs.msg import Point,Twist,Pose,PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class GotoGoal(Node):
    def __init__(self):
        super().__init__('goto_goal')
        self.Init = Event()
        self.Init.set()
        self.enable_scan = Event()
        self.found_obscale = Event()

        self.globalPos = Point()
        self.Init_pos = Point()

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


    def newScanCallback(self,msg):
        if (not self.enable_scan.is_set()):
            return

        # try to find obstacle
        if (not self.found_obscale.is_set()):
            # ~0 -> 2pi
            angle_inc = (msg.angle_max-msg.angle_min) / len(msg.ranges)
            angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            angles = (angles + np.pi) % (2*np.pi) - np.pi
            mask = np.bitwise_and(angles < radians(45), angles > -radians(45))
            roi = np.array(msg.ranges)[mask] < 0.3
            obstacle_point = np.sum(roi)
            print(obstacle_point)
            #self.found_obscale.set()
            #self.get_logger().info(f'found obstacle')

    def newOdomCallback(self, Odom):
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init.is_set():
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init.clear()
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang

        #self.get_logger().info(f'{self.globalPos}')
        #self.get_logger().info(f'{self.globalAng}')


    def main(self):
        try:
            self.get_logger().info(f'running sequence')
            self.sequence()
            #sleep(10)
        except KeyboardInterrupt:
            pass
        finally:
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
        '''
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
        self.get_logger().info(f'{self.globalPos}')
        self.get_logger().info(f'{self.globalAng}')
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
        self.get_logger().info(f'{self.globalPos}')
        self.get_logger().info(f'{self.globalAng}')
        sleep(dt)

        self.get_logger().info(f'turning 90 deg ccw')
        msg = Twist()
        msg.angular.z = 0.3
        self.publisher.publish(msg)

        sleep(radians(90)/0.3+dt_turn)

        msg = Twist()
        self.publisher.publish(msg)
        self.get_logger().info(f'done turning')
        self.get_logger().info(f'{self.globalPos}')
        self.get_logger().info(f'{self.globalAng}')
        sleep(dt)

        self.get_logger().info(f'going to intermediate node 1.5+0.45,1.4')
        msg = Twist()
        msg.linear.x = v_linear
        self.publisher.publish(msg)

        sleep(1.4/v_linear)
        msg = Twist()
        self.publisher.publish(msg)
        self.get_logger().info(f'arrived at intermediate node 1.5+0.45,1.4')
        self.get_logger().info(f'{self.globalPos}')
        self.get_logger().info(f'{self.globalAng}')
        sleep(dt)

        self.get_logger().info(f'turning 90 deg ccw')
        msg = Twist()
        msg.angular.z = 0.3
        self.publisher.publish(msg)

        sleep(radians(90)/0.3+dt_turn)

        msg = Twist()
        self.publisher.publish(msg)
        self.get_logger().info(f'done turning')
        self.get_logger().info(f'{self.globalPos}')
        self.get_logger().info(f'{self.globalAng}')
        sleep(dt)

        self.get_logger().info(f'going to second node 1.5,1.4')
        msg = Twist()
        msg.linear.x = v_linear
        self.publisher.publish(msg)

        sleep(0.45/v_linear+dt_linear)

        msg = Twist()
        msg.linear.x = 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f'arrived at second node 1.5,1.4')
        self.get_logger().info(f'{self.globalPos}')
        self.get_logger().info(f'{self.globalAng}')
        sleep(dt)
        '''

        #0 1.4
        self.get_logger().info(f'going to third node 0,1.4')
        self.enable_scan.set()
        msg = Twist()
        msg.linear.x = v_linear
        #self.publisher.publish(msg)
        t0 = time()
        while time()-t0 < (1.4/v_linear+dt_linear):
            if (self.found_obscale.wait(0.1)):
                break
            #self.get_logger().info(f'no obstacle yet... {time()-t0}')
        if (self.found_obscale.is_set()):
            self.get_logger().info(f'obstacle found !')

        self.get_logger().info(f'arrived at third node 0,1.4')


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
