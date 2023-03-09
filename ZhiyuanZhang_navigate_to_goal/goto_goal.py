# all in one
import rclpy
from rclpy.node import Node
from math import degrees,radians,cos,sin
import numpy as np
from time import sleep,time
from threading import Event,Thread
import matplotlib.pyplot as plt

from geometry_msgs.msg import Point,Twist,Pose,PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class GotoGoal(Node):
    def __init__(self):
        super().__init__('goto_goal')
        self.Init = Event()
        self.Init.set()
        # scan is enabled
        self.enable_scan = Event()
        # found an obstacle blocking path
        self.found_obscale = Event()
        # identified location of obstacle
        self.located_obstacle = Event()

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
        #self.fig, self.ax = plt.subplots(1, 2, figsize=(10, 10))
        #plt.clf()
        self.fig = plt.gcf()
        self.ax = self.fig.gca()


    def newScanCallback(self,msg):
        if (not self.enable_scan.is_set()):
            return

        # try to find obstacle
        if (not self.found_obscale.is_set() and not self.located_obstacle.is_set()):
            # ~0 -> 2pi
            angle_inc = (msg.angle_max-msg.angle_min) / len(msg.ranges)
            angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            angles = (angles + np.pi) % (2*np.pi) - np.pi
            mask = np.bitwise_and(angles < radians(45), angles > -radians(45))
            ranges = np.array(msg.ranges)
            roi = ranges[mask] < 1
            obstacle_point = np.sum(roi)
            #print(f'candidate point {obstacle_point}')

            if (obstacle_point < 15):
                return
            if (not self.found_obscale.is_set()):
                # we've found obstacle, haven't located it yet
                print(f'found obstacle, waiting 5 seconds before locating')
                self.found_obscale.set()
                self.enable_scan.clear()

            scan_x = np.cos(angles[mask]) * ranges[mask]
            scan_y = np.sin(angles[mask]) * ranges[mask]
            accumulator, thetas, rhos = self.hough_line(scan_x, scan_y)
            retval = self.getBestHough(scan_x, scan_y, accumulator, thetas, rhos)
            if (retval is None):
                return
            centroid, orientation = retval
            #print(f'orientation: {degrees(orientation)}')

            # line: rho = x*cos(theta) + y*sin(theta)
            if (False):
                ax = self.ax
                self.show_scan(ax, angles, ranges, mask)
                #self.show_hough_line(ax[1], accumulator, thetas, rhos)

                print(centroid)
                circle = plt.Circle(centroid,0.03, color='b')
                ax.add_patch(circle)
                plt.pause(0.05)

            # found an obstacle
            local_angle = np.arctan2(centroid[1],centroid[0])
            local_distance = np.sqrt(centroid[1]**2 + centroid[0]**2)
            global_x = self.globalPos.x + np.cos(self.globalAng + local_angle)*local_distance
            global_y = self.globalPos.y + np.sin(self.globalAng + local_angle)*local_distance
            global_orientation = orientation + self.globalAng

            self.obstacle_location = (global_x,global_y)
            self.obstacle_orientation = orientation

            self.located_obstacle.set()
            self.get_logger().info(f'found obstacle at {self.obstacle_location, degrees(orientation)}deg')


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
            #sleep(30)
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
        sleep(3)

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

        #0 1.4
        self.get_logger().info(f'going to third node 0,1.4')
        self.enable_scan.set()
        # more forward carefully, while scanning
        msg = Twist()
        msg.linear.x = v_linear
        self.publisher.publish(msg)

        t0 = time()
        while time()-t0 < (1.4/v_linear+dt_linear):
            if (self.found_obscale.wait(0.1)):
                break
            #self.get_logger().info(f'no obstacle yet... {time()-t0}')


        # stop !
        msg = Twist()
        self.publisher.publish(msg)
        sleep(0.1)
        if (self.found_obscale.is_set()):
            self.get_logger().info(f'obstacle found !')
        else:
            self.get_logger().info(f'arrived without obstacle')
            exit(0)

        # found obstacle, wait 5 seconds for obstacle to settle
        sleep(3)
        # re-enable scan
        self.enable_scan.set()
        if (not self.located_obstacle.wait(2)):
            print(f'did not get a fix on obstacle')

        # turn_left
        remaining_x = self.globalPos.x - 0.2
        # + -> obstacle is to the left
        lateral = self.globalPos.y - self.obstacle_location[1]
        print(f'current pos {self.globalPos}')
        print(f'remaining distance: {remaining_x}')
        print(f'lateral: {lateral}')

        if (self.obstacle_orientation > 0):
            # obstacle facing left, evade from left side
            self.turnLeft()
            self.forward(0.4 + lateral)
            self.turnRight()
            self.forward(remaining_x)
            self.turnRight()
            self.forward(0.4 + lateral)
        else:
            self.turnRight()
            self.forward(0.4 - lateral)
            self.turnLeft()
            self.forward(remaining_x)
            self.turnLeft()
            self.forward(0.4 - lateral )

        self.get_logger().info(f'arrived at third node 0,1.4')

        # find obstacle bounding box 
        # replan
    def turnLeft(self):
        print('turn left')
        dt_linear = 0.9
        dt_turn = 0.3
        v_linear = 0.15
        dt = 1
        msg = Twist()
        msg.angular.z = 0.3
        self.publisher.publish(msg)
        sleep(radians(90)/0.3+dt_turn)

        msg = Twist()
        self.publisher.publish(msg)
        sleep(1)

    def turnRight(self):
        print('turn right')
        dt_linear = 0.9
        dt_turn = 0.3
        v_linear = 0.15
        dt = 1
        msg = Twist()
        msg.angular.z = -0.3
        self.publisher.publish(msg)
        sleep(radians(90)/0.3+dt_turn)

        msg = Twist()
        self.publisher.publish(msg)
        sleep(1)

    def forward(self,distance):
        print(f'forward {distance}')
        dt_linear = 0.9
        dt_turn = 0.3
        v_linear = 0.15
        dt = 1

        msg = Twist()
        msg.linear.x = v_linear
        self.publisher.publish(msg)
        sleep(distance/v_linear+dt_linear)
        msg = Twist()
        self.publisher.publish(msg)
        sleep(dt)


    def hough_line(self,scan_x,scan_y, angle_step=1):
        """
        Hough transform for lines

        Input:
        angle_step - Spacing between angles to use every n-th angle
                     between -90 and 90 degrees. Default step is 1.
        lines_are_white - boolean indicating whether lines to be detected are white
        value_threshold - Pixel values above or below the value_threshold are edges

        Returns:
        accumulator - 2D array of the hough transform accumulator
        theta - array of angles used in computation, in radians.
        rhos - array of rho values. Max size is 2 times the diagonal
               distance of the input image.
        """
        # lilne: rho = x*cos(theta) + y*sin(theta)

        # Rho and Theta ranges
        thetas = np.deg2rad(np.arange(-90.0, 90.0, angle_step))
        n = 50
        # [-1,1] -> [0,n]
        rhos = (np.linspace(-1, 1, 2*n) + 1 )*n
        rhos = rhos.astype(np.int32)

        # Cache some reusable values
        cos_t = np.cos(thetas)
        sin_t = np.sin(thetas)
        num_thetas = len(thetas)

        # Hough accumulator array of theta vs rho
        accumulator = np.zeros((2*n+1, num_thetas), dtype=np.uint8)

        # Vote in the hough accumulator
        for i in range(len(scan_x)):
            x = scan_x[i]
            y = scan_y[i]

            for t_idx in range(num_thetas):
                # Calculate rho. diag_len is added for a positive index
                rho = x * cos_t[t_idx] + y * sin_t[t_idx]
                if (np.isnan(rho) or rho > 1 or rho < -1):
                    continue
                rho_idx = int((rho + 1 ) *n)
                accumulator[rho_idx, t_idx] += 1

        return accumulator, thetas, rhos

    def getBestHough(self, scan_x, scan_y, accumulator, thetas, rhos,angle_step=1):
        indices = np.unravel_index(np.argmax(accumulator), accumulator.shape)
        count = accumulator[indices[0], indices[1]]
        print(f'candidate count = {count}')
        if (count < 10):
            return None
        # essentially redu hough, find all associated points
        n = 50
        thetas = np.deg2rad(np.arange(-90.0, 90.0, angle_step))
        points = []
        rho_idx = indices[0]
        theta_idx = indices[1]
        rho = rho_idx/n-1
        theta = thetas[theta_idx]


        # Cache some reusable values
        cos_t = np.cos(thetas)
        sin_t = np.sin(thetas)
        num_thetas = len(thetas)

        for i in range(len(scan_x)):
            x = scan_x[i]
            y = scan_y[i]
            if (np.isnan(x)):
                continue

            this_rho = x * cos_t[theta_idx] + y * sin_t[theta_idx]
            this_rho_idx = int((this_rho + 1 ) *n)
            if (this_rho_idx == rho_idx):
                points.append( [scan_x[i],scan_y[i]] )
        points = np.array(points)
        centroid = (points[:,0].mean(), points[:,1].mean())
        orientation = np.arctan(-1/np.tan(theta))

        return centroid, orientation

    def show_scan(self, ax, angles, ranges, mask):
        ax.cla()
        scan_x = np.cos(angles) * ranges
        scan_y = np.sin(angles) * ranges
        ax.scatter(scan_x,scan_y,color='b')

        # ROI
        scan_x = np.cos(angles[mask]) * ranges[mask]
        scan_y = np.sin(angles[mask]) * ranges[mask]
        ax.scatter(scan_x,scan_y,color='k')

        # robot location
        circle = plt.Circle((0,0),0.1, color='r')
        ax.add_patch(circle)
        ax.set_xlim([-1,1])
        ax.set_ylim([-1,1])
        ax.set_aspect('equal','box')
        # robot FOV (ROI)
        ax.plot([0, cos(radians(45))],[0, sin(radians(45))] )
        ax.plot([0, cos(radians(-45))],[0, sin(-radians(45))] )

        return



    def show_hough_line(self, ax, accumulator, thetas, rhos):
        ax.imshow(
            accumulator, cmap='jet',
            extent=[np.rad2deg(thetas[-1]), np.rad2deg(thetas[0]), rhos[-1], rhos[0]])
        ax.set_aspect('equal', adjustable='box')
        ax.set_title('Hough transform')
        ax.set_xlabel('Angles (degrees)')
        ax.set_ylabel('Distance (pixels)')
        ax.axis('image')
        return



def main(args=None):
    rclpy.init(args=args)
    node = GotoGoal()


    #node.enable_scan.set()
    #rclpy.spin(node)

    thread = Thread(target=rclpy.spin, args=(node,),daemon=True)
    thread.start()
    node.main()

    thread.join()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
