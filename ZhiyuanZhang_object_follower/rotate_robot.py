# rotate_robot: This node should subscribe to the object coordinate messages from your find_object
#                node and publish velocity commands to the robot:
# subscribe to: /object_location, geometry_msgs/msg/Point (x,y,z)
# publish to: /cmd_vel, geometry_msgs/msg/Twist (.linear.{x,y,z}, .angular), only angular z



# find_object: This node should subscribe to receive images from the Raspberry Pi Camera on
#               the topic /camera/image/compressed. Example code in Python:
# self._img_subscriber = self.create_subscription(CompressedImage,'/camera/image/compressed', self._image_callback, qos_profile)

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point,Twist

class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 5)
        self.subscription = self.create_subscription(Point,'/object_location',self.objectLocationCallback,5)

    def objectLocationCallback(self,msg):
        self.get_logger().info(f'received: {msg}')
        out_msg = Twist()
        out_msg.angular.z = msg.z * 2
        self.publisher.publish(out_msg)
        self.get_logger().info(f'Publishing angular.z={out_msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    rotate_robot = RotateRobot()
    rclpy.spin(rotate_robot)

    rotate_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
