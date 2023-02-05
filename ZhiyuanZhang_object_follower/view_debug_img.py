
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage


class ViewDebugImg(Node):
    def __init__(self):

        super().__init__('find_object')
        #self.get_logger().info(f' publish_debug: {self.publish_debug}')
        self.br = CvBridge()

        self.subscription = self.create_subscription(CompressedImage,'/camera/image/compressed',self.newImageCallback,5)

    def newImageCallback(self,msg):
        frame = self.br.compressed_imgmsg_to_cv2(msg)
        cv2.imshow('frame',frame)
        key = cv2.waitKey(50) & 0xFF
        return


def main(args=None):
    rclpy.init(args=args)
    main = ViewDebugImg()
    rclpy.spin(main)

    main.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
