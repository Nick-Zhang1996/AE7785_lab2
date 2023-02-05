# load dummy images from video, publish to /camera/image/compresed

import cv2
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class PubTestImage(Node):
    def __init__(self):
        super().__init__('publish_test_image')
        #self.get_logger().info(f' publish_debug: {self.publish_debug}')
        self.publisher = self.create_publisher(CompressedImage,'/camera/image/compressed',5)
        self.timer = self.create_timer(0.1,self.publishImage)
        self.br = CvBridge()
        self.frame_count = 0

        # load image
        self.vid = cv2.VideoCapture('../../../../hw1/Video.MOV')

    def publishImage(self):
        success, frame = self.vid.read()
        if (not success):
            #self.get_logger().info(f'All images sent')
            self.get_logger().info(f'Cant read images')
            exit()
            return
        self.frame_count += 1

        frame = cv2.flip(frame,0)
        msg = self.br.cv2_to_compressed_imgmsg(frame)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'test_img'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published frame count {self.frame_count}')



def main(args=None):
    rclpy.init(args=args)
    main = PubTestImage()
    rclpy.spin(main)

    main.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
