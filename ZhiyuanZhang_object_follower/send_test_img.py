# load dummy images from video, publish to /camera/image/compresed

import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

class PubTestImage(Node):
    def __init__(self):
        super().__init__('publish_test_image')
        self.get_logger().info(f' publish_debug: {self.publish_debug}')
        self.publisher = self.create_publisher(CompressedImage,'/camera/image/compressed',5)
        self.timer = self.create_timer(0.1,self.publish)

        # load image
        self.vid = cv2.VideoCapture('../../../hw1/Video.MOV')

    def publishImage(self):
        success, frame = vid.read()
        if (not success):
            self.get_logger().info(f'All images sent')
            # TODO suicide
            self.destroy_node()

        frame = cv2.flip(frame,0)
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().noew().to_msg()
        msg.header.frame_id = 'test_img'
        msg.format = "png"
        msg.data = frame
        self.publisher.publish(frame)


def main(args=None):
    rclpy.init(args=args)
    main = PubTestImage()
    rclpy.spin(main)

    main.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
