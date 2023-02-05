# find_object: This node should subscribe to receive images from the Raspberry Pi Camera on
#               the topic /camera/image/compressed. Example code in Python:

import cv2
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage

class FindObject(Node):
    def __init__(self, publish_debug=False):
        self.publish_debug = publish_debug

        super().__init__('find_object')
        self.get_logger().info(f' publish_debug: {self.publish_debug}')

        self.publisher = self.create_publisher(Point, '/object_location', 5)
        self.subscription = self.create_subscription(CompressedImage,'/camera/image/compressed',self.newImageCallback,5)
        if (self.publish_debug):
            self.debug_image_publisher = self.create_subscription(CompressedImage, '/debug_img',5)

    # /object_location: only x matters, left positive,right negative
    def newImageCallback(self,msg):
        x,y = self.processImage(msg)

        #self.get_logger().info(f'received: {msg}')
        out_msg = Point()
        out_msg.x = x # object location, x,y, x is horizontal location
        out_msg.y = y 
        self.publisher.publish(out_msg)
        self.get_logger().info(f'Object detected at {out_msg.x,out_msg.y}')

    def processImage(self,msg):
        frame = msg.data
        blurred = cv2.GaussianBlur(frame, (11,11),0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        saturation = hsv[:,:,1]
        lower = 200
        higher = 255
        mask = cv2.inRange(saturation, lower, higher)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        area_vec = [cv2.contourArea(contour) for contour in contours]
        # DEBUG
        #cv2.drawContours(frame, contours, -1, (0,255,0), 3)

        index = np.argmax(area_vec)
        best_contour = contours[index]

        ((x,y), radius) = cv2.minEnclosingCircle(best_contour)

        if (publish_debug):
            M = cv2.moments(best_contour)
            center = (int(M['m10']/M['m00']),int(M['m01']/M['m00']))
            print(center)
            cv2.circle(frame, (int(x),int(y)), int(radius), (0,0,255), 3)
            cv2.circle(frame, center, 5, (0,0,255),-1)
            self.debug_image_publisher.publish(msg)
            
        return (x,y)


def main(args=None):
    rclpy.init(args=args)
    find_object = FindObject(publish_debug=True)
    rclpy.spin(find_object)

    find_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
