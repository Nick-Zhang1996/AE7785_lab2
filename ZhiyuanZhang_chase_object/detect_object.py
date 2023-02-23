# find_object: This node should subscribe to receive images from the Raspberry Pi Camera on
#               the topic /camera/image/compressed. Example code in Python:
# output: 

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class DetectObject(Node):
    def __init__(self, publish_debug=False):
        self.publish_debug = publish_debug

        super().__init__('detect_object')
        self.get_logger().info(f' publish_debug: {self.publish_debug}')
        self.br = CvBridge()

        self.publisher = self.create_publisher(Point, '/object_location', 5)
        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )

        #Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
        self.subscription = self.create_subscription(CompressedImage,'/camera/image/compressed',self.newImageCallback,image_qos_profile)
        if (self.publish_debug):
            self.debug_image_publisher = self.create_publisher(CompressedImage, '/debug_img',5)

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
        ori_frame = self.br.compressed_imgmsg_to_cv2(msg)
        '''
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
        '''
        x = y = 0

        if (self.publish_debug):
            '''
            M = cv2.moments(best_contour)
            center = (int(M['m10']/M['m00']),int(M['m01']/M['m00']))
            cv2.circle(frame, (int(x),int(y)), int(radius), (0,0,255), 3)
            cv2.circle(frame, center, 5, (0,0,255),-1)
            '''
            # one way
            frame = ori_frame.copy().astype(float)
            blue = frame[:,:,0] - (frame[:,:,1]/2 + frame[:,:,2]/2)

            # another way
            hsv = cv2.cvtColor(ori_frame, cv2.COLOR_BGR2HSV)
            # kinda
            hue = hsv[:,:,0]
            # good
            saturation = hsv[:,:,1]
            value = hsv[:,:,2]

            saturation = self.normalize(saturation)
            blue = self.normalize(blue)
            blue[blue<0.1] = 0
            saturation[saturation<0.1] = 0

            frame = self.toImage(blue*saturation)
            msg = self.br.cv2_to_compressed_imgmsg(frame)
            self.debug_image_publisher.publish(msg)
            self.get_logger().info(f' published debug image ')

        x_dimless = x/frame.shape[0]-0.5
        y_dimless = y/frame.shape[1]-0.5
            
        return (x_dimless,y_dimless)
    def normalize(self,val):
        return (val - np.mean(val)) / (np.max(val) - np.min(val))
    def toImage(self,val):
        return ((self.normalize(val) + 0.5 )*255).astype(int)
        
        


def main(args=None):
    rclpy.init(args=args)
    detect_object = DetectObject(publish_debug=True)
    rclpy.spin(detect_object)

    find_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
