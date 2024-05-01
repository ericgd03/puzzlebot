import cv2
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np


import warnings


class SuscriberNodeClass(Node):
    def __init__(self):
        super().__init__('suscriber_node')
        self.bridgeObject = CvBridge()
        self.topicNameFrames='video_source/raw'
        self.queueSize = 20

        self.subscription = self.create_subscription(Image, self.topicNameFrames,self.listener_callbackFunction,self.queueSize)
        self.get_logger().info('Image Subscription initiated')
        self.subscription

        self.puzzlebot_publisher = self.create_publisher(String, 'semaforo', 10)
        self.msg = String()

    def listener_callbackFunction(self, imageMessage):
        frame = self.bridgeObject.imgmsg_to_cv2(imageMessage)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define ranges for yellow, red, and green
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])

        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        masks = [mask_yellow, mask_red, mask_green]

        for idx, mask in enumerate(masks):
            res = cv2.bitwise_and(frame, frame, mask=mask)
            img = cv2.medianBlur(res, 5)
            ccimg = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
            cimg = cv2.cvtColor(ccimg, cv2.COLOR_BGR2GRAY)
            circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1, 100, param1=50, param2=22, minRadius=20, maxRadius=100,)
            if circles is not None:
                print(["Yellow", "Red", "Green"][idx])
                print(img.shape)
                circles = np.uint16(np.around(circles))
                self.msg.data = ['Yellow', 'Red', 'Green'][idx]
                self.puzzlebot_publisher.publish(self.msg)
                for i in circles[0, :]:
                    cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)



def main(args=None):
    rclpy.init(args=args)
    suscriberNode = SuscriberNodeClass()
    rclpy.spin(suscriberNode)
    suscriberNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()