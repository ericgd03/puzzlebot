import time
import rclpy
import cv2
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

        self.puzzlebot_publisher = self.create_publisher(String, 'angular', 10)
        self.msg = String()

    def listener_callbackFunction(self, imageMessage):

        # Define the ROI coordinates
        rx, ry, rw, rh = 220, 510, 590, 210

        # Define the minimum area of objects to consider (in pixels)
        min_area_threshold = 900  # Adjust this value as needed

        img = self.bridgeObject.imgmsg_to_cv2(imageMessage)

        # Resize the image
        image = cv2.resize(img, (1080, 720))

        # Extract the ROI from the image
        roi = image[ry:ry + rh, rx:rx + rw]

        # Convert the ROI to grayscale
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Apply thresholding to the ROI
        th_roi = cv2.inRange(roi, (0, 0, 0), (100, 100, 100))
        
        # Find contours in the ROI
        cnts, _ = cv2.findContours(th_roi, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours based on area
        cnts = [cnt for cnt in cnts if cv2.contourArea(cnt) > min_area_threshold]
        
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:1]
        cv2.drawContours(roi, cnts, 0, (0, 255, 0), 3)

        if len(cnts) > 0:
            # Get the bounding rectangle for the first contour
            x, y, w, h = cv2.boundingRect(cnts[0])

            # Get the minimum area rectangle (which can be rotated)
            blackbox = cv2.minAreaRect(cnts[0])
            (x_min, y_min), (w_min, h_min), ang = blackbox

            # Correct the angle if it is less than -45 degrees
            if ang < -45:
                ang = 90 + ang

            # Further adjust the angle based on the dimensions of the rectangle
            if w_min < h_min and ang > 0:
                ang = (90 - ang) * -1
            if w_min > h_min and ang < 0:
                ang = 90 + ang

            print("Angular:", ang)
            self.msg.data = str(ang)
            self.puzzlebot_publisher.publish(self.msg)
            
        # Replace the processed ROI back into the image
        image[ry:ry + rh, rx:rx + rw] = roi

        # Draw a rectangle to visualize the ROI on the image
        cv2.rectangle(image, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
        
        # Display the processed image
        #cv2.imshow('Processed Image', image)
        #cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    suscriberNode = SuscriberNodeClass()
    rclpy.spin(suscriberNode)
    suscriberNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()