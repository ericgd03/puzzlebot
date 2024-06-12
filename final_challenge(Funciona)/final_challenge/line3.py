import time
import rclpy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float32
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import warnings

class line_follower_node(Node):

    def __init__(self):

        super().__init__('line_node_inter')
        self.bridge = CvBridge()
        self.topic_name_frames = 'video_source/raw'
        self.queue_size = 20

        self.locked = 'unlocked'

        self.subscription = self.create_subscription(Image, self.topic_name_frames, self.listener_callback, self.queue_size)
        self.get_logger().info('Nodo No-Linear initiated')
        self.subscription

        self.interseccion_publisher = self.create_publisher(String, 'interseccion', 10)
        self.puzzlebot_publisher = self.create_publisher(Float32, 'error', 10)
        self.err = Float32()
        self.msg = String()
        self.msg.data = 'A'

    def listener_callback(self, image_message):

        # Define the ROI coordinates for the center
        self.rx, self.ry, self.rw, self.rh = 150, 490, 800, 190
        self.rx2, self.ry2, self.rw2, self.rh2 = 280, 540, 550, 100

        # Define the minimum area of objects to consider (in pixels)
        self.min_area_threshold = 11000  # Adjust this value as needed
        self.min_area_threshold_2 = 800

        img = self.bridge.imgmsg_to_cv2(image_message)

        # Resize the image
        image = cv2.resize(img, (1080, 720))
        gaussc_image = cv2.GaussianBlur(image, (11, 11), 25)

        # Extract the ROI from the image
        roi = gaussc_image[self.ry:self.ry + self.rh, self.rx:self.rx + self.rw]
        roi_2 = gaussc_image[self.ry2:self.ry2 + self.rh2, self.rx2:self.rx2 + self.rw2]

        # Apply thresholding to the ROI
        th_roi = cv2.inRange(roi, (0, 0, 0), (150, 150, 80))
        th_roi_2 = cv2.inRange(roi_2, (0, 0, 0), (150, 150, 80))

        # Find contours in the ROI
        cnts, _ = cv2.findContours(th_roi, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts_2, _ = cv2.findContours(th_roi_2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours based on area
        cnts = [cnt for cnt in cnts if cv2.contourArea(cnt) > self.min_area_threshold]
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:1]

        cnts_2 = [cnt_2 for cnt_2 in cnts_2 if cv2.contourArea(cnt_2) > self.min_area_threshold_2]
        cnts_2 = sorted(cnts_2, key=cv2.contourArea, reverse=True)

        cv2.drawContours(roi, cnts, 0, (0, 255, 0), 2)

        for cnt in cnts_2:
            cv2.drawContours(roi_2, [cnt], 0, (0, 0, 255), 2)

        # Function to get the center of the contour
        def get_center_of_contour(cnts):

            if len(cnts) > 0:
                M = cv2.moments(cnts[0])
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return (cx, cy)
            return None

        # Get center for the center ROI
        center_center = get_center_of_contour(cnts)

        # Desired center position of the ROI
        desired_center_x = self.rw / 2

        # Proportional control logic
        if center_center is not None:
            cx, cy = center_center
            cx = float(cx)

            error = desired_center_x - cx

            # self.get_logger().info(f"Error: {error}")
            self.err.data = error * 2
            self.puzzlebot_publisher.publish(self.err)

        # Replace the processed ROI back into the image
        image[self.ry:self.ry + self.rh, self.rx:self.rx + self.rw] = roi
        image[self.ry2:self.ry2 + self.rh2, self.rx2:self.rx2 + self.rw2] = roi_2

        # Draw a rectangle to visualize the ROI on the image
        cv2.rectangle(image, (self.rx, self.ry), (self.rx + self.rw, self.ry + self.rh), (0, 255, 0), 1)
        cv2.rectangle(image, (self.rx2, self.ry2), (self.rx2 + self.rw2, self.ry2 + self.rh2), (0, 0, 255), 1)

        #Si detecta la interseccion
        if len(cnts_2) >= 3:
            print("Intersecccccion", len(cnts_2)) # Agregar nodo señales como interseccion
            self.interseccion_publisher.publish(self.msg)


        # Display the processed image (optional)
        #cv2.imshow('Processed Image', image)
        #cv2.waitKey(1)

    def error_callback(self, msg):
        self.err = msg.data

def main(args=None):

    rclpy.init(args=args)
    l_f_n = line_follower_node()
    rclpy.spin(l_f_n)
    l_f_n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()