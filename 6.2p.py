#!/usr/bin/env python3

# Required Libraries
import sys
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage

class RoadLaneDetector:
    def __init__(self):
        rospy.init_node("lane_detector_node")
        self.bridge = CvBridge()

        # Subscribe to the compressed image topic
        self.image_subscriber = rospy.Subscriber('/duckie/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)

    def image_callback(self, msg):
        rospy.loginfo("Received Image")

        # Convert compressed image message to OpenCV image
        image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Define cropping parameters
        top, bottom, left, right = 100, 400, 100, 600

        # Crop the image
        cropped_image = image[top:bottom, left:right]

        # Convert cropped image to grayscale
        grayscale_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)

        # Set thresholds for detecting white lane markings
        lower_white = 200
        upper_white = 255

        # Threshold the grayscale image to get only white pixels
        white_mask = cv2.inRange(grayscale_image, lower_white, upper_white)

        # Apply Canny Edge Detection
        edges = cv2.Canny(white_mask, 50, 150)

        # Apply Hough Transform to detect lines
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=100, minLineLength=50, maxLineGap=50)

        # Draw lines on the original image
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(cropped_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Display the image with detected lines
        cv2.imshow('Detected Lines', cropped_image)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        lane_detector = RoadLaneDetector()
        lane_detector.run()
    except rospy.ROSInterruptException:
        pass
