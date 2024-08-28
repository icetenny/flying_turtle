#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


def image_callback(ros_image):
    # Convert the ROS Image message to an OpenCV image
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridgeError: {}".format(e))
        return

    # Display the image
    cv2.imshow("Webcam Feed", cv2.resize(cv_image, (640, 480)))
    print("Received", rospy.Time())

    # Wait for a key event for 1 millisecond
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User pressed 'q' key")


def main():
    # Initialize the ROS node
    rospy.init_node('webcam_subscriber', anonymous=True)

    # Create a subscriber to the webcam image topic
    rospy.Subscriber('webcam/compressed', CompressedImage,
                     image_callback, queue_size=1)

    # Spin to keep the script alive and processing callbacks
    rospy.spin()

    # Close all OpenCV windows when done
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # Create a CvBridge object
    bridge = CvBridge()
    print('Start Cam Sub')

    try:
        main()
    except rospy.ROSInterruptException:
        pass
