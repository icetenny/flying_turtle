#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Range
from flying_turtle.msg import CamHeight

camera_height = 0


def image_callback(msg):
    # Convert the ROS Image message to an OpenCV image
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(msg.image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridgeError: {}".format(e))
        return

    # Display the image
    print("Received", msg.height)

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 3
    color = (0, 0, 255)  # White color
    thickness = 2
    position = (10, 70)  # Bottom-left corner of the frame

    # Apply the text to the frame
    cv2.putText(cv_image, str(msg.height), position, font,
                font_scale, color, thickness, lineType=cv2.LINE_AA)

    cv2.imshow("Webcam Feed", cv2.resize(cv_image, (640, 480)))

    # Wait for a key event for 1 millisecond
    if cv2.waitKey(20) & 0xFF == ord('q'):
        rospy.signal_shutdown("User pressed 'q' key")


def drone_height_callback(msg):
    global camera_height
    detect_range = msg.range
    camera_height = detect_range


def main():
    global camera_height
    # Initialize the ROS node
    rospy.init_node('webcam_subscriber', anonymous=True)

    # Create a subscriber to the webcam image topic
    rospy.Subscriber('webcam/cam_and_height', CamHeight,
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
