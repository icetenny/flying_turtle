#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from flying_turtle.msg import CamHeight
from sensor_msgs.msg import Range

camera_height = 0


def drone_height_callback(msg: Range):
    global camera_height
    detect_range = msg.range
    camera_height = detect_range


def publish_webcam():
    global camera_height
    # Initialize the ROS node
    rospy.init_node('webcam_publisher', anonymous=True)

    # Create a publisher for the image topic
    # image_pub = rospy.Publisher(
    #     'webcam/compressed', CompressedImage, queue_size=1)

    image_pub = rospy.Publisher(
        'webcam/cam_and_height', CamHeight, queue_size=1)

    rospy.Subscriber(
        '/mavros/px4flow/ground_distance', Range, drone_height_callback)

    # Create a CvBridge object to convert OpenCV images to ROS Image messages
    bridge = CvBridge()

    # Open the webcam
    # '0' is usually the default webcam; change if necessary

    resolution = (1920, 1080)  # Resolution of the camera
    chosen_camera = rospy.get_param('/flying_turtle/camera_index', 0)

    cap = cv2.VideoCapture(chosen_camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    resolution = (cap.get(3), cap.get(4))
    print("resolution:", resolution)

    if not cap.isOpened():
        rospy.logerr("Could not open webcam")
        return

    # Loop to continuously publish the webcam feed
    rate = rospy.Rate(30)  # Publish rate in Hz, adjust as needed
    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if not ret:
            rospy.logerr("Failed to capture image from webcam")
            break

        try:
            # Convert the OpenCV image to a ROS Image message
            ros_image = bridge.cv2_to_compressed_imgmsg(frame)

            cam_and_height = CamHeight()
            cam_and_height.height = camera_height
            cam_and_height.image = ros_image

            # Publish the image
            image_pub.publish(cam_and_height)
            rospy.loginfo(
                "Published webcam image to topic 'webcam/cam_and_height")
        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: {}".format(e))

        # Sleep to maintain the loop rate
        rate.sleep()

    # Release the webcam
    cap.release()


if __name__ == '__main__':
    try:
        publish_webcam()
    except rospy.ROSInterruptException:
        pass
