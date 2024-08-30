#!/usr/bin/env python3

import rospy
import rosbag
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import os
import sys


def extract_images_from_bag(bag_file, output_dir, topic_name):
    # Create the output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Initialize the CvBridge
    bridge = CvBridge()
    id = 0

    # Open the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            # Convert the compressed image message to an OpenCV image
            cv_image = bridge.compressed_imgmsg_to_cv2(
                msg.image, desired_encoding="bgr8")

            # Create a filename based on the timestamp
            timestamp = "%.6f" % msg.image.header.stamp.to_sec()
            filename = os.path.join(
                output_dir, f"frame{str(id).zfill(4)}_{msg.height}.png")

            # Save the image
            cv2.imwrite(filename, cv_image)
            rospy.loginfo("Saved image %s" % filename)
            id += 1


if __name__ == "__main__":
    rospy.init_node('extract_images', anonymous=True)

    if len(sys.argv) >= 2:
        bag_file = sys.argv[1]
    else:
        bag_file = "webcam_h.bag"  # Replace with your bag file

    output_dir = "rosbag_pic"  # Replace with your desired output directory
    topic_name = "/webcam/cam_and_height"  # Replace with your image topic

    extract_images_from_bag(bag_file, output_dir, topic_name)
