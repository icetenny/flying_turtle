#!/usr/bin/env python3

import rospy
import cv2
import os
from std_msgs.msg import Float32
from datetime import datetime
from sensor_msgs.msg import Range


class ImageSaver:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_saver_with_height', anonymous=True)
        resolution = (1920, 1080)  # Resolution of the camera

        chosen_camera = rospy.get_param('/flying_turtle/camera_index', 0)

        # Open the camera
        self.cap = cv2.VideoCapture(chosen_camera)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        resolution = (self.cap.get(3), self.cap.get(4))
        print("resolution:", resolution)

        # Check if the camera opened successfully
        if not self.cap.isOpened():
            rospy.logerr("Error: Could not open camera.")
            return

        # Subscribe to the height topic
        self.current_height = 0.0
        self.CAM_HEIGHT_CONST = 0
        self.height_file = open("heights.txt", "w")

        # Subscribe to the height topic
        rospy.Subscriber(
            '/mavros/px4flow/ground_distance', Range, self.drone_height_callback)

        # Create an output directory for images if it doesn't exist
        self.image_dir = "captured_images"
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)

    def save_images(self):
        rate = rospy.Rate(1)  # Set the rate to 1 Hz (1 image per second)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Error: Failed to capture image from camera.")
                break

            # Create a unique filename based on the current timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            image_filename = f"{self.image_dir}/image_{timestamp}.png"

            # Save the image
            cv2.imwrite(image_filename, frame)

            # Save the current height value with the image filename
            self.height_file.write(
                f"{image_filename}: {self.current_height}\n")

            # Display the frame (optional)
            cv2.imshow('Camera', frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rate.sleep()

    def drone_height_callback(self, msg: Range):
        detect_range = msg.range
        self.current_height = detect_range - self.CAM_HEIGHT_CONST

    def shutdown(self):
        # Release resources when the node is shutdown
        self.cap.release()
        self.height_file.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    image_saver = ImageSaver()
    if image_saver.cap.isOpened():
        rospy.on_shutdown(image_saver.shutdown)
        image_saver.save_images()
