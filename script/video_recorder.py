#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Range


class VideoSaver:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('video_saver_with_height', anonymous=True)

        # Video settings
        self.video_filename = "output_video.avi"
        self.frame_rate = 1  # Frames per second
        self.codec = cv2.VideoWriter_fourcc(*'XVID')

        # Open the camera
        self.cap = cv2.VideoCapture(0)  # '0' is usually the default camera

        # Check if the camera opened successfully
        if not self.cap.isOpened():
            rospy.logerr("Error: Could not open camera.")
            return

        # Get the default frame width and height
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Initialize VideoWriter
        self.video_writer = cv2.VideoWriter(self.video_filename, self.codec, self.frame_rate,
                                            (self.frame_width, self.frame_height))

        # Store the latest height value
        self.current_height = 0.0
        self.CAM_HEIGHT_CONST = 0
        self.height_file = open("heights.txt", "w")

        # Subscribe to the height topic
        rospy.Subscriber(
            '/mavros/px4flow/ground_distance', Range, self.drone_height_callback)

    def read_camera_and_save(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Error: Failed to capture image from camera.")
                break

            # Write the frame to the video file
            self.video_writer.write(frame)

            # Save the current height value
            self.height_file.write(f"{self.current_height}\n")

            # Display the frame (optional)
            cv2.imshow('Camera', frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def drone_height_callback(self, msg: Range):
        detect_range = msg.range
        self.current_height = detect_range - self.CAM_HEIGHT_CONST

    def shutdown(self):
        # Release resources when the node is shutdown
        self.cap.release()
        self.video_writer.release()
        self.height_file.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    video_saver = VideoSaver()
    if video_saver.cap.isOpened():
        rospy.on_shutdown(video_saver.shutdown)
        video_saver.read_camera_and_save()
