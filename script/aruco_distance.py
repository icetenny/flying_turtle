#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Bool, String, Float32, Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import cv2
import cv2.aruco as aruco
import math
import numpy as np


def calculate_real_distance(px1, py1, px2, py2, w2h_ratio, resolution, known_height):
    # Unpack resolution
    pxW, pxH = resolution

    # Aspect ratio
    ar = pxW / pxH

    # Real-life width and height at the known distance
    real_width = known_height * w2h_ratio
    real_height = known_height * w2h_ratio / ar

    dW = (px2-px1) / pxW * real_width
    dH = (py2 - py1) / pxH * real_height

    # Calculate real-life distance between the two points
    real_distance = math.sqrt(dW**2 + dH **2)

    return dW, dH, real_distance


def main():
    rospy.init_node("aruco_detector", anonymous=True)

    resolution = (1024, 576)  # Resolution of the camera
    width2distance_ratio = 91 / 67  # Ratio of width to distance of cam

    # Get ArUco IDs from ROS parameters
    known_distance = rospy.get_param('/drone_height', 1.5)  # Known distance from camera to the plane of the points
    turtlebot_aruco_id = rospy.get_param('/turtlebot_aruco_id', 971)
    target_aruco_id = rospy.get_param('/target_aruco_id', 212)

    # Initialize the detector parameters using default values
    parameters = aruco.DetectorParameters()
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshWinSizeStep = 10
    parameters.minMarkerPerimeterRate = 0.03
    parameters.maxMarkerPerimeterRate = 4.0
    parameters.polygonalApproxAccuracyRate = 0.03
    parameters.minCornerDistanceRate = 0.05
    parameters.minDistanceToBorder = 3
    parameters.minMarkerDistanceRate = 0.05

    # Load the dictionary that was used to generate the markers.
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

    # List available cameras
    chosen_camera = rospy.get_param('/camera_index', 0)
    # Start the video capture
    cap = cv2.VideoCapture(chosen_camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

    # rospy.Subscriber(
    #         '/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    
    # rospy.Subscriber(
    #         '/initialpose', PoseWithCovarianceStamped, initial_callback)
    
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)


    rate = rospy.Rate(10) # 10hz


    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            break

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect the markers in the image
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Draw the detected markers on the frame
        if ids is not None:
            corners_dict = dict(zip(ids.flatten(), corners))
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

            if turtlebot_aruco_id in corners_dict and target_aruco_id in corners_dict:
                turtlebot_corner_coord = corners_dict[turtlebot_aruco_id][0][0]
                target_corner_coord = corners_dict[target_aruco_id][0][0]
                ttb2target_distance = calculate_real_distance(*turtlebot_corner_coord, *target_corner_coord, w2h_ratio=width2distance_ratio, resolution=resolution, known_height=known_distance)
                rospy.loginfo("Estimated distance: %s", ttb2target_distance)

                x, y, rz, rw = 0,0,0,0
                pub_goal = PoseStamped()
                pub_goal.header.stamp = rospy.Time.now()
                pub_goal.header.frame_id = "map"

                pub_goal.pose.position.x = x
                pub_goal.pose.position.y = y

                pub_goal.pose.orientation.z = rz
                pub_goal.pose.orientation.w = rw

                goal_pub.publish(pub_goal)

        # Display the resulting frame
        cv2.imshow('Webcam ArUco Detection', frame)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        rate.sleep()
    rospy.spin()

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
