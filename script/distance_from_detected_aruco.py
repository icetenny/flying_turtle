#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Bool, String, Float32, Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import cv2
import cv2.aruco as aruco
import math
import numpy as np
from flying_turtle.msg import *


# def calculate_real_distance(px1, py1, px2, py2, w2h_ratio, resolution, known_height):
#     # Unpack resolution
#     pxW, pxH = resolution

#     # Aspect ratio
#     ar = pxW / pxH

#     # Real-life width and height at the known distance
#     real_width = known_height * w2h_ratio
#     real_height = known_height * w2h_ratio / ar

#     dW = (px2-px1) / pxW * real_width
#     dH = (py2 - py1) / pxH * real_height

#     # Calculate real-life distance between the two points
#     real_distance = math.sqrt(dW**2 + dH ** 2)

#     return dW, dH, real_distance


def calculate_real_distance(marker1: ArucoMarker, marker2: ArucoMarker, w2h_ratio, resolution, known_height):
    print(marker1.corners)

    px1, py1 = marker1.corners[0].x, marker1.corners[0].y
    px2, py2 = marker2.corners[0].x, marker2.corners[0].y
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
    real_distance = math.sqrt(dW**2 + dH ** 2)

    return dW, dH, real_distance

def markers_callback(data: ArucoMarkers):
    print(data.marker_list)

def main():
    rospy.init_node("aruco_distance", anonymous=True)

    resolution = (1024, 576)  # Resolution of the camera
    width2distance_ratio = 91 / 67  # Ratio of width to distance of cam

    # Get ArUco IDs from ROS parameters
    # Known distance from camera to the plane of the points
    known_distance = rospy.get_param('/drone_height', 1.5)
    turtlebot_aruco_id = rospy.get_param('/turtlebot_aruco_id', 971)
    target_aruco_id = rospy.get_param('/target_aruco_id', 212)


    rospy.Subscriber(
            '/flying_turtle/detected_aruco', ArucoMarkers, markers_callback)

    # goal_pub = rospy.Publisher(
    #     '/move_base_simple/goal', PoseStamped, queue_size=10)

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        x, y, rz, rw = 0, 0, 0, 0
        pub_goal = PoseStamped()
        pub_goal.header.stamp = rospy.Time.now()
        pub_goal.header.frame_id = "map"

        pub_goal.pose.position.x = x
        pub_goal.pose.position.y = y

        pub_goal.pose.orientation.z = rz
        pub_goal.pose.orientation.w = rw

        # goal_pub.publish(pub_goal)
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
