#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Bool, String, Float32, Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import cv2
import cv2.aruco as aruco
import math
import numpy as np
from flying_turtle.msg import *


class FlyingTurtle():
    def __init__(self) -> None:
        rospy.init_node("main", anonymous=True)

        self.resolution = (1024, 576)  # Resolution of the camera
        self.width2distance_ratio = 91 / 67  # Ratio of width to distance of cam

        # Get ArUco IDs from ROS parameters
        # Known distance from camera to the plane of the points
        self.drone_height = 1.5
        self.turtlebot_aruco_id = rospy.get_param('/turtlebot_aruco_id', 971)

        self.target_markers_dict = dict()
        self.turtlebot_marker = ArucoMarker()


        rospy.Subscriber(
                '/flying_turtle/detected_aruco', ArucoMarkers, self.markers_callback)
        
        rospy.Subscriber(
                '/flying_turtle/drone_height', Float32, self.height_callback)

        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)
    
    def height_callback(self, data):
        self.drone_height = data.data


    def markers_callback(self, data: ArucoMarkers):
        new_marker_dict = dict()
        for marker in data.marker_list:
            if marker.id == self.turtlebot_aruco_id:
                self.turtlebot_marker = marker
            else:
                new_marker_dict[marker.id] = marker
        self.target_markers_dict = new_marker_dict
        # print(self.detected_markers_dict)

        target_marker = list(self.target_markers_dict.values())[0]

        dW, dH, real_distance = self.calculate_real_distance(self.turtlebot_marker, target_marker)
        print(dW, dH, real_distance)


        # Transform goal position to account for turtlebot's orientation
        x_goal = dW * np.cos(target_marker.z_rotation) - dH * np.sin(target_marker.z_rotation)
        y_goal = dW * np.sin(target_marker.z_rotation) + dH * np.cos(target_marker.z_rotation)

        # Publish the goal
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = x_goal
        goal.pose.position.y = y_goal
        goal.pose.orientation.z = np.sin(target_marker.z_rotation / 2)
        goal.pose.orientation.w = np.cos(target_marker.z_rotation / 2)

        self.goal_pub.publish(goal)
        print("Publishing Goal...........")

    def calculate_real_distance(self, marker1: ArucoMarker, marker2: ArucoMarker):

        px1, py1 = marker1.corners[0].x, marker1.corners[0].y
        px2, py2 = marker2.corners[0].x, marker2.corners[0].y
        # Unpack resolution
        pxW, pxH = self.resolution

        # Aspect ratio
        ar = pxW / pxH

        # Real-life width and height at the known distance
        real_width = self.drone_height * self.width2distance_ratio
        real_height = self.drone_height * self.width2distance_ratio/ ar

        dW = (px2-px1) / pxW * real_width
        dH = (py2 - py1) / pxH * real_height

        # Calculate real-life distance between the two points
        real_distance = math.sqrt(dW**2 + dH ** 2)

        return dW, dH, real_distance




def main():
    myturtle = FlyingTurtle()

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
