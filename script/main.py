#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Bool, String, Float32, Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import cv2
import cv2.aruco as aruco
import math
import numpy as np
from flying_turtle.msg import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class FlyingTurtle():
    def __init__(self) -> None:
        rospy.init_node("main", anonymous=True)

        self.resolution = (1024, 576)  # Resolution of the camera
        self.width2distance_ratio = 91 / 67  # Ratio of width to distance of cam

        # Get ArUco IDs from ROS parameters
        # Known distance from camera to the plane of the points
        self.drone_height = 1.5
        self.turtlebot_aruco_id = rospy.get_param('/turtlebot_aruco_id', 971)

        self.goal_sequence = []
        self.turtlebot_marker = ArucoMarker()

        rospy.Subscriber(
            '/flying_turtle/path_sequence', ArucoMarkers, self.path_sequence_callback)

        rospy.Subscriber(
            '/flying_turtle/drone_height', Float32, self.height_callback)

        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)

    def height_callback(self, data):
        self.drone_height = data.data

    def path_sequence_callback(self, data: ArucoMarkers):
        marker_list = data.marker_list

        if len(marker_list) <= 2:
            print("Invalid Goal")
            return

        turtlebot_marker = marker_list[0]

        if turtlebot_marker.id != self.turtlebot_aruco_id:
            print("What?")
            return

        for goal_marker in turtlebot_marker[1:-1]:

            dW, dH, real_distance = self.calculate_real_distance(
                self.turtlebot_marker, goal_marker)
            print(dW, dH, real_distance)

            # Transform goal position to account for turtlebot's orientation
            x_goal = dW * np.cos(self.turtlebot_marker.z_rotation) - \
                dH * np.sin(self.turtlebot_marker.z_rotation)
            y_goal = dW * np.sin(self.turtlebot_marker.z_rotation) + \
                dH * np.cos(self.turtlebot_marker.z_rotation)

            # Publish the goal
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = x_goal
            goal.pose.position.y = y_goal
            goal.pose.orientation.z = 1
            goal.pose.orientation.w = 1
            # goal.pose.orientation.z = np.sin(goal_marker.z_rotation / 2)
            # goal.pose.orientation.w = np.cos(goal_marker.z_rotation / 2)

            # self.goal_pub.publish(goal)
            self.goal_sequence.append(goal)
            # print("Publishing Goal...........")

    def calculate_real_distance(self, marker1: ArucoMarker, marker2: ArucoMarker):

        px1, py1 = marker1.corners[0].x, marker1.corners[0].y
        px2, py2 = marker2.corners[0].x, marker2.corners[0].y
        # Unpack resolution
        pxW, pxH = self.resolution

        # Aspect ratio
        ar = pxW / pxH

        # Real-life width and height at the known distance
        real_width = self.drone_height * self.width2distance_ratio
        real_height = self.drone_height * self.width2distance_ratio / ar

        dW = (px2-px1) / pxW * real_width
        dH = (py2 - py1) / pxH * real_height

        # Calculate real-life distance between the two points
        real_distance = math.sqrt(dW**2 + dH ** 2)

        return dW, dH, real_distance

    def move_to_goal(x, y, w):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = w

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()


def main():
    myturtle = FlyingTurtle()

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():

        if myturtle.goal_sequence:
            for goal in myturtle.goal_sequence:
                result = myturtle.move_to_goal(
                    goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.w)
                if result:
                    rospy.loginfo("Goal execution done!")
                else:
                    rospy.loginfo("Failed to reach goal")

        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
