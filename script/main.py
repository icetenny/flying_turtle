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
from actionlib_msgs.msg import GoalStatusArray, GoalID


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
        rospy.Subscriber('/move_base/status', GoalStatusArray,
                         self.move_base_status_callback)

        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)
        self.goal_reached = False

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

        for goal_marker in marker_list[1:-1]:

            dW, dH, real_distance = self.calculate_real_distance(
                self.turtlebot_marker, goal_marker)
            # print(dW, dH, real_distance)

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

        px1, py1 = marker1.center.x, marker1.center.y
        px2, py2 = marker2.center.x, marker2.center.y
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

    def move_to_goal(self, x, y, w):
        print("Moving to", x, y, w)
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

    def move_to_goal2(self, x, y, w):
        self.goal_reached = False
        pub_goal = PoseStamped()
        pub_goal.header.stamp = rospy.Time.now()
        pub_goal.header.frame_id = "map"

        pub_goal.pose.position.x = x
        pub_goal.pose.position.y = y

        # pub_goal.pose.orientation.z = rz
        pub_goal.pose.orientation.w = w

        rospy.loginfo(f"Sending goal point: ({x})")
        for _ in range(3):
            self.goal_pub.publish(pub_goal)
            self.goal_reached = False

        rospy.sleep(2)
        self.goal_reached = False

    # def is_stop(self):
    #     return self.cmd_vel.linear.x == 0 and self.cmd_vel.angular.z == 0

    # def at_goal(self, tolerance=0.4):
    #     if self.last_goal:
    #         pose1 = self.last_goal
    #         pose2 = self.amcl_pose

    #         dx = pose1[0] - pose2[0]
    #         dy = pose1[1] - pose2[1]
    #         distance = math.sqrt(dx**2 + dy**2)

    #         print(distance)

    #         if distance <= tolerance:
    #             return True

    #     return False

    def move_base_status_callback(self, msg):
        if msg.status_list:
            goal_status = msg.status_list[0].status
            if goal_status == 3:
                if not self.goal_reached:
                    print("GOAL REACHED")
                    self.goal_reached = True

            elif goal_status == 1:
                self.goal_reached = False


def main():
    myturtle = FlyingTurtle()

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():

        if myturtle.goal_sequence:
            for goal in myturtle.goal_sequence:
                myturtle.goal_reached = False
                myturtle.move_to_goal2(
                    goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.w)

                while not myturtle.goal_reached:
                    pass
                    # print("Goal not Reach")
                # if result:
                #     rospy.loginfo("Goal execution done!")
                # else:
                #     rospy.loginfo("Failed to reach goal")

        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
