#!/usr/bin/env python3
import rospy
import math
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion


def get_current_pose():
    data = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation

    # Convert quaternion to Euler angles
    roll, pitch, yaw = euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])

    return position.x, position.y, yaw


def calculate_goal_point(x_relative, y_relative, current_x, current_y, current_yaw):
    goal_x = current_x + (x_relative * math.cos(current_yaw) -
                          y_relative * math.sin(current_yaw))
    goal_y = current_y + (x_relative * math.sin(current_yaw) +
                          y_relative * math.cos(current_yaw))

    return goal_x, goal_y


def movebase_client(x, y, z, w):

    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait for the action server to come up
    client.wait_for_server()

    # Create a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()

    # Set the goal parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = Point(x, y, 0)
    goal.target_pose.pose.orientation = Quaternion(0, 0, z, w)

    # Send the goal to the action server
    client.send_goal(goal)

    # Wait for the server to finish performing the action
    wait = client.wait_for_result()

    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # Result of executing the action
        return client.get_result()


if __name__ == '__main__':
    try:
        # Initialize a ROS node
        rospy.init_node('send_goal_py')

        # Define a set of goal points (x, y, z, w) coordinates for TurtleBot
        relative_goals = [
            (-0.5, 0, 0.0, 1.0),  # Goal 1
            (0, 0.5, 0.0, 1.0),  # Goal 2
            # (3.0, 6.0, 0.0, 1.0)   # Goal 3
        ]

        # Send each goal to TurtleBot

        current_x, current_y, current_yaw = get_current_pose()

        # Calculate the goal point relative to the TurtleBot (e.g., 1 meter ahead)
        goal_x, goal_y = calculate_goal_point(
            0, 1, current_x, current_y, current_yaw)
        for r_goal in relative_goals:
            print(r_goal)
            goal_x, goal_y = calculate_goal_point(
                r_goal[0], r_goal[1], current_x, current_y, current_yaw)
            result = movebase_client(goal_x, goal_y, 0, 1)
            if result:
                rospy.loginfo("Goal execution done!")
            else:
                rospy.loginfo("Goal execution failed!")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
