#!/usr/bin/env python3

import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Int32, Bool, String, Float32, Header, Int32MultiArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import cv2
import cv2.aruco as aruco
import math
import numpy as np
from flying_turtle.msg import ArucoMarker, ArucoMarkers, Point

import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Range
from flying_turtle.msg import *
from std_msgs.msg import Int32

import rospy
import math
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    # print(f"Objective: {solution.ObjectiveValue()}")
    max_route_distance = 0
    route = []
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route : "
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += f" {manager.IndexToNode(index)} -> "
            route.append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        route.append(manager.IndexToNode(index))
        plan_output += f"{manager.IndexToNode(index)}"
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    # print(f"Maximum of the route distances: {max_route_distance}m")
    return route


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


def goal_callback(msg: ArucoMarkers):
    global coord_dict
    for g in msg.marker_list:
        id = int(g.id)
        x, y = g.real_coord.x, g.real_coord.y

        if coord_dict.get(id):
            coord_dict[id][0] += x
            coord_dict[id][1] += y
            coord_dict[id][2] += 1

        else:
            coord_dict[id] = [x, y, 1]


def get_distance(coord1: Point, coord2: Point):
    return int(math.sqrt((coord2.x - coord1.x) ** 2 + (coord2.y - coord1.y) ** 2))


def create_data_model(turtlebot_aruco_id: int, marker_list: ArucoMarkers):
    # Sample location
    turtlebot_index = -1
    index_list = []
    coor = []

    distance_matrix = []

    from_marker: ArucoMarker
    to_marker: ArucoMarker

    for i, from_marker in enumerate(marker_list.marker_list):
        row = []
        marker_id = from_marker.id
        if marker_id == turtlebot_aruco_id:
            turtlebot_index = i
        # if marker_id not in index_list:
        index_list.append(marker_id)
        coor.append([from_marker.real_coord.x, from_marker.real_coord.y])

        for to_marker in marker_list.marker_list:
            row.append(get_distance(
                from_marker.real_coord, to_marker.real_coord))
        distance_matrix.append(row)

    # Create data model
    data = {
        "num_vehicles": 1,
        "depot": turtlebot_index,
        "distance_matrix": distance_matrix,
        "coordinates": coor,
    }
    return data


def start_callback(msg):
    global coord_dict

    depot_point = rospy.get_param('/flying_turtle/turtlebot_aruco_id', 212)

    markers_to_do_path = ArucoMarkers()

    ttb_0 = ArucoMarker()
    ttb_0.id = depot_point
    markers_to_do_path.marker_list = [ttb_0]

    for c_id, c_v in coord_dict:
        if c_v[2] < 10:
            continue
        c_m = ArucoMarker()
        c_m.id = c_id

        c_m.real_coord.x = c_v[0] / c_v[2]
        c_m.real_coord.y = c_v[1] / c_v[2]
        markers_to_do_path.marker_list.append(c_m)

    if msg.data == 1:

        """Entry point of the program."""
        # Instantiate the data problem.

        data = create_data_model(depot_point, marker_list=markers_to_do_path)

        if data["depot"] == -1:
            return

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
        )

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index): return data["distance_matrix"][manager.IndexToNode(
            from_index)][manager.IndexToNode(to_index)]

        transit_callback_index = routing.RegisterTransitCallback(
            distance_callback)

        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Distance constraint.
        dimension_name = "Distance"
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            30000000000,  # vehicle maximum travel distance
            True,  # start cumul to zero
            dimension_name,
        )
        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(100)

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        solution = routing.SolveWithParameters(search_parameters)
        if solution:
            route = print_solution(data, manager, routing, solution)
            # plot_route(data, route)
            pub_route = ArucoMarkers()
            pub_route.header = Header()
            pub_route.header.stamp = rospy.Time.now()
            pub_route.camera_height = msg.camera_height

            for r in route:
                pub_route.marker_list.append(msg.marker_list[r])
            # goal_pub.publish(pub_route)

            current_x, current_y, current_yaw = get_current_pose()

            for r_goal in pub_route.marker_list[1:]:
                print(r_goal)
                goal_x, goal_y = calculate_goal_point(
                    r_goal.real_coord.x, r_goal.real_coord.y, current_x, current_y, current_yaw)
                result = movebase_client(goal_x, goal_y, 0, 1)
                if result:
                    rospy.loginfo("Goal execution done!")
                else:
                    rospy.loginfo("Goal execution failed!")

        else:
            print("No solution found !")


def main():
    global coord_dict

    coord_dict = dict()
    # Initialize the ROS node
    rospy.init_node('webcam_subscriber', anonymous=True)

    # Create a subscriber to the webcam image topic
    rospy.Subscriber('turtlebot_goal', ArucoMarkers,
                     goal_callback, queue_size=10)

    rospy.Subscriber('start', Int32,
                     start_callback, queue_size=10)

    rospy.Subscriber(
        '/flying_turtle/path_sequence', ArucoMarkers, queue_size=10)

    # Spin to keep the script alive and processing callbacks
    rospy.spin()

    # Close all OpenCV windows when done
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # Create a CvBridge object
    bridge = CvBridge()
    print('Start Cam Sub')

    try:
        main()
    except rospy.ROSInterruptException:
        pass
