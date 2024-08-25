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


def plot_route(data, route, frame):
    coordinates = data["coordinates"]

    # Plot points
    for point, coor in coordinates.items():
        cv2.circle(frame, (coor[0], coor[1]), 5, (255, 0, 0), -1)
        cv2.putText(frame, str(
            point), (coor[0], coor[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)

    # Plot path
    for i in range(len(route) - 1):
        start_point = coordinates[route[i]]
        end_point = coordinates[route[i + 1]]
        cv2.line(frame, (start_point[0], start_point[1]),
                 (end_point[0], end_point[1]), (0, 0, 255), 2)

    return frame


def markers_to_path_callback(msg: ArucoMarkers):
    global goal_pub
    """Entry point of the program."""
    # Instantiate the data problem.

    depot_point = rospy.get_param('/flying_turtle/turtlebot_aruco_id', 212)
    data = create_data_model(depot_point, marker_list=msg)

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

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

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
        for r in route:
            pub_route.marker_list.append(msg.marker_list[r])
        goal_pub.publish(pub_route)

    else:
        print("No solution found !")


def main():
    global goal_pub
    rospy.init_node("markers_to_path", anonymous=True)

    rospy.Subscriber(
        '/flying_turtle/detected_aruco', ArucoMarkers, markers_to_path_callback)

    goal_pub = rospy.Publisher(
        '/flying_turtle/path_sequence', ArucoMarkers, queue_size=10)

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
