#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import time
from flying_turtle.msg import *
import rospkg
import rospy
import os
import math
import yaml


def rotate_point(point, pivot, angle_degrees):
    # Unpack the point and pivot coordinates
    x, y = point
    px, py = pivot

    # Convert angle to radians
    angle_radians = math.radians(angle_degrees)

    # Translate point to origin
    x_translated = x - px
    y_translated = y - py

    # Apply rotation
    x_rotated = x_translated * \
        math.cos(angle_radians) - y_translated * math.sin(angle_radians)
    y_rotated = x_translated * \
        math.sin(angle_radians) + y_translated * math.cos(angle_radians)

    # Translate the point back
    x_final = x_rotated + px
    y_final = y_rotated + py
    return x_final, y_final


def path_sequence_callback(msg: ArucoMarkers):
    global aruco_list, new_data_received, turtlebot_real_coord, turtle_bot_angle

    if msg.marker_list:
        turtlebot_real_coord = msg.marker_list[0].real_coord
        turtle_bot_angle = msg.marker_list[0].z_rotation

    aruco_list = msg
    new_data_received = True


def timer_callback(event):
    global new_data_received, aruco_list, turtlebot_real_coord, map_data, turtle_bot_angle, SAVE_PLOT_NAME, SAVE_DATA_NAME
    data_list = []

    marker_list = aruco_list.marker_list
    camera_height = aruco_list.camera_height
    if new_data_received and marker_list:
        x_list = []
        y_list = []
        # Clear the previous plot and update it with new points
        for i in range(len(marker_list) - 1):
            marker: ArucoMarker
            marker = marker_list[i]
            id = marker.id
            start_point = marker.real_coord
            map_x = start_point.x - turtlebot_real_coord.x + \
                turtlebot_origin[0]
            map_y = start_point.y - \
                turtlebot_real_coord.y + turtlebot_origin[1]

            rot_map_x, rot_map_y = rotate_point(
                (map_x, map_y), turtlebot_origin, turtle_bot_angle)

            if map_data.get(id):
                plt.plot(rot_map_x, rot_map_y, 'x',
                         color=map_data[id]['plot_color'])

                x_list.append(rot_map_x)
                y_list.append(rot_map_y)
                print(map_x, map_y, rot_map_x, rot_map_y)
                # end_point = marker_list[i+1].real_coord

                data_dict = {'id': id,
                             'pixel_x': marker.center.x,
                             'pixel_y': marker.center.y,
                             'real_coord_x': marker.real_coord.x,
                             'real_coord_y': marker.real_coord.y,
                             'map_coord_x': map_x,
                             'map_coord_y': map_y,
                             'final_coord_x': rot_map_x,
                             'final_coord_y': rot_map_y,
                             'z_rotation': marker.z_rotation}

                data_list.append(data_dict)

        # plt.plot(x_list, y_list, 'x')

        new_data_received = False

        # Save the current plot as an image
        plt.savefig(SAVE_PLOT_NAME)

        with open(SAVE_DATA_NAME, "a") as file:
            # Write the dictionary to the file in YAML format
            d_file = {"map_data": map_data,
                      "camera_height": camera_height, "results": data_list}
            yaml.dump([d_file], file)


def listener():
    # Initialize the ROS node
    rospy.init_node('path_plotter', anonymous=True)

    # Subscribe to the topic that publishes the path sequence
    rospy.Subscriber(
        '/flying_turtle/path_sequence', ArucoMarkers, path_sequence_callback)

    # Create a ROS timer to call the timer_callback every 3 seconds
    rospy.Timer(rospy.Duration(2), timer_callback)

    # Keep the node running
    rospy.spin()


if __name__ == '__main__':
    SAVE_PLOT_NAME = "plot_results.png"
    SAVE_DATA_NAME = "results.yaml"
    MAP_DATA = "map.txt"

    color_rank = ['black'] + ['C' + str(i) for i in range(10)]
    # Initialize ROS package manager
    map_name = rospy.get_param('/flying_turtle/map_name', 'map1.txt')

    # rospkg = rospkg.RosPack()
    # # Get the path to the 'flying_turtle' package
    # package_path = rospkg.get_path('flying_turtle')

    # # Define the relative path to the map1.txt file
    # map_file_path = os.path.join(package_path, 'map', map_name)

    # Initialize an empty list to store the path points
    aruco_list = ArucoMarkers()

    # Flag to indicate if new data has been received
    new_data_received = False

    map_config = dict()

    plt.figure(figsize=(8, 8))
    plt.xlim(0, 4)
    plt.ylim(0, 4)

    plt.title('Goal vs Predicted')
    plt.xlabel('X')
    plt.ylabel('Y')

    plt.grid(True)

    map_data = dict()
    # Read the content of the file
    with open(MAP_DATA, 'r') as file:
        for n, md in enumerate(file.readlines()):
            mid, mx, my = [float(i) for i in md.strip().split()]
            # map_data[int(mid)] = ((mx, my), color_rank[n])
            map_data[int(mid)] = {'x': mx, 'y': my,
                                  'plot_color': color_rank[n]}

            plt.plot(mx, my, 'o', color=color_rank[n], label=str(int(mid)))

        plt.legend()
        plt.savefig(SAVE_PLOT_NAME)

    # Print the content of the file
    print(map_data)

    turtlebot_origin = (list(map_data.values())[
                        0]['x'], list(map_data.values())[0]['y'])
    turtlebot_real_coord = Point()
    turtle_bot_angle = 0

    with open(SAVE_DATA_NAME, "w") as file:
        file.close()

    # x_gt = [p[0][0] for p in map_data.values()]
    # y_gt = [p[0][1] for p in map_data.values()]

    # plt.plot(x_gt, y_gt, 'o', color='black')

    # Start the ROS subscriber and timer
    listener()
