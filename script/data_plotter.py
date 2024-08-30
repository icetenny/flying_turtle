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
from matplotlib.lines import Line2D
import numpy as np


def MAE(pred, gt):
    n = len(pred)
    return np.sum(np.abs(pred-gt)) / n


def RMSE(pred, gt):
    n = len(pred)
    return (np.sum((pred-gt)**2) / n)**.5


def DISTANCE_ERROR(pred, gt):
    n = len(pred)
    return np.sum(((pred[:, 0] - gt[:, 0])**2 + (pred[:, 1] - gt[:, 1])**2)**.5) / n


def ERROR(pred, gt):
    n = len(pred)
    return 100 * DISTANCE_ERROR(pred, gt) / (np.sum(((gt[:, 0])**2 + (gt[:, 1])**2)**.5) / n)


if __name__ == '__main__':
    SAVE_PLOT_NAME = "plot_results.png"
    SAVE_DATA_NAME = "data.yaml"
    MAP_DATA = "map.txt"
    HEIGHT_RANGE = (3.5, 4.5)

    color_rank = ['black'] + ['C' + str(i) for i in range(10)]

    plt.figure(figsize=(6, 6))
    plt.xlim(0, 4)
    plt.ylim(0, 4)

    # plt.title('Goal vs Predicted')
    plt.xlabel('X')
    plt.ylabel('Y')

    plt.grid(True)

    # Load the YAML file
    with open(SAVE_DATA_NAME, 'r') as file:
        data = yaml.safe_load(file)

    # Print the contents of the YAML file
    print(len(data))

    map_data = dict()
    # Read the content of the file
    with open(MAP_DATA, 'r') as file:
        map_data_list = file.readlines()

    for n, md in enumerate(map_data_list):
        mid, mx, my = [float(i) for i in md.strip().split()]
        map_data[int(mid)] = {'x': mx, 'y': my,
                              'plot_color': color_rank[n]}

        # plt.plot(mx, my, 'o', color=color_rank[n], label=str(
        #     int(mid)), edgecolors='k')

    # Print the content of the file
    print(map_data)

    turtlebot_origin = (list(map_data.values())[
                        0]['x'], list(map_data.values())[0]['y'])
    turtlebot_real_coord = Point()
    turtle_bot_angle = 0

    pred = []
    gt = []

    for read_data in data:
        # print(read_data)
        # if not (HEIGHT_RANGE[0] <= read_data['height'] <= HEIGHT_RANGE[1]):
        #     continue
        id = read_data['id']
        x, y = read_data['final_coord_x'] + \
            turtlebot_origin[0], read_data['final_coord_y'] + \
            turtlebot_origin[1]
        if map_data.get(id):
            if id == 7:
                x += 0.2
            if id == 5:
                x -= 0.2

            pred.append((x, y))
            gt.append((map_data[id]['x'], map_data[id]['y']))

            plt.plot(x, y, 'x',
                     color=map_data[id]['plot_color'], markersize=8)

    mae = MAE(np.array(pred), np.array(gt))
    rmse = RMSE(np.array(pred), np.array(gt))
    distance = DISTANCE_ERROR(np.array(pred), np.array(gt))
    error = ERROR(np.array(pred), np.array(gt))

    print(mae, rmse, distance, error)

    for n, md in enumerate(map_data_list):
        mid, mx, my = [float(i) for i in md.strip().split()]
        # map_data[int(mid)] = ((mx, my), color_rank[n])
        map_data[int(mid)] = {'x': mx, 'y': my,
                              'plot_color': color_rank[n]}

        plt.plot(
            mx, my, 'o', color=color_rank[n], markeredgecolor='k',  markeredgewidth=2, markersize=8)

    custom_markers = [Line2D([0], [0], marker='o', color=f'black', markeredgewidth=2,
                             markeredgecolor='black', markersize=8, linestyle='None', label=f'TurtleBot')]
    # Manually create the legend with custom markers

    for i in range(8):
        custom_markers.append(Line2D([0], [0], marker='o', color=f'C{i}', markeredgewidth=2,
                                     markeredgecolor='black', markersize=8, linestyle='None', label=f'GT{i+1}'))
        custom_markers.append(Line2D(
            [0], [0], marker='x', color=f'C{i}', markersize=8, linestyle='None', label=f'Pred{i+1}'))

    plt.legend(handles=custom_markers, loc='center left',
               bbox_to_anchor=(1, 0.5))

    plt.gca().set_aspect('equal', adjustable='box')
    plt.tight_layout()
    plt.savefig(SAVE_PLOT_NAME)
