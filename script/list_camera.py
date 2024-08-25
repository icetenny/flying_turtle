#!/usr/bin/env python3


import cv2
import cv2.aruco as aruco


def list_cameras():
    arr = []
    for index in range(10):
        cap = cv2.VideoCapture(index)
        if not cap.read()[0]:
            continue
        else:
            arr.append(index)
        cap.release()

    if not arr:
        print("No cameras found.")
        exit()
    else:
        print("Available cameras:")
        for i, cam in enumerate(arr):
            print(f"{i}: Camera {cam}")

        # # Let the user choose a camera
        # cam_index = int(input("Choose a camera index: "))

        # if cam_index < 0 or cam_index >= len(arr):
        #     print("Invalid camera index.")
        #     exit()
        # else:
        #     return arr[cam_index]
    # return arr


# List available cameras
list_cameras()
