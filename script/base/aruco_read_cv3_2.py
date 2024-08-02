import cv2
import cv2.aruco as aruco
import math
import numpy as np


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

        # Let the user choose a camera
        cam_index = int(input("Choose a camera index: "))

        if cam_index < 0 or cam_index >= len(arr):
            print("Invalid camera index.")
            exit()
        else:
            return arr[cam_index]
    # return arr


def calculate_real_distance(px1, py1, px2, py2, w2h_ratio, resolution, known_height):
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


resolution = (1024, 576)  # Resolution of the camera
known_distance = 1.5  # Known distance from camera to the plane of the points
width2distance_ratio = 91/67  # Ratio of width to distance of cam

# dW, dH, d = calculate_real_distance(px1, py1, px2, py2, width2height_ratio, resolution, known_distance)
turtlebot_aruco_id = 971
target_aruco_id = 212

# Initialize the detector parameters using default values
parameters = aruco.DetectorParameters_create()
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 23
parameters.adaptiveThreshWinSizeStep = 10
parameters.minMarkerPerimeterRate = 0.03
parameters.maxMarkerPerimeterRate = 4.0
parameters.polygonalApproxAccuracyRate = 0.03
parameters.minCornerDistanceRate = 0.05
parameters.minDistanceToBorder = 3
parameters.minMarkerDistanceRate = 0.05

# Load the dictionary that was used to generate the markers.
aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

# List available cameras
chosen_camera = list_cameras()
# Start the video capture
cap = cv2.VideoCapture(chosen_camera)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect the markers in the image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters)

    # Draw the detected markers on the frame
    if ids is not None:
        corners_dict = dict(zip(ids.flatten(), corners))
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

        if turtlebot_aruco_id in corners_dict and target_aruco_id in corners_dict:
            turtlebot_corner_coord = corners_dict[turtlebot_aruco_id][0][0]
            target_corner_coord = corners_dict[target_aruco_id][0][0]
            ttb2target_distance = calculate_real_distance(
                *turtlebot_corner_coord, *target_corner_coord, w2h_ratio=width2distance_ratio, resolution=resolution, known_height=known_distance)
            print("Estimated distance: ", ttb2target_distance)

    # Display the resulting frame
    cv2.imshow('Webcam ArUco Detection', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture and close the windows
cap.release()
cv2.destroyAllWindows()
