#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import os
import numpy as np
import rospy
from flying_turtle.msg import *
import rospy
from std_msgs.msg import Bool, String, Float32, Header, Int32MultiArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import cv2
import cv2.aruco as aruco
import math
import numpy as np
from flying_turtle.msg import ArucoMarker, ArucoMarkers, Point
import yaml


def estimate_pose_single_markers_angle(corners, marker_length, camera_matrix, dist_coeffs):
    # Define the 3D coordinates of the marker corners in its own coordinate system
    # Assuming the marker is centered at the origin
    half_marker_length = marker_length / 2.0
    obj_points = np.array([
        [-half_marker_length,  half_marker_length, 0],
        [half_marker_length,  half_marker_length, 0],
        [half_marker_length, -half_marker_length, 0],
        [-half_marker_length, -half_marker_length, 0]
    ], dtype=np.float32)

    # Initialize lists to hold rotation and translation vectors
    rvecs = []
    tvecs = []

    # Loop through each detected marker
    for corner in corners:
        # Solve PnP to get rotation and translation vectors
        ret, rvec, tvec = cv2.solvePnP(
            obj_points, corner, camera_matrix, dist_coeffs)
        if ret:
            rvecs.append(rvec)
            tvecs.append(tvec)

    rmat = cv2.Rodrigues(rvecs[0])[0]

    # Extract the rotation angle in radians around the z-axis
    theta = np.arctan2(rmat[1, 0], rmat[0, 0])

    # Convert the angle to degrees
    angle_deg = np.degrees(theta)

    return angle_deg


def rotation_matrix_x(angle_degrees):
    # Convert angle from degrees to radians
    theta = np.radians(angle_degrees)

    # Define the rotation matrix for rotation around X-axis
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(theta), -np.sin(theta)],
                    [0, np.sin(theta), np.cos(theta)]])

    return R_x


def calculate_angle(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    theta_degrees = math.degrees(math.atan((y1-y2)/(x1-x2)))

    return theta_degrees


def project_to_plane(p_x, p_y, K, R, T, plane_normal=[0, 0, 1], plane_d=0):

    # Camera intrinsics (K)
    f_x = K[0, 0]
    f_y = K[1, 1]
    c_x = K[0, 2]
    c_y = K[1, 2]

    # Step 1: Convert pixel to normalized camera coordinates
    x_norm = (p_x - c_x) / f_x
    y_norm = (p_y - c_y) / f_y

    # Ray direction in camera coordinates
    d_camera = np.array([x_norm, y_norm, 1.0])

    # Step 2: Transform ray direction to world coordinates
    d_world = np.dot(R, d_camera)

    # Camera position in world coordinates
    C = T

    # Step 3: Solve for t (intersection with the plane)
    plane_normal = np.array(plane_normal)
    numerator = - (np.dot(plane_normal, C) + plane_d)
    denominator = np.dot(plane_normal, d_world)

    if np.abs(denominator) < 1e-6:
        raise ValueError(
            "Ray is parallel to the plane and does not intersect.")

    t = numerator / denominator

    # Step 4: Compute the intersection point
    P = C + t * d_world

    return P


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


def main():
    SAVE_DATA_NAME = 'data.yaml'
    # Path to the folder containing the images
    rospy.init_node("aruco_reader", anonymous=True)

    image_folder = "rosbag_pic"  # Replace with your folder path

    camera_matrix = np.array([[1306, 0, 960],
                              [0, 1306, 540],
                              [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1))  # Assuming no lens distortion
    # List available cameras
    camera_angle_x = rospy.get_param('/flying_turtle/camera_angle_x', 30)
    camera_angle_y = rospy.get_param('/flying_turtle/camera_angle_y', 0)
    turtlebot_aruco_id = rospy.get_param(
        '/flying_turtle/turtlebot_aruco_id', 0)

    resolution = (1920, 1080)

    # Load the ArUco dictionary and parameters
    parameters = aruco.DetectorParameters()
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 70
    parameters.adaptiveThreshWinSizeStep = 5
    parameters.minMarkerPerimeterRate = 0.02
    parameters.maxMarkerPerimeterRate = 4.0
    parameters.polygonalApproxAccuracyRate = 0.03
    parameters.minCornerDistanceRate = 0.05
    parameters.minDistanceToBorder = 0
    parameters.minMarkerDistanceRate = 0.05

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_detector = aruco.ArucoDetector(aruco_dict, parameters)

    # Get a sorted list of image files in the folder
    images = sorted([img for img in os.listdir(image_folder)
                    if img.endswith(".png") or img.endswith(".jpg")])

    # Create a window to display the images
    cv2.namedWindow("ArUco Detection")

    det_marker = dict()
    data_list = []

    for image_name in images:
        # Read the image
        image_path = os.path.join(image_folder, image_name)

        camera_height = float(image_name.strip('.png').split("_")[1])

        frame = cv2.imread(image_path)

        frame = cv2.rotate(frame, cv2.ROTATE_180)

        if frame is None:
            print("Failed to load image:", image_name)
            continue

        # Convert the image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        kernel = np.array([[0, -1, 0],
                           [-1, 5, -1],
                           [0, -1, 0]])
        gray = cv2.filter2D(gray, -1, kernel)

        aruco_list = ArucoMarkers()
        aruco_list.header = Header()
        aruco_list.header.stamp = rospy.Time.now()
        aruco_list.camera_height = camera_height

        # Detect ArUco markers in the image
        corners, ids, rejectedImgPoints = aruco_detector.detectMarkers(
            gray)
        # print(rejectedImgPoints)

        # Draw the detected markers on the image
        if ids is not None:
            # print(f"Detected markers in {image_name}: {ids.flatten()}")

            frame = aruco.drawDetectedMarkers(frame, corners, ids)

            detect_turtle = None

            for id, id_corners in zip(ids.flatten(), corners):
                # if id == 0:
                #     continue
                detected_aruco = ArucoMarker(id=int(id))
                coor_x = []
                coor_y = []
                for coord_corner in id_corners[0]:
                    # coord_center = []
                    point = Point(
                        x=int(coord_corner[0]), y=int(coord_corner[1]))
                    detected_aruco.corners.append(point)
                    coor_x.append(coord_corner[0])
                    coor_y.append(coord_corner[1])
                # coord_center.append(np.mean(coor_x))
                # coord_center.append(np.mean(coor_y))
                cpoint = Point(
                    x=int(np.mean(coor_x)), y=int(np.mean(coor_y)))
                detected_aruco.center = cpoint

                # R = np.eye(3)  # Identity rotation matrix (no rotation)
                R = rotation_matrix_x(camera_angle_x)
                # Camera position in world coordinates
                T = np.array([0, 0, camera_height])

                # Plane is horizontal (normal along Z-axis)
                plane_normal = [0, 0, 1]
                plane_d = 0
                if id == turtlebot_aruco_id:
                    T = np.array([0, 0, camera_height - 0.16])
                else:
                    T = np.array([0, 0, camera_height])

                world_point = project_to_plane(
                    (resolution[0] - cpoint.x), cpoint.y, K=camera_matrix, R=R, T=T, plane_normal=plane_normal, plane_d=plane_d)

                # if id == turtlebot_aruco_id:
                #     corner1_coord = id_corners[0][0]
                #     corner2_coord = id_corners[0][1]
                #     corner1_real = project_to_plane(
                #         (resolution[0] - corner1_coord[0]), corner1_coord[1], K=camera_matrix, R=R, T=T, plane_normal=plane_normal, plane_d=plane_d)
                #     corner2_real = project_to_plane(
                #         (resolution[0] - corner2_coord[0]), corner2_coord[1], K=camera_matrix, R=R, T=T, plane_normal=plane_normal, plane_d=plane_d)
                #     real_angle_deg = calculate_angle(
                #         corner1_real[:-1], corner2_real[:-1])
                #     print(angle_deg, real_angle_deg)
                # print("Projected world coordinates:", world_point)
                detected_aruco.real_coord = Point(
                    x=world_point[0], y=world_point[1])

                if id == turtlebot_aruco_id:
                    angle_deg = estimate_pose_single_markers_angle(
                        id_corners, 0.05, camera_matrix, dist_coeffs)

                    detected_aruco.z_rotation = angle_deg

                    # Display the rotation angle on the frame
                    cv2.putText(frame, f'Z-Angle: {angle_deg:.2f}',
                                (int(id_corners[0][0][0]), int(
                                    id_corners[0][0][1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                    detect_turtle = detected_aruco

                # plot_route(path_sequence=path_sequence, frame=frame)
                aruco_list.marker_list.append(detected_aruco)

            # print(aruco_list.marker_list)

            # goal_pub.publish(aruco_list)

            if detect_turtle:
                for marker in aruco_list.marker_list:
                    if marker.id == turtlebot_aruco_id:
                        continue
                    # print(f"ID: {m.id} | {m.real_coord}")

                    # if det_marker.get(m.id):
                    #     det_marker[m.id] += 1
                    # else:
                    #     det_marker[m.id] = 1

                    # print(det_marker)

                    turtlebot_origin = (0, 0)

                    id = marker.id
                    start_point = marker.real_coord
                    map_x = start_point.x - detect_turtle.real_coord.x + \
                        turtlebot_origin[0]
                    map_y = start_point.y - \
                        detect_turtle.real_coord.y + turtlebot_origin[1]

                    rot_map_x, rot_map_y = rotate_point(
                        (map_x, map_y), turtlebot_origin, detect_turtle.z_rotation)

                    data_dict = {'id': int(id),
                                 'pixel_x': float(marker.center.x),
                                 'pixel_y': float(marker.center.y),
                                 'real_coord_x': float(marker.real_coord.x),
                                 'real_coord_y': float(marker.real_coord.y),
                                 'map_coord_x': float(map_x),
                                 'map_coord_y': float(map_y),
                                 'final_coord_x': float(rot_map_x),
                                 'final_coord_y': float(rot_map_y),
                                 'z_rotation': float(marker.z_rotation),
                                 'height': float(camera_height)}
                    print(data_dict)

                    data_list.append(data_dict)

        # frame = aruco.drawDetectedMarkers(frame, rejectedImgPoints)
        # Display the image
        # cv2.imshow("ArUco Detection", frame)

        # # Wait for key press
        # key = cv2.waitKey(1) & 0xFF
        # if key == ord('q'):  # Press 'q' to quit
        #     cv2.destroyAllWindows()
        #     return

    # Close the window after processing all images
    cv2.destroyAllWindows()

    with open(SAVE_DATA_NAME, "w") as file:
        # Write the dictionary to the file in YAML format
        yaml.dump(data_list, file)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
