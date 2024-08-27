#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Bool, String, Float32, Header, Int32MultiArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import cv2
import cv2.aruco as aruco
import math
import numpy as np
from flying_turtle.msg import ArucoMarker, ArucoMarkers, Point
from sensor_msgs.msg import Range


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


def calculate_real_distance_to_cam_center(pixel_x, pixel_frame, height, cam_angle, fov_axis):
    delta_x = (pixel_frame / 2) - pixel_x
    x = height * math.tan(cam_angle + (math.atan((delta_x/pixel_frame)
                          * math.tan(fov_axis)))) - (height * math.tan(cam_angle))
    return x


def calculate_angle(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    # # Calculate the dot product of the vectors
    # dot_product = x1 * x2 + y1 * y2

    # # Calculate the magnitudes of the vectors
    # magnitude_v1 = math.sqrt(x1**2 + y1**2)
    # magnitude_v2 = math.sqrt(x2**2 + y2**2)

    # # Calculate the cosine of the angle
    # cos_theta = dot_product / (magnitude_v1 * magnitude_v2)

    # # Ensure the value is within the range of -1 to 1 to avoid domain errors
    # cos_theta = max(-1, min(1, cos_theta))

    # # Calculate the angle in radians
    # theta_radians = math.acos(cos_theta)

    # # Convert the angle to degrees
    # theta_degrees = math.degrees(theta_radians)

    theta_degrees = math.degrees(math.atan((y1-y2)/(x1-x2)))

    return theta_degrees


def path_sequence_callback(data):
    global path_sequence
    path_sequence = data


def plot_route(path_sequence: ArucoMarkers, frame):
    marker_list = path_sequence.marker_list

    if marker_list:
        # Plot points
        # for point, coor in coordinates.items():
        #     cv2.circle(frame, (coor[0], coor[1]), 5, (255, 0, 0), -1)
        # for i, seq in enumerate(path_sequence[:-1]):
        #     coord = marker_list[seq][0]
        #     cv2.putText(frame, str(i), (coord.x, coord.y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)

        # Plot path
        for i in range(len(marker_list) - 1):
            start_point = marker_list[i].center
            end_point = marker_list[i+1].center
            cv2.putText(frame, str(i), (int(start_point.x), int(start_point.y)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.line(frame, (int(start_point.x), int(start_point.y)),
                     (int(end_point.x), int(end_point.y)), (0, 0, 255), 2)

        return frame


def rotation_matrix_x(angle_degrees):
    # Convert angle from degrees to radians
    theta = np.radians(angle_degrees)

    # Define the rotation matrix for rotation around X-axis
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(theta), -np.sin(theta)],
                    [0, np.sin(theta), np.cos(theta)]])

    return R_x


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


def drone_height_callback(msg: Range):
    global camera_height, CAM_HEIGHT_CONST
    detect_range = msg.range
    camera_height = detect_range - CAM_HEIGHT_CONST


def main():
    global path_sequence, camera_height, CAM_HEIGHT_CONST
    path_sequence = ArucoMarkers()
    camera_height = 0.87
    CAM_HEIGHT_CONST = 0.0

    rospy.init_node("aruco_reader", anonymous=True)

    # resolution = (1024, 576)  # Resolution of the camera
    resolution = (1920, 1080)  # Resolution of the camera

    # Initialize the detector parameters using default values
    parameters = aruco.DetectorParameters()
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
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    aruco_detector = aruco.ArucoDetector(aruco_dict, parameters)

    # Camera calibration parameters (Need Calibration**)
    # camera_matrix = np.array([[743.74, 0, 512],
    #                           [0, 736, 288],
    #                           [0, 0, 1]], dtype=np.float32)
    camera_matrix = np.array([[1361.52, 0, 960],
                              [0, 1361.52, 540],
                              [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1))  # Assuming no lens distortion
    # List available cameras
    chosen_camera = rospy.get_param('/flying_turtle/camera_index', 2)
    print("Chosen Camera: ", chosen_camera)
    camera_angle_x = rospy.get_param('/flying_turtle/camera_angle_x', 0)
    camera_angle_y = rospy.get_param('/flying_turtle/camera_angle_y', 0)
    turtlebot_aruco_id = rospy.get_param(
        '/flying_turtle/turtlebot_aruco_id', 212)

    # Start the video capture
    cap = cv2.VideoCapture(chosen_camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    resolution = (cap.get(3), cap.get(4))
    print("resolution:", resolution)

    # fov = (743.74, 736)  # Camera FoV
    # fov_angle = [math.atan(r/f) for (r, f) in zip(resolution, fov)]

    goal_pub = rospy.Publisher(
        '/flying_turtle/detected_aruco', ArucoMarkers, queue_size=10)

    rospy.Subscriber(
        '/flying_turtle/path_sequence', ArucoMarkers, path_sequence_callback)
    rospy.Subscriber(
        '/mavros/px4flow/ground_distance', Range, drone_height_callback)

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown() and cap.isOpened():
        # Capture frame-by-frame
        ret, frame = cap.read()

        aruco_list = ArucoMarkers()
        aruco_list.header = Header()
        aruco_list.header.stamp = rospy.Time.now()
        aruco_list.camera_height = camera_height

        if not ret:
            break

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect the markers in the image
        corners, ids, rejectedImgPoints = aruco_detector.detectMarkers(gray)

        # Draw the detected markers on the frame
        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

            for id, id_corners in zip(ids.flatten(), corners):
                if id == 0:
                    continue
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

                # # Estimate pose of each marker
                # rvec, tvec, _ = aruco.estimatePoseSingleMarkers(id_corners, 0.05, camera_matrix, dist_coeffs)
                # # Draw axis for the marker
                # # cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)
                # # Convert rotation vector to rotation matrix
                # rmat = cv2.Rodrigues(rvec[0])[0]

                # # Extract the rotation angle in radians around the z-axis
                # theta = np.arctan2(rmat[1, 0], rmat[0, 0])

                # # Convert the angle to degrees
                # angle_deg = np.degrees(theta)

                if id == turtlebot_aruco_id:
                    angle_deg = estimate_pose_single_markers_angle(
                        id_corners, 0.05, camera_matrix, dist_coeffs)

                    detected_aruco.z_rotation = angle_deg

                    # Display the rotation angle on the frame
                    cv2.putText(frame, f'Z-Angle: {angle_deg:.2f}',
                                (int(id_corners[0][0][0]), int(
                                    id_corners[0][0][1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                # Calculate Real World Coord
                # K = np.array([[320, 0, 320],  # Intrinsic matrix
                #             [0, 320, 160],
                #             [0, 0, 1]])

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

                if id == turtlebot_aruco_id:
                    corner1_coord = id_corners[0][0]
                    corner2_coord = id_corners[0][1]
                    corner1_real = project_to_plane(
                        (resolution[0] - corner1_coord[0]), corner1_coord[1], K=camera_matrix, R=R, T=T, plane_normal=plane_normal, plane_d=plane_d)
                    corner2_real = project_to_plane(
                        (resolution[0] - corner2_coord[0]), corner2_coord[1], K=camera_matrix, R=R, T=T, plane_normal=plane_normal, plane_d=plane_d)
                    real_angle_deg = calculate_angle(
                        corner1_real[:-1], corner2_real[:-1])
                    print(angle_deg, real_angle_deg)
                # print("Projected world coordinates:", world_point)

                # d_x = calculate_real_distance_to_cam_center(
                #     pixel_x=cpoint.x, pixel_frame=resolution[0], height=camera_height, cam_angle=camera_angle_x, fov_axis=fov_angle[0])
                # d_y = calculate_real_distance_to_cam_center(
                #     pixel_x=cpoint.y, pixel_frame=resolution[1], height=camera_height, cam_angle=camera_angle_y, fov_axis=fov_angle[1])
                detected_aruco.real_coord = Point(
                    x=world_point[0], y=world_point[1])

                plot_route(path_sequence=path_sequence, frame=frame)
                aruco_list.marker_list.append(detected_aruco)

            # print(aruco_list.marker_list)

            for m in aruco_list.marker_list:
                print(f"ID: {m.id} | {m.real_coord}")

            goal_pub.publish(aruco_list)

        # Display the resulting frame
        cv2.imshow('Webcam ArUco Reader', cv2.resize(frame, (960, 540)))

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            exit()
            # break
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
