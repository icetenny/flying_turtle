#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Bool, String, Float32, Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import cv2
import cv2.aruco as aruco
import math
import numpy as np
from flying_turtle.msg import ArucoMarker, ArucoMarkers, Point

def estimate_pose_single_markers_angle(corners, marker_length, camera_matrix, dist_coeffs):
    # Define the 3D coordinates of the marker corners in its own coordinate system
    # Assuming the marker is centered at the origin
    half_marker_length = marker_length / 2.0
    obj_points = np.array([
        [-half_marker_length,  half_marker_length, 0],
        [ half_marker_length,  half_marker_length, 0],
        [ half_marker_length, -half_marker_length, 0],
        [-half_marker_length, -half_marker_length, 0]
    ], dtype=np.float32)

    # Initialize lists to hold rotation and translation vectors
    rvecs = []
    tvecs = []

    # Loop through each detected marker
    for corner in corners:
        # Solve PnP to get rotation and translation vectors
        ret, rvec, tvec = cv2.solvePnP(obj_points, corner, camera_matrix, dist_coeffs)
        if ret:
            rvecs.append(rvec)
            tvecs.append(tvec)

    rmat = cv2.Rodrigues(rvecs[0])[0]

    # Extract the rotation angle in radians around the z-axis
    theta = np.arctan2(rmat[1, 0], rmat[0, 0])

    # Convert the angle to degrees
    angle_deg = np.degrees(theta)

    return angle_deg

def main():
    rospy.init_node("aruco_reader", anonymous=True)

    resolution = (1024, 576)  # Resolution of the camera

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
    camera_matrix = np.array([[1000, 0, 320],
                            [0, 1000, 240],
                            [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1))  # Assuming no lens distortion

    # List available cameras
    chosen_camera = rospy.get_param('/camera_index', 0)
    # Start the video capture
    cap = cv2.VideoCapture(chosen_camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    
    goal_pub = rospy.Publisher('/flying_turtle/detected_aruco', ArucoMarkers, queue_size=10)


    rate = rospy.Rate(10) # 10hz


    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = cap.read()

        aruco_list = ArucoMarkers()
        aruco_list.header = Header()
        aruco_list.header.stamp = rospy.Time.now()

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
                detected_aruco = ArucoMarker(id=id)
                for coord_corner in id_corners[0]:
                    point = Point(x=coord_corner[0], y=coord_corner[1])
                    detected_aruco.corners.append(point)

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

                angle_deg = estimate_pose_single_markers_angle(id_corners, 0.05, camera_matrix, dist_coeffs)

                detected_aruco.z_rotation = angle_deg

                # Display the rotation angle on the frame
                cv2.putText(frame, f'Z-Angle: {angle_deg:.2f}', 
                            (int(id_corners[0][0][0]), int(id_corners[0][0][1]) - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                
                aruco_list.marker_list.append(detected_aruco)

            print(aruco_list.marker_list)

            goal_pub.publish(aruco_list)

        # Display the resulting frame
        cv2.imshow('Webcam ArUco Reader', frame)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        rate.sleep()
    rospy.spin()

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
