#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import os
import numpy as np
import rospy


def unsharp_mask(image, kernel_size=(5, 5), sigma=1.0, amount=1.5, threshold=0):
    # Blur the image
    blurred = cv2.GaussianBlur(image, kernel_size, sigma)

    # Calculate the sharpened image
    sharpened = float(amount + 1) * image - float(amount) * blurred

    # Clip the values to [0, 255] and convert back to uint8
    sharpened = np.maximum(sharpened, np.zeros(sharpened.shape))
    sharpened = np.minimum(sharpened, 255 * np.ones(sharpened.shape))
    sharpened = sharpened.round().astype(np.uint8)

    # Apply the threshold
    if threshold > 0:
        low_contrast_mask = np.absolute(image - blurred) < threshold
        np.copyto(sharpened, image, where=low_contrast_mask)

    return sharpened


def main():
    # Path to the folder containing the images
    image_folder = "rosbag_pic"  # Replace with your folder path

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

    for image_name in images:
        # Read the image
        image_path = os.path.join(image_folder, image_name)
        frame = cv2.imread(image_path)

        if frame is None:
            print("Failed to load image:", image_name)
            continue

        # Convert the image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        kernel = np.array([[0, -1, 0],
                           [-1, 5, -1],
                           [0, -1, 0]])
        gray = cv2.filter2D(gray, -1, kernel)

        # gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # gray = unsharp_mask(gray)

        # gray = cv2.adaptiveThreshold(
        #     gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 19, 5)

        # Detect ArUco markers in the image
        corners, ids, rejectedImgPoints = aruco_detector.detectMarkers(
            gray)
        # print(rejectedImgPoints)

        # Draw the detected markers on the image
        if ids is not None:
            print(f"Detected markers in {image_name}: {ids.flatten()}")

        frame = aruco.drawDetectedMarkers(frame, corners, ids)

        # frame = aruco.drawDetectedMarkers(frame, rejectedImgPoints)
        # Display the image
        cv2.imshow("ArUco Detection", frame)

        # Wait for key press
        while True:
            key = cv2.waitKey(0) & 0xFF
            if key == ord('s'):  # Press 's' to move to the next image
                break
            elif key == ord('q'):  # Press 'q' to quit
                cv2.destroyAllWindows()
                return

    # Close the window after processing all images
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
