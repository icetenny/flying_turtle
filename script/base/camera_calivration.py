import cv2
import numpy as np
import glob
import os
import cv2 as cv

# # Initialize the camera (0 is usually the default camera)
# cap = cv2.VideoCapture(2)

# # Set the resolution to 1080p (1920x1080)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# # Check if the camera is opened successfully
# if not cap.isOpened():
#     print("Error: Could not open camera.")
#     exit()

# # Capture a single frame

# while cap.isOpened():
#     ret, frame = cap.read()

#     # Check if the frame was captured successfully
#     if not ret:
#         print("Error: Could not read frame.")
#     else:
#         # Display the captured image
#         cv2.imshow('Captured Image', frame)

#         # Wait for a key press and close the window
#         if cv2.waitKey(1) == ord('s'):

#             # Save the captured image to a file
#             cv2.imwrite('captured_image.jpg', frame)
#             # Release the camera and close windows
#             cap.release()
#             cv2.destroyAllWindows()

# exit()


# Define the dimensions of the chessboard
chessboard_size = (9, 7)
# This is the actual size of a square on the chessboard, in your preferred units (e.g., cm, meters)
square_size = 2.0

# Prepare object points, e.g. (0,0,0), (1,0,0), (2,0,0) ...., (8,5,0)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0],
                       0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points from all the images
objpoints = []  # 3d point in real-world space
imgpoints = []  # 2d points in image plane

# Get list of calibration images
images = glob.glob(
    '/home/ice/catkin_ws/src/flying_turtle/script/base/captured_image.jpg')

# Iterate over the images and find chessboard corners
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    print(ret)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # Optional: draw and display the corners
        img = cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('Chessboard Corners', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Perform camera calibration to get the camera matrix (K) and distortion coefficients (D)
ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

# Print the results
print("Camera Matrix (K):\n", K)
print("Distortion Coefficients (D):\n", D)

undistorted_image = cv2.undistort(img, K, D)

# Save or display the undistorted image
cv2.imwrite('undistorted_image.jpg', undistorted_image)
cv2.imshow('Undistorted Image', undistorted_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
