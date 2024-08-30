import cv2
import cv2.aruco as aruco
import numpy as np

# Function to initialize ArUco parameters


def initialize_aruco_parameters():
    parameters = aruco.DetectorParameters()
    # parameters.adaptiveThreshWinSizeMin = 5
    # parameters.adaptiveThreshWinSizeMax = 50
    # parameters.adaptiveThreshWinSizeStep = 5
    # parameters.minMarkerPerimeterRate = 0.02
    # parameters.maxMarkerPerimeterRate = 4.5
    # parameters.polygonalApproxAccuracyRate = 0.02
    # parameters.minCornerDistanceRate = 0.02
    # parameters.minDistanceToBorder = 1
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 50
    parameters.adaptiveThreshWinSizeStep = 5
    parameters.minMarkerPerimeterRate = 0.03
    parameters.maxMarkerPerimeterRate = 4.0
    parameters.polygonalApproxAccuracyRate = 0.03
    parameters.minCornerDistanceRate = 0.05
    parameters.minDistanceToBorder = 0
    parameters.minMarkerDistanceRate = 0.05

    return parameters


# Initialize the detector parameters using default values
parameters = initialize_aruco_parameters()

# Load the dictionary that was used to generate the markers.
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
aruco_detector = aruco.ArucoDetector(aruco_dict, parameters)

# Specify the path to your AVI file
avi_file_path = "/home/ice/Downloads/output.mp4"

# Start the video capture from the AVI file
cap = cv2.VideoCapture(avi_file_path)

# Check if the video file opened successfully
if not cap.isOpened():
    print("Error: Could not open the video file.")
    exit()

detect_count = dict()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("End of video file reached or failed to read the frame.")
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    kernel = np.array([[0, -1, 0],
                       [-1, 5, -1],
                       [0, -1, 0]])
    gray = cv2.filter2D(gray, -1, kernel)

    gray = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 19, 5)

    # Detect the markers in the image
    corners, ids, rejectedImgPoints = aruco_detector.detectMarkers(gray)

    # Draw the detected markers on the frame
    if ids is not None:
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

        # Print detected marker IDs and their corners
        # print("Detected markers:")
        # for i, marker_id in enumerate(ids):
        #     print(f"ID: {marker_id[0]}")
        #     print(f"Corners: {corners[i][0]}")

        for i, marker_id in enumerate(ids):
            if detect_count.get(int(marker_id)):
                detect_count[int(marker_id)] += 1
            else:
                detect_count[int(marker_id)] = 1

    # Display the resulting frame
    cv2.imshow('AVI File ArUco Detection', gray)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(detect_count)
        break

print(detect_count)
# When everything is done, release the capture and close the windows
cap.release()
cv2.destroyAllWindows()
