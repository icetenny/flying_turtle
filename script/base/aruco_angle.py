import cv2
import cv2.aruco as aruco
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

# List available cameras
chosen_camera = list_cameras()

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

# Load the dictionary that was used to generate the markers
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

# Camera calibration parameters (replace these with your actual calibration results)
camera_matrix = np.array([[1000, 0, 320],
                          [0, 1000, 240],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))  # Assuming no lens distortion

# Start the video capture
cap = cv2.VideoCapture(chosen_camera)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 576)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect the markers in the image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Draw the detected markers on the frame
    if ids is not None:
        frame = aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Print detected marker IDs and their corners
        print("Detected markers:")
        for i, marker_id in enumerate(ids):
            print(f"ID: {marker_id[0]}")
            print(f"Corners: {corners[i][0]}")

            # Estimate pose of each marker
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, camera_matrix, dist_coeffs)
            
            # Draw axis for the marker
            # aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)
            # Convert rotation vector to rotation matrix
            rmat = cv2.Rodrigues(rvec[0])[0]

            # Extract the rotation angle in radians around the z-axis
            theta = np.arctan2(rmat[1, 0], rmat[0, 0])

            # Convert the angle to degrees
            angle_deg = np.degrees(theta)

            print(f'Marker ID: {marker_id[0]}, Rotation Angle: {angle_deg:.2f} degrees')

            # Display the rotation angle on the frame
            cv2.putText(frame, f'ID: {marker_id[0]}, Angle: {angle_deg:.2f}', 
                        (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

    # Display the resulting frame
    cv2.imshow('Webcam ArUco Detection', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture and close the windows
cap.release()
cv2.destroyAllWindows()
