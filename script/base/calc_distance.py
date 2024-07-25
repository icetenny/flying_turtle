import math

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
    real_distance = math.sqrt(dW**2 + dH **2)

    return dW, dH, real_distance

# Example usage
px1, py1 = 0, 0  # Coordinates of the first point
px2, py2 = 640, 480  # Coordinates of the second point
width2height_ratio = 1.0  # Ratio of height to weight of cam
resolution = (640, 480)  # Resolution of the camera
known_distance = 0.67  # Known distance from camera to the plane of the points

dW, dH, d = calculate_real_distance(px1, py1, px2, py2, width2height_ratio, resolution, known_distance)
print(f"The real-life distance between the points is: {dW:.2f}, {dH:.2f}, {d:.2f} units")
