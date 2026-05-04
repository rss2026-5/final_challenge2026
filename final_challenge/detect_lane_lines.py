import cv2
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################


def image_print(img):
    """
    Helper function to print out images, for debugging. Pass them in as a list.
    Press any key to continue.
    """
    cv2.imshow("image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def detect_lane_lines(img, lookahead_ratio=1.0):
    """
    Detect left and right lane lines in an image using color-based segmentation.

    This function processes an input image to identify lane markings (e.g., white lines)
    by applying HSV color thresholding, morphological filtering, and contour-based
    line fitting. It returns geometric representations of the detected lanes along
    with derived information useful for vehicle control.

    Pipeline:
        1. Convert image from BGR to HSV color space
        2. Apply color threshold to isolate lane markings
        3. Clean mask using morphological operations
        4. Detect contours corresponding to lane regions
        5. Filter contours based on size/shape
        6. Separate contours into left/right lanes
        7. Fit a line to each lane using cv2.fitLine
        8. Estimate lane center

    Args:
        img (np.ndarray or str):
            Input image (BGR). Can be either:
            - a NumPy array (already loaded image), or
            - a file path to an image.

    Returns:
        dict:
            {
                "left_line": (vx, vy, x, y) or None,
                    Line parameters for the left lane in vector form.

                "right_line": (vx, vy, x, y) or None,
                    Line parameters for the right lane in vector form.

                "lane_center": int or None,
                    Estimated x-coordinate of the lane center at the bottom
                    of the image.

                "image": np.ndarray,
                    Output image with detected lane lines drawn for visualization.
            }

    Notes:
        - If only one lane is detected, the other is estimated using a fixed lane width.
        - If no lanes are detected, center and steering error will be None.
        - Assumes camera is forward-facing with lanes roughly vertical in the image.
    """
    # Load image if given as path
    if isinstance(img, str):
        img_bgr = cv2.imread(img)
    else:
        img_bgr = img.copy()

    img_h, img_w = img_bgr.shape[:2]
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # --- Color thresholding for white lanes ---
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    mask_white = cv2.inRange(img_hsv, lower_white, upper_white)

    # --- Morphology to clean the mask ---
    kernel = np.ones((5,5), np.uint8)
    mask_clean = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

    # --- Edge detection ---
    edges = cv2.Canny(mask_clean, 50, 150)

    # --- Hough Transform ---
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=100, maxLineGap=50)

    left_lines = []
    right_lines = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1 + 1e-6)
            if abs(slope) < 0.3:
                continue  # ignore near-horizontal lines
            if slope < 0:
                left_lines.append(line[0])
            else:
                right_lines.append(line[0])

    def average_lane(lines):
        if len(lines) == 0:
            return None
        x_coords = []
        y_coords = []
        for x1, y1, x2, y2 in lines:
            x_coords += [x1, x2]
            y_coords += [y1, y2]
        # fit line y = mx + b
        poly = np.polyfit(y_coords, x_coords, 1)
        m, b = poly
        # return as vector form for compatibility
        y1 = img_h
        y2 = int(img_h * 0.5)
        x1 = int(m * y1 + b)
        x2 = int(m * y2 + b)
        vx = x2 - x1
        vy = y2 - y1
        return (vx, vy, x1, y1)

    left_line = average_lane(left_lines)
    right_line = average_lane(right_lines)

    def get_x_at_y(line, y):
        if line is None:
            return None
        vx, vy, x0, y0 = line
        if abs(vy) < 1e-6:
            return None
        return int(x0 + (y - y0) * vx / vy)
    
    y_eval = int(img_h * lookahead_ratio)
    left_x = get_x_at_y(left_line, y_eval)
    right_x = get_x_at_y(right_line, y_eval)

    lane_center = None
    if left_x is not None and right_x is not None:
        lane_center = (left_x + right_x) // 2
    elif left_x is not None:
        lane_center = left_x + 100
    elif right_x is not None:
        lane_center = right_x - 100

    # --- Visualization ---
    output_img = img_bgr.copy()
    def draw_line(img, line, color):
        if line is None:
            return
        vx, vy, x0, y0 = line
        y1 = img.shape[0]
        y2 = int(img.shape[0] * 0.5)
        x1 = int(x0 + (y1 - y0) * vx / vy)
        x2 = int(x0 + (y2 - y0) * vx / vy)
        cv2.line(img, (x1, y1), (x2, y2), color, 3)

    draw_line(output_img, left_line, (255, 0, 0))
    draw_line(output_img, right_line, (0, 0, 255))
    if lane_center is not None:
        cv2.circle(output_img, (lane_center, y_eval), 6, (0,255,255), -1)

    # # image_print(output_img)

    return {
        "left_line": left_line,
        "right_line": right_line,
        "lane_center": lane_center,
        "image": output_img
    }

# filename = "./racetrack_images/lane_3/image69.png"
# output = detect_lane_lines(filename, 0.55)

# print(output["lane_center"])