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


def detect_lane_lines(img):
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
        8. Estimate lane center and steering error

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
    # --- image preprocessing ---
    # convert from BGR to HSV
    filename = ""
    img_bgr = cv2.imread(img)
    # img_bgr = img
    # image_print(img_bgr)

    # define region of interest and crop
    img_h, img_w = img_bgr.shape[:2]
    img_roi = img_bgr[0:img_h, 0:img_w]
    # image_print(img_roi)

    img_hsv = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)
    # image_print(img_hsv)
    # cv2.imwrite(filename + "_hsv.jpg", img_hsv)    

    # H: 0–180, S: 0–50, V: 200–255
    # define color thresholds & generate mask
    lower = np.array([0, 0, 200])
    upper = np.array([180, 50, 255])
    hsv_mask = cv2.inRange(img_hsv, lower, upper) 

    # Clean it
    kernel = np.ones((5, 5), np.uint8)
    img_clean = cv2.morphologyEx(hsv_mask, cv2.MORPH_OPEN, kernel)
    img_clean = cv2.morphologyEx(img_clean, cv2.MORPH_CLOSE, kernel)

    # image_print(img_clean)

    contours, _ = cv2.findContours(img_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # print(len(contours))

    # --- lane detection ---
    lane_contours = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        # print(f"area: {area}")
        if area < 500:  # tune this
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        # cv2.rectangle(img_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # image_print(img_bgr)

        # lanes are typically taller than wide
        if h > 50:
            lane_contours.append(cnt)

    # print(f"lane contours: {lane_contours}")
    mid_x = img_w // 2

    left_lane = []
    right_lane = []

    for cnt in lane_contours:
        x, y, w_box, h_box = cv2.boundingRect(cnt)
        cx = x + w_box // 2

        if cx < mid_x:
            left_lane.append(cnt)
        else:
            right_lane.append(cnt)

    # print(f"left lane: {left_lane}")
    # print(f"right lane: {right_lane}")

    def fit_lane(contours):
        if not contours:
            return None
        
        points = np.vstack(contours)
        vx, vy, x, y = [v[0] for v in cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)]
        return vx, vy, x, y
    
    left_line = fit_lane(left_lane)
    right_line = fit_lane(right_lane)

    def get_x_at_y(line, y):
        if line is None:
            return None
        
        vx, vy, x, y0 = line
        
        if abs(vy) < 1e-6:
            return None
        
        return int(x + (y - y0) * vx / vy)
    
    y_eval = img_bgr.shape[0]

    left_x = get_x_at_y(left_line, y_eval)
    right_x = get_x_at_y(right_line, y_eval)

    lane_center = None
    if left_x is not None and right_x is not None:
        lane_center = (left_x + right_x) // 2

    elif left_x is not None:
        # estimate right lane
        lane_width = 300  # tune this
        lane_center = left_x + lane_width // 2

    elif right_x is not None:
        # estimate left lane
        lane_width = 300
        lane_center = right_x - lane_width // 2


    # # --- visualization ---
    # def draw_line(img, line, color):
    #     if line is None:
    #         return
        
    #     vx, vy, x, y = line
        
    #     h = img.shape[0]
        
    #     # extend line across ROI
    #     y1 = h
    #     y2 = int(h * 0.5)

    #     x1 = int(x + (y1 - y) * vx / vy)
    #     x2 = int(x + (y2 - y) * vx / vy)

    #     cv2.line(img, (x1, y1), (x2, y2), color, 3)

    # draw_line(img_bgr, left_line, (255, 0, 0))
    # draw_line(img_bgr, right_line, (255, 0, 0))

    # if lane_center is not None:
    #     h = img_bgr.shape[0]

    #     # Draw a vertical line at lane center
    #     cv2.line(img_bgr, (lane_center, h), (lane_center, int(h * 0.5)), (0, 255, 255), 2)

    #     # Draw a circle at bottom center point
    #     cv2.circle(img_bgr, (lane_center, h - 10), 6, (0, 255, 255), -1)

    # image_print(img_bgr)


    return {
        "left_line": left_line,
        "right_line": right_line,
        "lane_center": lane_center,
        "image": img_bgr
        }  

filename = "./racetrack_images/lane_3/image1.png"
output = detect_lane_lines(filename)

# print(output["lane_center"])