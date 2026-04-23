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


def color_segmentation(img, template=None):
    """
    Implement the line detection using color segmentation algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected. BGR.
        template: Not required, but can optionally be used to automate setting hue filter values.
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
            (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
    """
    ########## YOUR CODE STARTS HERE ##########

    # convert from BGR to HSV
    filename = ""
    img_bgr = cv2.imread(img)
    # img_bgr = img
    image_print(img_bgr)
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    # image_print(img_hsv)
    # cv2.imwrite(filename + "_hsv.jpg", img_hsv)    

    # H: 0–180, S: 0–50, V: 200–255

    # define color thresholds & generate mask
    lower1 = np.array([0, 0, 200])
    upper1 = np.array([180, 50, 255])
    # lower2 = np.array([165, 50, 50])
    # upper2 = np.array([185, 255, 255])
    hsv_mask = cv2.inRange(img_hsv, lower1, upper1) #+ cv2.inRange(img_hsv, lower2, upper2)
    # hsv_mask = cv2.inRange(img_hsv, lower2, upper2) 

    img_masked = cv2.bitwise_and(img_bgr, img_bgr, mask=hsv_mask)
    image_print(img_masked)
    # cv2.imwrite(filename 
    # cv2.imwrite(filename + "_mask_init.jpg", img_masked)  

    # image cleanup / noise reduction
    kernel = np.ones((3, 3), np.uint8)
    img_erode = cv2.erode(img_masked, kernel, iterations=1)
    image_print(img_erode) 

    kernel = np.ones((3, 3), np.uint8)
    img_dilate = cv2.dilate(img_erode, kernel, iterations=1)
    image_print(img_dilate)  
    # cv2.imwrite(filename + "_mask_final.jpg", img_dilate)  

    # detect contours
    img_gray = cv2.cvtColor(img_dilate, cv2.IMREAD_GRAYSCALE)
    img_edges = cv2.Canny(img_gray, 199, 200)

    # img_edges = cv2.Canny(img_masked, 199, 200)

    image_print(img_edges)
    # cv2.imwrite(filename + "_edges.jpg", img_edges)  

    contours, _ = cv2.findContours(img_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    # ---- FROM THE CONE DETECTION ----
    # area_list = []
    x_list = []
    y_list = []
    w_list = []
    h_list = []

    # # draw bounding box
    # for i in range(len(contours)):        
    #     x, y, w, h = cv2.boundingRect(contours[i])
    #     x_list.append(x)
    #     y_list.append(y)
    #     w_list.append(w)
    #     h_list.append(h)
    #     # area = w*h
    #     # area_list.append(area)

    # # print(area_list)
    # if h_list != []:
    #     chosen = h_list.index(max(h_list))  
    #     bounding_box = ((x_list[chosen], y_list[chosen]), (x_list[chosen] + w_list[chosen], y_list[chosen] + h_list[chosen]))
    # else: bounding_box = ((0,0), (0,0))

    # cv2.rectangle(img_bgr, (x_list[chosen], y_list[chosen]), (x_list[chosen] + w_list[chosen], y_list[chosen] + h_list[chosen]), (0, 255, 0), 2)
    # image_print(img_bgr)
    # cv2.imwrite(filename + "_boxed.jpg", img_bgr) 
    # print(bounding_box)

    # else: bounding_box = ((0,0), (0,0))
    
    # ---- FROM THE CONE DETECTION ----


    # --- trying to see what contours are recognized ---
    # for i in range(len(contours)):
    #     x, y, w, h = cv2.boundingRect(contours[i])
    #     cv2.rectangle(img_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
    #     image_print(img_bgr)
    #     bounding_box = ((x, y), (x + w, y + h))
    #     print(i, bounding_box)

    bounding_box = ((0,0), (0,0))

    ########### YOUR CODE ENDS HERE ###########

    # Return bounding box
    return bounding_box

filename = "./racetrack_images/lane_3/image3.png"
color_segmentation(filename)