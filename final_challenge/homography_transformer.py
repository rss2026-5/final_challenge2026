#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

import cv2

from geometry_msgs.msg import PointStamped

# The following collection of pixel locations and corresponding relative
# ground plane locations are used to compute our homography matrix

# PTS_IMAGE_PLANE units are in pixels
# see README.md for coordinate frame description

######################################################
PTS_IMAGE_PLANE = [[403, 286],
                   [203, 280],
                   [378, 249],
                   [569, 279],
                   [366, 195]]
######################################################

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
PTS_GROUND_PLANE = [[10,  0],
                    [10,  10],
                    [15,  0],
                    [10,  -10],
                    [35,  0]]
######################################################


# ######################################################
# PTS_IMAGE_PLANE = [[347, 171],
#                    [473, 272],
#                    [105, 319],
#                    [347, 353],
#                    [239, 182]]
# ######################################################

# # PTS_GROUND_PLANE units are in inches
# # car looks along positive x axis with positive y axis to left

# ######################################################
# PTS_GROUND_PLANE = [[51,  0],
#                     [17,  -3],
#                     [13,  9],
#                     [10.5,  0],
#                     [46, 12]]
# ######################################################

METERS_PER_INCH = 0.0254


class HomographyTransformer(Node):
    def __init__(self):
        super().__init__("homography_transformer")

        self.cone_pub = self.create_publisher(PointStamped, "/irl_lane_target", 10)
        self.cone_px_sub = self.create_subscription(PointStamped, "/lane_target", self.cone_detection_callback, 1)

        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rclpy.logerr("ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

        # Initialize data into a homography matrix

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

        self.get_logger().info("Homography Transformer Initialized")

    def cone_detection_callback(self, msg):
        # Extract information from message
        u = msg.point.x
        v = msg.point.y

        # Call to main function
        x, y = self.transformUvToXy(u, v)

        # Publish relative xy position of object in real world
        relative_xy_msg = PointStamped()
        relative_xy_msg.point.x = x
        relative_xy_msg.point.y = y

        self.cone_pub.publish(relative_xy_msg)

    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.

        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y

def main(args=None):
    rclpy.init(args=args)
    homography_transformer = HomographyTransformer()
    rclpy.spin(homography_transformer)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
