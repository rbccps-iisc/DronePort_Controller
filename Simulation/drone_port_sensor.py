#!/user/bin/env python

from __future__ import print_function

import imutils as imutils
import roslib
import sys
import rospy
from cv2 import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_parse:
    def __init__(self):
        self.prev_cv_image = np.zeros((640, 480, 3), np.uint8)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/Marker0/c920/image_raw", Image,
                                          self.callback)
        self.set_s = 0

        #centroid of UAV
        self.cX = 0
        self.cY = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # (rows, cols, channels) = cv_image.shape
        # if cols > 60 and rows > 60:
        #    cv2.circle(cv_image, (50, 50), 10, 255)
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)

        if (self.set_s == 0):
            self.prev_cv_image = cv_image.copy()
            self.set_s = 1
            binary = cv_image.copy()
            # print("Here Once")

        curr_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        prev_gray = cv2.cvtColor(self.prev_cv_image, cv2.COLOR_BGR2GRAY)

        frame_diff = cv2.absdiff(curr_gray, prev_gray)


        ret, thresh = cv2.threshold(frame_diff, 30, 255, cv2.THRESH_BINARY)

        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(thresh, kernel, iterations=1)

        cnts = cv2.findContours(dilated.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        ctemp_c = cv_image.copy()
        valid_cntr = []
        for c in cnts:
            #(x, y, w, h) = cv2.boundingRect(c)
            M = cv2.moments(c)

            self.cX = int(M["m10"] / M["m00"])
            self.cY = int(M["m01"] / M["m00"])

        cv2.drawContours(ctemp_c, cnts, -1, (127, 200, 0), 2)
        cv2.circle(ctemp_c,(self.cX,self.cY),7, (255,255,255),-1)

        print("center",self.cX,self.cY)
        # Detect blobs(groups of pixels)
        # image = self.DetectBlobs(morphed, cv_image)  # Detect groups using countour area (Green formula)

        cv2.imshow("Image window", cv_image)
        cv2.imshow('frame diff ', frame_diff)
        cv2.imshow("contour",ctemp_c)
        cv2.waitKey(1)
        self.prev_cv_image = cv_image.copy()


def main(args):
    ic = image_parse()
    rospy.init_node('image_parse', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
