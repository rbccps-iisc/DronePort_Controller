#!/user/bin/env python

from __future__ import print_function

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

        #self.params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        #self.params.minThreshold = 1

        #self.params.filterByArea = True

        #self.params.minArea = 1

        #self.params.minDistBetweenBlobs = 5

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #(rows, cols, channels) = cv_image.shape
        #if cols > 60 and rows > 60:
        #    cv2.circle(cv_image, (50, 50), 10, 255)
        #cv2.imshow("Image window", cv_image)
        #cv2.waitKey(3)

        if (self.set_s == 0):
            self.prev_cv_image = cv_image
            self.set_s = 1
            print("Here Once")
        #detector = cv2.SimpleBlobDetector_create(self.params)

        curr_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        prev_gray = cv2.cvtColor(self.prev_cv_image, cv2.COLOR_BGR2GRAY)

        frame_diff = cv2.absdiff(curr_gray, prev_gray)

        #keypoints = detector.detect(frame_diff)

        #print(keypoints)
        #im_with_keypoints = cv2.drawKeypoints(frame_diff, keypoints, np.array([]), (0, 0, 255),
        #                                      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        cv2.imshow("Image window", cv_image)
        cv2.imshow('frame diff ', frame_diff)
        #cv2.imshow("Keypoints", im_with_keypoints)
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
