#!/usr/bin/env python

import rospy
import time
import numpy as np
import cv2
import copy
import cv_bridge
from sensor_msgs.msg import Image
import message_filters
import sys
bridge = cv_bridge.CvBridge()
from dougsm_helpers.timeit import TimeIt

class DilatedFilter():

    def __init__(self):
        rospy.init_node('mask_dilate', anonymous=True)

        self.depth_sub = message_filters.Subscriber("/camera/depth/image_meters", Image)
        self.filt_sub = message_filters.Subscriber("/camera/urdf_filtered_mask", Image)
        
        self.filt_pub = rospy.Publisher("/camera/urdf_filtered_dilated",Image, queue_size=1)

        self.kernel = np.ones((10,10),np.uint8)      

        self.ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.filt_sub], queue_size=1, slop=0.1)
        self.ts.registerCallback(self.callback)

    def callback(self, msg1, msg2):
        depth_img = bridge.imgmsg_to_cv2(msg1)
        filt_img = bridge.imgmsg_to_cv2(msg2)

        curr_mask_img_dil = cv2.dilate(filt_img, self.kernel, iterations=7)
        curr_mask_img_dil = (255 - curr_mask_img_dil)
        curr_mask_img_dil = cv2.normalize(curr_mask_img_dil, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        output = np.multiply(depth_img, curr_mask_img_dil)          
        output = bridge.cv2_to_imgmsg(output)
        df.filt_pub.publish(output)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    df = DilatedFilter()
    df.run()