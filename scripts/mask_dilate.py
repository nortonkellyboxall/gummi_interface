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

class DilatedFilter():

    def __init__(self):

        self.depth_sub = rospy.Subscriber("/camera/depth/image_meters", Image, self.depthcb)
        self.filt_sub = rospy.Subscriber("/camera/urdf_filtered_mask", Image, self.filtcb)
        self.filt_pub = rospy.Publisher("/camera/urdf_filtered_dilated",Image,queue_size=1)
        self.depth_img = None
        self.filt_img = None


    def depthcb(self, msg):
        self.depth_img = copy.deepcopy(bridge.imgmsg_to_cv2(msg))

    def filtcb(self, msg):
        self.filt_img = copy.deepcopy(bridge.imgmsg_to_cv2(msg))    


if __name__ == '__main__':
    rospy.init_node('mask_dilate', anonymous=True)
    rate = rospy.Rate(100)
    df = DilatedFilter()
    rospy.wait_for_message("/camera/depth/image_meters",Image)

    while not rospy.is_shutdown():
        kernel = np.ones((7,7),np.uint8)
        curr_mask_img_dil = cv2.dilate(df.filt_img,kernel,iterations=5)
        curr_mask_img_dil = 255 - curr_mask_img_dil
        output = np.multiply(df.depth_img, curr_mask_img_dil)
        output = bridge.cv2_to_imgmsg(output)
        df.filt_pub.publish(output)
        # cv2.imshow('image',output)
        # cv2.waitKey(1)        
        rate.sleep()
