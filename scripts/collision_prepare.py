#!/usr/bin/env python

import rospy
import time
import numpy as np
import cv2
import sys
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Image, CameraInfo, JointState
import copy
import dougsm_helpers.tf_helpers as tfh
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gummi_interface.gummi import Gummi
from gummi_interface.msg import CoContraction

import cv_bridge
bridge = cv_bridge.CvBridge()

pub = None
base = None


def callback(msg):
    #make a copy of the image so that the end effector point can be inserted into the depth image
    curr_depth_img = copy.deepcopy(bridge.imgmsg_to_cv2(msg))

    #find the center for use in the distance calculation later
    centre = (curr_depth_img.shape[0]/2, curr_depth_img.shape[1]/2)

    #set the point you wish to measure everything to
    px, py, dp = get_ee_point(centre)

    #pass into function to calculate the distance of every pixel to your point
    min_dist, min_dist_px = calc_min_dist(curr_depth_img, px, py, dp, centre)

    #plotting of various things
    curr_depth_img = cv2.cvtColor(curr_depth_img,cv2.COLOR_GRAY2RGB)
    cv2.circle(curr_depth_img,(px, py),10,(0,0,255),-1)
    cv2.circle(curr_depth_img,(min_dist_px[1],min_dist_px[0]),10,(0,255,0),-1)
    cv2.imshow('image',curr_depth_img)
    cv2.waitKey(1)

    brace_for_impact(min_dist)


def brace_for_impact(min_dist):
    global pub 
    if pub is None:
        print("Hasn't been created yet.")
        return

    eps = 0.03 #minimum distance that you want to let it go to
    mod_effort = 1-(eps/min_dist)
    if mod_effort < 0:
        mod_effort = -1*mod_effort
    msg = CoContraction()
    msg.name = ['wrist_pitch','shoulder_roll','shoulder_yaw', 'shoulder_pitch', 'elbow']
    #name = ['wrist_pitch','shoulder_roll','head_yaw','upperarm_roll','shoulder_yaw', 'shoulder_pitch', 'elbow','head_pitch','forearm_roll']
    msg.effort = [ mod_effort, mod_effort, mod_effort,mod_effort, mod_effort]
    # effort = [ 0.5, 0.5, 100, 0.5, 100, 0.5, 0.5, 100]

    #base.effort = effort
    #rospy.logwarn(base.effort) 
    pub.publish(msg)
    
def calc_min_dist(img, px, py, dp, centre):

    f = 393.64373779296875  #Camera intrinsics

    #if the depth of the object is less than the point you are measuring to then set object depth to point depth
    img[img < img[py, px]] = img[py, px]

    #compute the distance of every point
    yv = np.arange(0, img.shape[0], 1).reshape((img.shape[0], 1)) - centre[0]
    xv = np.arange(0, img.shape[1], 1).reshape((1, img.shape[1])) - centre[1]

    dy = (yv * img)/f - (py-centre[0])*dp/f
    dx = (xv * img)/f - (px-centre[1])*dp/f
    dz = img - dp

    dist = np.sqrt(dy**2 + dx**2 + dz**2)
    
    # #make sure the point you are measuring to is excluded from the min function
    # dp[point[0], point[1]] = np.inf

    #make sure the nans arent there
    dist[np.isnan(dist)] = np.inf
    dist[centre[0]:,centre[1]:] = np.inf

    min_dist = dist.min()
    min_dist_px = np.unravel_index(np.argmin(dist), dist.shape)

    print('Min Distance: ', min_dist)
    print('Min Pixel: ', min_dist_px)

    return min_dist, min_dist_px

def get_ee_point(centre):
    #get the position of the end effector
    ee_pos = group.get_current_pose().pose
    #get pose wrt the optical frame
    ee_pos = tfh.convert_pose(ee_pos,'base_link',"camera_depth_optical_frame")
    #get rid of rotations
    ee_pos = [ee_pos.position.x, ee_pos.position.y, ee_pos.position.z]

    #translate that point into the projection in the depth space
    f = 393.64373779296875  #Camera intrinsics

    px = int((ee_pos[0]*f)/ee_pos[2]) + centre[1]
    py = int((ee_pos[1]*f)/ee_pos[2]) + centre[0]
    dp = ee_pos[2]

    return px, py, dp


if __name__ == '__main__':
    global pub
    pub = rospy.Publisher('/gummi/cocontraction', CoContraction, queue_size=1)
    rospy.init_node('collision_prepare', anonymous=True)
    rate = rospy.Rate(100)
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group = moveit_commander.MoveGroupCommander("right_arm")
    
    rospy.Subscriber("/camera/depth/image_meters", Image, callback)

    base = rospy.wait_for_message('/gummi/joint_states', JointState)
    print base

    while not rospy.is_shutdown():  
       rate.sleep()