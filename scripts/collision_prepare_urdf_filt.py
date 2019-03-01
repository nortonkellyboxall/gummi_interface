#!/usr/bin/env python

import rospy
import time
import numpy as np
import cv2
import sys
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_msgs.msg import Float64
import copy
import dougsm_helpers.tf_helpers as tfh
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gummi_interface.gummi import Gummi
from gummi_interface.msg import CoContraction
from dougsm_helpers.timeit import TimeIt

import cv_bridge
bridge = cv_bridge.CvBridge()

class CollisionPrepare():

    def __init__(self):
        self.pub = rospy.Publisher('/gummi/cocontraction', CoContraction, queue_size=1)
        self.pub2 = rospy.Publisher("/gummi/distance_to_ee",Float64,queue_size=1)
        self.pub3 = rospy.Publisher("/gummi/elbow_cocontraction",Float64, queue_size=1)
        self.sub = rospy.Subscriber("/camera1/urdf_filtered_dilated", Image, self.callback) 
        # self.sub2 = rospy.Subscriber("/camera2/depth/image_meters", Image, self.callback)
        
        self.avg_mod_effort = np.zeros([5])
        self.count = 0
        
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()
    
        self.group = moveit_commander.MoveGroupCommander("head")

        plan1 = self.group.plan()
        self.group.set_named_target('dab')    
        self.group.go(wait=True)
        self.group.clear_pose_targets()
        self.group = moveit_commander.MoveGroupCommander("right_arm")

        self.box = 0.30 #the half width of the observation bounding box
        self.f = 393.64373779296875  #Camera intrinsics

    def callback(self,msg):
        #make a copy of the image so that the end effector point can be inserted into the depth image

        curr_depth_img = copy.deepcopy(bridge.imgmsg_to_cv2(msg))
        plot_img = copy.deepcopy(curr_depth_img) 

        #find the center for use in the distance calculation later
        centre = (curr_depth_img.shape[0]/2, curr_depth_img.shape[1]/2)

        curr_depth_img[curr_depth_img < 0.3] = 100


        #set the point you wish to measure everything to
        px, py, dp = self.get_ee_point(centre)

        if not 0 <= px <= curr_depth_img.shape[1] and not 0 <= py <=curr_depth_img.shape[0]:
            rospy.logerr('Camera Cannot See the End Effector')
            print(px,py,dp)
        else:
            #Create bounding cube around EE to take measurements from
            
            # xs = int(self.box*(self.f/(dp-self.box)))
            # ys = int(self.box*(self.f/(dp-self.box)))

            # x = [(px-xs/2), (px+xs/2)] 
            # y = [(py-ys/2), (py+ys/2)]

            # if y[0] < 0:
            #     y[0] = 0
            # if y[1] > curr_depth_img.shape[0]:
            #     y[1] = curr_depth_img.shape[0]
            # if x[0] < 0:
            #     x[0] = 0
            # if x[1] > curr_depth_img.shape[1]:
            #     x[1] = curr_depth_img.shape[1]
            
            # img_crop = curr_depth_img[ y[0]:y[1] , x[0]:x[1]]
            # img_crop[np.isnan(img_crop)] = 100

            # img_filt = (img_crop > (dp-self.box)) * (img_crop < (dp+self.box))

            # img_crop = img_crop*img_filt
            # img_crop[img_crop == 0] = 100
            # img_crop = cv2.copyMakeBorder(img_crop,y[0],y[1],x[0],x[1],cv2.BORDER_CONSTANT,100)     

            #pass into function to calculate the distance of every pixel to your point

            # # Change the pixel value from the cropped pixel value to the full image
            # px_crop = px-x[0]
            # py_crop = py-y[0]
            # min_dist, min_dist_px = self.calc_min_dist(img_crop, px_crop, py_crop, dp, centre)
            min_dist, min_dist_px = self.calc_min_dist(curr_depth_img, px, py, dp, centre)

            #plotting of various things
            # plot_img = cv2.normalize(plot_img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            plot_img = cv2.cvtColor(plot_img,cv2.COLOR_GRAY2RGB)
            cv2.circle(plot_img,(px, py),10,(0,0,255),-1)
            # cv2.rectangle(plot_img,(x[0],y[0]),(x[1],y[1]),(255,0,0), 2)

            # cv2.circle(plot_img,(min_dist_px[1]+x[0],min_dist_px[0]+y[0]),10,(0,255,0),-1)
            cv2.circle(plot_img,(min_dist_px[1],min_dist_px[0]),10,(0,255,0),-1)
            

            cv2.imshow('image2',plot_img)
            cv2.waitKey(1)

            self.brace_for_impact(min_dist)
            

    def get_ee_point(self, centre):

        #get the position of the end effector
        ee_pos = self.group.get_current_pose().pose
        #get pose wrt the optical frame
        ee_pos = tfh.convert_pose(ee_pos,'base_link',"camera1_depth_optical_frame")
        #get rid of rotations
        ee_pos = [ee_pos.position.x, ee_pos.position.y, ee_pos.position.z]


        px = int(((ee_pos[0]*self.f)/ee_pos[2]) + centre[1])
        py = int(((ee_pos[1]*self.f)/ee_pos[2]) + centre[0])
        dp = ee_pos[2]
        print(px, py, dp)

        return px, py, dp

    def calc_min_dist(self, img, px, py, dp, centre):

        #if the depth of the object is less than the point you are measuring to then set object depth to point depth
        img[img < dp] = dp
        img[img < 0] = np.nan
        yv = np.arange(0, img.shape[0], 1).reshape((img.shape[0], 1)) - centre[0]
        xv = np.arange(0, img.shape[1], 1).reshape((1,img.shape[1])) - centre[1]

        dy = (yv * img)/self.f - (py-centre[0])*dp/self.f
        dx = (xv * img)/self.f - (px-centre[1])*dp/self.f
        dz = img - dp

        dist = np.sqrt(dy**2 + dx**2 + dz**2)

        #make sure the nans arent there
        dist[np.isnan(dist)] = 100
        # dist[centre[0]:,centre[1]:] = 100
        if len(dist) > 0:
            min_dist = dist.min()
            min_dist_px = np.unravel_index(np.argmin(dist), dist.shape)

            if min_dist < 0.05:
                min_dist = 0.05

            print('Min Distance: ', min_dist)
            print('Min Pixel: ', min_dist_px)            
        else:
            min_dist = 1
            min_dist_px = [1,1]
            print('Min Distance: ', min_dist)
            print('Min Pixel: ', min_dist_px)       
        self.pub2.publish(min_dist) 

        return min_dist, min_dist_px


    def brace_for_impact(self, min_dist):

        if self.pub is None:
            print("Hasn't been created yet.")
            return
        msg = CoContraction()

        lower, upper = 0.25, 1
        eps = 0.05 #minimum distance that you want to let it go to
        mod_effort = (1-(eps/min_dist)) #linear decay

        mod_effort_2 = -1*(-1-(1-np.exp(eps/min_dist))) #Exponential   

        mod_effort_3 = -1*(1/np.exp(eps/min_dist))
        # effort = [mod_effort,mod_effort_2,mod_effort_3]

        #Sanity Checking to make sure effort doesnt go outside bounds    
        if mod_effort < 0:
            mod_effort = 0

        if mod_effort > 1:
            mod_effort = 1
        #Ensure effort only ranges between lower and upper
        mod_effort = lower + (upper-lower)*mod_effort

        # self.count %= len(self.avg_mod_effort)
        # self.avg_mod_effort[self.count] = mod_effort
        # self.count = self.count + 1
        # mod_effort = np.mean(self.avg_mod_effort)
        
        # msg.name = ['shoulder_pitch']
        # msg.effort = [mod_effort]

        msg.name = ['wrist_pitch','shoulder_roll','shoulder_yaw', 'shoulder_pitch', 'elbow']
        msg.effort = [ mod_effort, mod_effort, mod_effort,mod_effort, mod_effort]
        rospy.logwarn(mod_effort)
        self.pub3.publish(mod_effort)

        self.pub.publish(msg)
        
if __name__ == '__main__':

    rospy.init_node('collision_prepare', anonymous=True)
    rate = rospy.Rate(50)

    cp = CollisionPrepare()

    rospy.wait_for_message('/gummi/joint_states', JointState)
    rospy.wait_for_message('/camera2/depth/image_meters', JointState)

    while not rospy.is_shutdown():  
       rate.sleep()