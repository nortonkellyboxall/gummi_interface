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
import message_filters
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
        #publishers for data gathering 
        self.pub = rospy.Publisher('/gummi/cocontraction', CoContraction, queue_size=1)
        self.pub2 = rospy.Publisher("/gummi/distance_to_ee",Float64,queue_size=1)
        #publisher for control
        self.pub3 = rospy.Publisher("/gummi/elbow_cocontraction",Float64, queue_size=1)
        self.sub = message_filters.Subscriber("/camera2/color/image_rect_color", Image) #/camera1 is the camera on the frame
        self.sub2 = message_filters.Subscriber("/camera2/depth/image_meters", Image) #/camera2 is the camera not mounted on the frame

        #This allows both image streams to be relatively synced
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub, self.sub2], queue_size=1, slop=0.1)
        self.ts.registerCallback(self.cb) 

        #initialising various things
        self.avg_mod_effort = np.zeros([5]) #creating to allow for a moving average filter
        self.count = 0                      #for moving average filter
        
        #Initialising MoveIt and moving the head to a position that can see the arm
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()  
        self.group = moveit_commander.MoveGroupCommander("head")
        plan1 = self.group.plan()
        self.group.set_named_target('dab')    
        self.group.go(wait=True)
        self.group.clear_pose_targets()
        self.group = moveit_commander.MoveGroupCommander("right_arm")  #changing move group back to arm

        #Initialising constants for a region of interest box
        self.box = 0.30 #the half width of the observation bounding box
        self.fx = 618.57080078125  #Camera intrinsics
        self.fy = 618.9249267578125

        #Initialising kernal for erosion function
        self.kernel = np.ones((5,5),np.uint8)

        self.rate = rospy.Rate(60)

        self.filt_img = None
        self.plot_img = None
        self.debug_img = None

    def cb(self,msg1,msg2):

        ##Creating a green colour filter to act as a binary filter for the depth image
        col_img = copy.deepcopy(bridge.imgmsg_to_cv2(msg1))
        hsv_img = cv2.cvtColor(col_img, cv2.COLOR_BGR2HSV)

        # define range of pink color in HSV
        lower_hsv = np.array([35,240,30])
        upper_hsv = np.array([40,255,50])

        # Threshold the HSV image to get only green colors
        col_mask = cv2.inRange(hsv_img, lower_hsv, upper_hsv)
        col_mask = cv2.erode(col_mask,self.kernel,iterations = 1)
        col_mask = cv2.normalize(col_mask, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

        ## Operate on the Depth Image
        plot_img = copy.deepcopy(bridge.imgmsg_to_cv2(msg2))
        filt_depth_img = np.multiply(plot_img, col_mask)
        filt_img = np.multiply(plot_img, col_mask)

        #find the center for use in the distance calculation later
        centre = (filt_depth_img.shape[0]/2, filt_depth_img.shape[1]/2)

        # curr_depth_img[curr_depth_img < 0.3] = 100

        #set the point you wish to measure everything to
        px, py, dp = self.get_ee_point(centre)


        if not 0 <= px <= filt_depth_img.shape[1] and not 0 <= py <=filt_depth_img.shape[0]:
            rospy.logerr('Camera Cannot See the End Effector')
            print(px,py,dp)
        else:
            ##Create area of interest cube around EE to take measurements from
            
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

            ## Find the minimum distance and pixel coordintate in the image frame
            min_dist, min_dist_px = self.calc_min_dist(filt_depth_img, col_mask, px, py, dp, centre)

            #plotting of various things
            # # plot_img = cv2.normalize(plot_img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            plot_img = cv2.cvtColor(plot_img,cv2.COLOR_GRAY2RGB)
            cv2.circle(plot_img,(px, py),10,(0,0,255),-1)                           #red ee point
            cv2.circle(plot_img,(min_dist_px[1],min_dist_px[0]),10,(255,0,0),-1)    #Blue Object point

            ## Use if AOE is applied
            # cv2.rectangle(plot_img,(x[0],y[0]),(x[1],y[1]),(255,0,0), 2)
            # cv2.circle(plot_img,(min_dist_px[1]+x[0],min_dist_px[0]+y[0]),10,(0,255,0),-1)

            
            self.plot_img = plot_img
            self.filt_img = filt_img
            
            ## Apply minimum distance to controller for stiffness
            self.brace_for_impact(min_dist)
            

    def get_ee_point(self, centre):

        #get the position of the end effector
        ee_pos = self.group.get_current_pose().pose
        #get pose wrt the optical frame       
        ee_pos = tfh.convert_pose(ee_pos,'base_link',"camera2_depth_optical_frame")
        #get rid of rotations
        ee_pos = [ee_pos.position.x, ee_pos.position.y, ee_pos.position.z]


        px = int(((ee_pos[0]*self.fx)/ee_pos[2]) + centre[1])
        py = int(((ee_pos[1]*self.fy)/ee_pos[2]) + centre[0])
        dp = ee_pos[2]

        return px, py, dp

    def calc_min_dist(self, img, mask, px, py, dp, centre):

        #if the depth of the object is less than the point you are measuring to then set object depth to point depth
        img1 = copy.copy(img)
        pixels = np.argwhere(mask==1)
        dp = dp + 0.15
        yv = pixels[:,0]
        xv = pixels[:,1]

        ydist = yv - centre[0]
        xdist = xv - centre[1]


        # yv = np.arange(0, img.shape[0], 1).reshape((img.shape[0], 1)) - centre[0]
        # xv = np.arange(0, img.shape[1], 1).reshape((1,img.shape[1])) - centre[1]


        dy = (ydist * img1[yv,xv])/self.fy - (py-centre[0])*dp/self.fy
        dx = (xdist * img1[yv,xv])/self.fx - (px-centre[1])*dp/self.fx
        dz = img1[yv,xv] - dp

        # dy = (yv * img)/self.fy - (py-centre[0])*dp/self.fy
        # dx = (xv * img)/self.fx - (px-centre[1])*dp/self.fx
        # dz = img - dp


        dist = np.sqrt(dy**2 + dx**2 + dz**2)


        #make sure the nans arent there
        dist[np.isnan(dist)] = 100
        # dist[centre[0]:,centre[1]:] = 100
        if len(dist) > 0:
            min_dist = dist.min()
            # find where min_dist is in dist
            # dist -> yv,xv mapping
            # yv[dist[min_dist]],xv[dist[min_dist]] == im_px
            min_dist_arg = np.argmin(dist)
            min_dist_px = yv[min_dist_arg] , xv[min_dist_arg]
            # min_dist_px = np.unravel_index(np.argmin(dist), dist.shape)
            print('Min Distance: ', min_dist)
            print('Min Pixel: ', min_dist_px)   

            if min_dist < 0.05:
                min_dist = 0.05

            print('Min Distance: ', min_dist)
            print('Min Pixel: ', min_dist_px)            
        else:
            min_dist = 1
            min_dist_px = [1,1]
            print('Min Distance: ', min_dist)
            print('Min Pixel: ', min_dist_px)       
        # self.pub2.publish(min_dist) 

        return min_dist, min_dist_px


    def brace_for_impact(self, min_dist):
        if self.pub is None:
            print("Hasn't been created yet.")
            return
        msg = CoContraction()

        lower, upper = 0.1, 1
        eps = 0.47 #minimum distance that you want to let it go to
        if min_dist < eps:
            min_dist = eps

        else:
            min_dist = 100

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
        
        msg.name = ['shoulder_pitch']
        msg.effort = [mod_effort]

        # msg.name = ['wrist_pitch','shoulder_roll','shoulder_yaw', 'shoulder_pitch', 'elbow']
        # msg.effort = [ mod_effort, mod_effort, mod_effort,mod_effort, mod_effort]
        # self.pub3.publish(mod_effort)

        self.pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():  
            show_img = False
            if self.plot_img is not None and self.filt_img is not None:
                cv2.imshow('Plot image', self.plot_img)
                cv2.imshow('Current Image', self.filt_img)
                show_img = True

            if self.debug_img is not None:
                cv2.imshow('Debug img1', self.debug_img)
                show_img = True
            
            if show_img:
                cv2.waitKey(1)
            self.rate.sleep()
        
if __name__ == '__main__':

    rospy.init_node('collision_prepare', anonymous=True)
    tfh.init()
    cp = CollisionPrepare()
    
    
    rospy.wait_for_message('/gummi/joint_states', JointState)
    rospy.wait_for_message('/camera2/depth/image_meters', JointState)
    rospy.wait_for_message('/camera2/color/image_rect_color', JointState)

    cp.run()
 