#!/usr/bin/env python

import dougsm_helpers.tf_helpers as tfh
from ggcnn.srv import *
import tf.transformations as tft
import rospy
import sys
import random
import time
import numpy as np
from gummi_interface.gummi import Gummi
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def get_grasp():
    rospy.wait_for_service('/ggcnn_service/predict')
    try:
        ggcnn_service = rospy.ServiceProxy('/ggcnn_service/predict', GraspPrediction)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    predict = ggcnn_service.call()
    pose = predict.best_grasp.pose
    orientation = tfh.quaternion_to_list(pose.orientation)
    
    change_rot = list(tft.euler_from_quaternion(orientation))
    change_rot[0] = np.arctan(np.tan(change_rot[0]))
    change_rot[1] = np.arctan(np.tan(change_rot[1]))
    change_rot[2] = np.arctan(np.tan(change_rot[2]))
    
    orientation = tft.quaternion_from_euler(*change_rot)
    pose.orientation = tfh.list_to_quaternion(orientation)

    tfh.publish_pose_as_transform(pose, 'base_link', 'Grasp', 1.0)

    return pose

def move_to_grasp(pose):
    waypoints = []

    # start with the current pose
    waypoints.append(group.get_current_pose().pose)
    pose_target_1 = pose
    pose_target_1.position.x -= 0.05
    waypoints.append(copy.deepcopy(pose_target_1))
    pose_target_1.position.x += 0.05 
    waypoints.append(copy.deepcopy(pose_target_1))

    (plan3, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold


    # print "============ Printing robot state"
    # print robot.get_current_state()
    # print "============"

    # print "============ Generating plan 1"
    # print "Pose=>",group.get_current_pose().pose
    # pose_target_1 = pose
    # pose_target_1.orientation = group.get_current_pose().pose.orientation
    # plan2 = group.plan()
    # group.set_pose_target(pose_target_1)
    # print "============ Waiting while RVIZ displays plan1..."
    # rospy.sleep(1)

    # print "============ Visualizing plan1"
    # display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    # display_trajectory.trajectory_start = robot.get_current_state()
    # display_trajectory.trajectory.append(plan2)
    # display_trajectory_publisher.publish(display_trajectory)
    # rospy.sleep(1)

    group.go(wait=True)

    group.clear_pose_targets()

def move_to_pose(name):

    plan1 = group.plan()
    group.set_named_target(name)    
    group.go(wait=True)
    group.clear_pose_targets()

def push(pose):

    push_pose = group.get_current_pose().pose
    push_pose.position.x = push_pose.position.x + 0.03
    move_to_grasp(push_pose)

def choice_dict(operator, pose):
    if operator == 'find':
        pose = get_grasp()
    elif operator == 'move':
        move_to_grasp(pose)
    elif operator == 'home':
        move_to_pose('default')
    elif operator == 'start':
        move_to_pose(operator)
    elif operator == 'push':
        push(pose)
    else:
        pass
    return pose
    # return {
    #     'find': lambda: get_grasp(),
    #     'move': lambda: move_to_grasp(pose),
    #     'home': lambda: move_to_home(),
    # }.get(choice,lambda: None)(),


if __name__ == '__main__':
    rospy.init_node('move_to_grasp')
    moveit_commander.roscpp_initialize(sys.argv)
        
    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group = moveit_commander.MoveGroupCommander("pointer")

    display_trajectory_publisher = rospy.Publisher(
                                        '/moveit/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory,
                                        queue_size=20)
    rospy.sleep(2)
    print "============ Reference frame: %s" % group.get_planning_frame()
    print "============ Reference frame: %s" % group.get_end_effector_link()

    # print "============ Robot Groups:"
    # print robot.get_group_names()
    pose = get_grasp()
    choice = None
    while choice != 'finish':
        choice = raw_input("What would you like to do? \n Find a Grasp? \n Move to the Grasp? \n Move to Pose?\n")
        pose = choice_dict(choice,pose)
    
    
