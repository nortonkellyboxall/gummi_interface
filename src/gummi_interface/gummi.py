#!/usr/bin/env python

import rospy
import sys

# from threading import Lock

# from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from msg import CoContraction

from antagonist import Antagonist
from direct_drive import DirectDrive

class Gummi:

    def __init__(self):
        self.initVariables()
        self.initJoints()
        self.initPublishers()

        # Calling initSubscribers automatically may create conflicts
        # that will force the use of locks when it's not necessary,
        # probably slowing down the system.
        self.initSubscribers()


    def initVariables(self):
        self.pi = 3.1416

        self.teleop = rospy.get_param("~teleop", 1)
        if self.teleop:
            rospy.logwarn("Expecting teleoperation ('teleop' parameter in gummi.yaml file), current value: " + str(self.teleop) + ".")

        self.velocity_control = rospy.get_param("~velocity_control", 1)
        if self.velocity_control:
            rospy.logwarn("Expecting velocity control ('velocity_control' parameter in gummi.yaml file), current value: " + str(self.velocity_control) + ".")

        self.joints = rospy.get_param("~joints")

        # the JointState object is created here to save time later
        self.JoinStateMsg = JointState()
        self.JoinStateMsg.name = self.joints.keys()
        self.JoinStateMsg.position = [None]*len(self.JoinStateMsg.name)
        self.JoinStateMsg.velocity = [None]*len(self.JoinStateMsg.name)
        self.JoinStateMsg.effort = [None]*len(self.JoinStateMsg.name)


    def initJoints(self):
        for name in self.joints.keys():
            if self.joints[name]['antagonist']:
                self.joints[name]['controller'] = Antagonist(name)
            else:
                self.joints[name]['controller'] = DirectDrive(name)

            self.joints[name]['position'] = self.joints[name]['controller'].getJointAngle()
            self.joints[name]['velocity'] = self.joints[name]['controller'].getJointVelocity()


    def initPublishers(self):
        self.jointStatePub = rospy.Publisher("gummi/joint_states", JointState,  queue_size=1)


    def initSubscribers(self):
        rospy.logwarn("Processing commands from gummi/joint_commands. Don't try to control it in parallel!")
        rospy.Subscriber('gummi/joint_commands', JointState, self.cmdCallback)
        rospy.Subscriber('gummi/cocontraction', CoContraction, self.setCocontraction, queue_size=1)


    def cmdCallback(self, msg):
        if self.teleop == 1 or self.velocity_control == 1:
            for name, velocity, effort in zip(msg.name, msg.velocity, msg.effort):
                self.joints[name]['velocity'] = velocity
                self.joints[name]['effort'] = effort
            if len(msg.position) is not 0:
                rospy.logwarn("Receiving position list, but in teleop mode. Ignoring.")
            else:
                self.doVelocityUpdate()
        else:
            for name, position, velocity, effort in zip(msg.name, msg.position, msg.velocity, msg.effort):
                self.joints[name]['position'] = position
                self.joints[name]['effort'] = effort
            if len(msg.position) is 0:
                rospy.logwarn("Receiving zero length position list, but not in teleop mode. Ignoring.")
            else:
                if sum([effort >=0 for effort in msg.effort]):
                    self.servoTo()
                else:
                    self.passiveHold()


    def doUpdate(self):
        for name in self.joints.keys():
            self.joints[name]['controller'].doUpdate()


    def doVelocityUpdate(self):
        for name in self.joints.keys():
                if self.joints[name]['effort'] < 0: # shift equilibrium point in open loop
                    if self.joints[name]['antagonist']:
                        self.joints[name]['controller'].moveWith(self.joints[name]['velocity'],
                                                                 abs(self.joints[name]['effort']))
                    else: # directdrive
                        self.joints[name]['controller'].servoWith(self.joints[name]['velocity'],
                                                                 abs(self.joints[name]['effort']))
                else: # normal movement
                    self.joints[name]['controller'].servoWith(self.joints[name]['velocity'],
                                                              self.joints[name]['effort'])
        self.publishJointState()


    def publishJointState(self):
        self.JoinStateMsg.header.stamp = rospy.Time.now()
        for i,name in enumerate(self.JoinStateMsg.name):
            self.JoinStateMsg.position[i] = self.joints[name]['controller'].getJointAngle()
            self.JoinStateMsg.velocity[i] = self.joints[name]['controller'].getJointVelocity()
            self.JoinStateMsg.effort[i] = self.joints[name]['effort']

        self.jointStatePub.publish(self.JoinStateMsg)

    def servoTo(self):
        if self.teleop == 0:
            for name in self.joints.keys():
                if self.joints[name]['antagonist']:
                    self.joints[name]['controller'].servoTo(self.joints[name]['position'],
                                                            self.joints[name]['effort'])
                else:
                    self.joints[name]['controller'].setTorqueLimit(self.joints[name]['effort'])
                    self.joints[name]['controller'].servoTo(self.joints[name]['position'])
            self.publishJointState()
        else:
            rospy.logwarn("Asked to servo to pose, but ignoring as in teleop mode. Check gummi.yaml file.")


    def passiveHold(self):
        for name in self.joints.keys():
            if self.joints[name]['antagonist']:
                self.joints[name]['controller'].passiveHold(self.joints[name]['effort'])
            else:
                self.joints[name]['controller'].doUpdate()


    def goRestingPose(self, now):
        for name in self.joints.keys():
            if self.joints[name]['antagonist']:
                self.joints[name]['controller'].servoTo(self.joints[name]['controller'].getRestingPoseAngle(),
                                                     self.joints[name]['effort'])
            else:
                self.joints[name]['controller'].servoTo(0)


    def setCollisionResponses(self, **args):
        for name in args.keys():
            if self.joints[name]['antagonist']:
                self.joints[name]['controller'].setCollisionResponse(args[name])
            else:
                rospy.logerr("Tried to setCollisionResponses to non-antagonist joint: {0}".format(name))


    def doGradualStartup(self):
        for name in self.joints.keys():
            if self.joints[name]['antagonist']:
                self.joints[name]['controller'].moveTo(self.joints[name]['gradual_startup_position'],
                                                       self.joints[name]['effort'])
            else:
                self.joints[name]['controller'].servoTo(self.joints[name]['gradual_startup_position'])
            rospy.sleep(self.joints[name]['gradual_startup_time'])

    def testLimits(self):
        for name in self.joints.keys():
            if self.joints[name]['antagonist']:
                raw_input("Press Enter to move the %s joint to its minimum postion" % self.joints[name]['controller'].name)
                self.joints[name]['controller'].moveTo(self.joints[name]['controller'].angle.minAngle,
                                                       self.joints[name]['effort'])
                raw_input("Press Enter to move the %s joint to its maximum postion" % self.joints[name]['controller'].name)
                self.joints[name]['controller'].moveTo(self.joints[name]['controller'].angle.maxAngle,
                                                       self.joints[name]['effort'])
                raw_input("Press Enter to move the %s joint to its zero postion" % self.joints[name]['controller'].name)
                self.joints[name]['controller'].moveTo(0,
                                                       self.joints[name]['effort'])
            else:
                raw_input("Press Enter to move the %s joint to its minimum postion" % self.joints[name]['controller'].name)
                self.joints[name]['controller'].servoTo(self.joints[name]['controller'].angle.minAngle)
                raw_input("Press Enter to move the %s joint to its maximum postion" % self.joints[name]['controller'].name)
                self.joints[name]['controller'].servoTo(self.joints[name]['controller'].angle.maxAngle)
                raw_input("Press Enter to move the %s joint to its zero postion" % self.joints[name]['controller'].name)
                self.joints[name]['controller'].servoTo(0)

    def doZeroAllServos(self):
        for name in self.joints.keys():
            if self.joints[name]['antagonist']:
                self.joints[name]['controller'].eqModel.flexor.servoTo(0)
                self.joints[name]['controller'].eqModel.extensor.servoTo(0)
            else:
                self.joints[name]['controller'].servoTo(0)
            rospy.sleep(self.joints[name]['gradual_startup_time'])

    def doSetPoseToDesired(self):
        for name in self.joints.keys():
            angle = self.joints[name]['controller'].getJointAngle()
            if self.joints[name]['antagonist']:
                self.joints[name]['controller'].servoTo(angle, self.joints[name]['effort'])
            else:
                self.joints[name]['controller'].servoTo(angle)

    # This function is not used anywhere here...
    def getJointAngles(self):
        angles = list()
        for name in self.joints.keys():
            angles.append(self.joints[name]['controller'].getJointAngle())
        return angles

    # This function is not used anywhere here...
    def getJointVelocities(self):
        velocities = list()
        for name in self.joints.keys():
            velocities.append(self.joints[name]['controller'].getJointVelocity())
        return velocities

    # This function is not used anywhere here...
    def setCocontraction(self, msg):
        print msg
        for i,name in enumerate(msg.name):
            self.joints[name]['effort'] = msg.effort[i]
        self.servoTo()
        #for name, effort in zip(msg.name, msg.effort):
        #    self.joints[name]['effort'] = effort
        # for msg in msgs:
        #     self.joints[msg.name]['effort'] = msg.effort


    # This function is not used anywhere here...
    def goTo(self, positions, now):
        if self.teleop == 0:
            self.shoulderYaw.goTo(positions[0], self.shoulderYawCocont, now)
            self.shoulderRoll.goTo(positions[1], self.shoulderRollCocont, now)
            self.shoulderPitch.goTo(positions[2], self.shoulderPitchCocont, now)

            self.upperarmRoll.servoTo(positions[3])

            self.elbow.goTo(positions[4], self.elbowCocont, now)
            self.forearmRoll.servoTo(positions[5])
            self.wrist.servoTo(positions[6])
            # self.wrist.goTo(positions[6], self.wristCocont, now)
            self.gripper.servoTo(positions[7])
            # self.handDOF1.servoTo(positions[7])

            self.publishJointState()
        else:
            rospy.logwarn("Asked to go to pose, but ignoring as in teleop mode. Check gummi.yaml file.")
