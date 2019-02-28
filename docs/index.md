<a href="#package layout">Click here to learn about the file layout in the package</a>
------

# Introduction
Gummi_interface is where all of the core functions and algorithms that control the gummi arm are stored. This package is one of many packages required to run the gummi arm.

The packages are as follows:
- [gummi_base_luffy](https://nortonkellyboxall.github.io/gummi_base_luffy/)
- [gummi_ee_luffy](https://nortonkellyboxall.github.io/gummi_ee_luffy/)
- [gummi_head_twodof](https://nortonkellyboxall.github.io/gummi_head_twodof/)
- [gummi_interface](https://nortonkellyboxall.github.io/gummi_interface/)
- [gummi_moveit](https://github.com/nortonkellyboxall/gummi_moveit)
- [gummi_hardware_luffy](https://nortonkellyboxall.github.io/gummi_hardware_Luffy/)

Each of these packages are connected and required to be cloned or forked.

<img src="images/Gummi_Forearm.png" alt="Gummi Forearm"/>

<a id = "package layout"> Package layout </a>
======

## Launch
This folder contains the launch files for both starting up the gummi arm and also to launch the various algorithms to test and debug the arm. 

### Pre-Running Launch 
The configure.launch file should be run before the arm is started up. It is part of the construction process but it is also useful for debugging purposes if the joints arent responding properly. This launch file will set all of the motors to their zero position (NOTE: This is not the joint zero position necessarily, however you want it to be close). This should put the arm in a position close to zero on the joints, if it isnt, or if you want the tendons to be in higher tension, then detach the pulley for that specific motor and reattach it at a more tensioned point. The usefulness of this launch file can be better seen [here](https://github.com/mstoelen/GummiArm/wiki/Mount-agonist-antagonist-tendon-pulleys).

### Start Running Launch
The gummi.launch file will start up the arm and run it through its required routine. This is handled in the all.launch discussed [here](https://nortonkellyboxall.github.io/gummi_base_luffy/#startup).

### Post Running Launch
The rest of the launch files are either things you can do with the gummi arm like the hand_shake.launch (not running on Luffy) or for debugging like test_antagonist.launch which will give you the ability to test each antagonist joint through its full stiffness and position range.

### Scripts
This folder holds all of the scripts written in python that the launch files reference. This is the place you want to go if you want to see how they did something.

### Src
This is where all of the low level control happens and where the Gummi class is located. This holds all of the definitions for controlling the arm under the equilibirum model for each of the antagonist joints etc.

# Getting a Deeper Understanding of the Package

##Start up Script
The following section will explore the start up script used by the launch file. Firstly the script imports the necessary packages and classes for operation.
``` python
#!/usr/bin/env python

import rospy
import sys
import random
import time
from math import pi
from gummi_interface.gummi import Gummi

```
This section first initialises the gummi arm node, anonymous is set to false so that only one gummi arm can be running at once. Next the rate is set, this is set quite low at the moment since daisy chaining the motors together means their read/write is slow and so this prevents overshooting of the controller. If you want to increase it, you will need to adjust the PID controllers for each joint. The gummi class is then called and a start up regime is run, this startup moves all joints to their zero position in an open loop manner. 

``` python
rospy.init_node('GummiArm', anonymous=False)
r = rospy.Rate(40)
gummi = Gummi()
rospy.logwarn('Moving joints sequentially to startup equilibrium positions.')
gummi.doGradualStartup()
```
Once the arm has completed its gradual startup it will wait a second and then go to its resting pose. This is a closed loop move to the zero positions. Each joint will move simultaneously there unlike the gradual startup which moves one joint at a time. Once it is there ADD ONCE YOU KNOW ABOUT COLLISION RESPONSE
``` python
rospy.logwarn('Moving to resting pose, hold arm!')
rospy.sleep(1)

gummi.goRestingPose(True)
for i in range(0,400):
    gummi.goRestingPose(False)
    r.sleep()

gummi.setCollisionResponses(shoulder_yaw=False, shoulder_roll=False, shoulder_pitch=False, elbow=False)
rospy.loginfo("GummiArm is live!")
```
Once it is in its resting pose the arm will constantly verify that the dynamixel manager is still running and it publishes the joint states at the rate set at the beginning. 
```python
while not rospy.is_shutdown():
    try:
        rospy.has_param("/dynamixel_manager_arm/namespace")
        # rospy.has_param("/dynamixel_manager_arm/namespace") is a way to verify if the manager is running

        if gummi.teleop == 0 and gummi.velocity_control == 0:
            gummi.doUpdate()

        gummi.publishJointState()
        r.sleep()
    except:
        rospy.signal_shutdown("gummi dynamixel manager seems to be dead... exiting!")
```

## Gummi Class
This class defines all of the functionality currently available in the Gummi Arm. I will do my best to bring light to what they are and how they work. First the required packages are imported. The JointState message is the message that dynamixel use for their motor positions. The antagonist and DirectDrive classes are lower level classes that deal with the types of joints present in the gummi arm.

``` python
#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import JointState

from antagonist import Antagonist
from direct_drive import DirectDrive
```
The next part of the class initialises everything for the arm. Firstly the variables are initialised, this section ensures mostly that the joint state message is configured correctly. Next the joints are initialised, this lets the system know which controller to use for each joint. Finally the publishers and subscribers are initialises so that the joints can be controlled.

``` python
class Gummi:

    def __init__(self):
        self.initVariables()
        self.initJoints()
        self.initPublishers()
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
```
Next the callback is defined for the gummi/joint_commands topic. In this, first the joint is updated with the new position, velocity and efforts as commanded. Then unless you are in teleop or velocity control, the arm will then either stay still if the position or effort does not change or it will closed loop control to the desired position and effort.

``` python
def cmdCallback(self, msg):
    for name, position, velocity, effort in zip(msg.name, msg.position, msg.velocity, msg.effort):
        self.joints[name]['position'] = position
        self.joints[name]['velocity'] = velocity
        self.joints[name]['effort'] = effort

    if self.teleop == 1 or self.velocity_control == 1:
        self.doVelocityUpdate()
    else:
        if sum([effort >=0 for effort in msg.effort]):
            self.servoTo()
        else:
            self.passiveHold()
```

The servoTo function is one of the most used functions in the class, this function is called when you publish in gummi/joint_commands. All it does is check which controller is used for each joint and then moves them to their respective position using the specific controller required.
``` python
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
```
Since there are two joint types in the Gummi Arm there are two lower level controllers that are used when commanding the gummi arm. If the joint is antagonistic then it uses the Antagonist class in antagonist.py and if it is a direct drive it uses the DirectDrive class in direct_drive.py. The latter is a much simpler class since the joint position is the motor position, however there is a little more complexity within the Antagonist class since the position and stiffness of the joint are controlled by two motors.




