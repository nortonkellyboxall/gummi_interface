#!/usr/bin/env python

from math import pi
import rospy
import numpy as np

from direct_drive import DirectDrive
from helpers import fetchParam

class EquilibriumModel:
    def __init__(self, name):
        self.name = name

        # retrives loadLimit parameter from
        self.loadLimit = rospy.get_param("~" + self.name + "/equilibrium/loadLimit")

        self.sign = rospy.get_param("~" + self.name + "/equilibrium/sign")
        self.signFlexor = rospy.get_param("~" + self.name + "/equilibrium/signFlexor")
        self.signExtensor = rospy.get_param("~" + self.name + "/equilibrium/signExtensor")
        self.nameFlexor = rospy.get_param("~" + self.name + "/equilibrium/nameFlexor")
        self.nameExtensor = rospy.get_param("~" + self.name + "/equilibrium/nameExtensor")
        self.servoRange = rospy.get_param("~" + self.name + "/equilibrium/servoRange")
        self.servoOffset = rospy.get_param("~" + self.name + "/equilibrium/servoOffset")

        self.flexor = DirectDrive(self.nameFlexor, 1000)
        self.extensor = DirectDrive(self.nameExtensor, 1000)

        self.initVariables()
        self.flexor.setTorqueLimit(1)
        self.extensor.setTorqueLimit(1)

    def initVariables(self):
        self.commandFlexor = 0
        self.commandExtensor = 0
        self.dEquilibrium = 0

        self.dCocontraction = 0
        self.cCocontraction = 0

        self.maxCocontraction = 1.0
        self.dEqVelCalibration = 1.0

    def getEquilibriumForAlphas(self):
        return (self.extensor.getJointAngle()*self.signExtensor - self.flexor.getJointAngle()*self.signFlexor)*2/self.servoRange

    def getCocontractionForAlphas(self):
        return (self.extensor.getJointAngle()*self.signExtensor + self.flexor.getJointAngle()*self.signFlexor)/pi

    def createCommand(self):
        equilibrium = self.dEquilibrium
        cocontraction = self.cCocontraction

        self.commandFlexor = self.signFlexor * ((-equilibrium*self.servoRange/4.0 + cocontraction*pi/2.0) + self.servoOffset)
        self.commandExtensor = self.signExtensor * ((equilibrium*self.servoRange/4.0 + cocontraction*pi/2.0) + self.servoOffset)

    def capCocontraction(self):
        if self.cCocontraction > self.maxCocontraction:
            self.cCocontraction = self.maxCocontraction
        else:
            if self.cCocontraction < 0:
                self.cCocontraction = 0.0

    def publishCommand(self):
        self.flexor.servoTo(self.commandFlexor)
        self.extensor.servoTo(self.commandExtensor)

    def doEquilibriumIncrement(self, vel):
        self.dEquilibrium = self.dEquilibrium + vel * self.sign * self.dEqVelCalibration;

    def calculateEqVelCalibration(self, jointRange):
        eqRange = 2 * 2.0
        self.dEqVelCalibration = eqRange/jointRange;
        print("Equilibrium to joint velocity calibration: " + str(self.dEqVelCalibration) + ".")

    def getDesiredEquilibrium(self):
        return self.dEquilibrium

    def getCommandedCocontraction(self):
        return self.cCocontraction

    # Prevents motors from overloading ("limit" value is passed in antagonist.py)
    def limitLoad(self):

        overloadDone = False

        try:

            # Reduce co-contraction and move equilibrium position when motor approaching loadLimit.
            # Turn off torque once loadLimit is reached
            # (BUG: KEEP LOAD-LIMIT BELOW ACTUAL MOTOR STALL CURRENT TO PREVENT DAMAGE TO MOTOR).
             if (self.flexor.isOverloaded(self.loadLimit - 0.1) or self.extensor.isOverloaded(self.loadLimit - 0.1) ) and self.cCocontraction > 0.0:

                 self.cCocontraction -= 0.1
                 self.doEquilibriumIncrement(-0.002)

                 if self.flexor.isOverloaded(self.loadLimit):
                     self.flexor.setTorqueLimit(0.0)

                 if self.extensor.isOverloaded(self.loadLimit):
                     self.extensor.setTorqueLimit(0.0)

                 # Apply overload-response changes.
                 self.createCommand()
                 self.publishCommand()

                 overloadDone = True


             if overloadDone:

                 # Do not change equilibrium position/ cocontraction when safely below load limit.
                 # is torqueLimit proportional to cocontraction?

                 self.flexor.setTorqueLimit(self.cCocontraction)
                 self.extensor.setTorqueLimit(self.cCocontraction)



             print self.cCocontraction

        except rospy.ServiceException, e:
            print "Could not limit load: %s"%e
