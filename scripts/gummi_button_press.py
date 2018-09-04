#!/usr/bin/env python

import rospy
import sys
import random
import time
from math import pi

from gummi_interface.gummi import Gummi

def main(args):
    gummi.setCocontraction(0.6, 0.6, 0.6, 0.6, 0.6)
    gummi.goTo((0.05,1.8,0.2792526803190927,-0.17453292519943295,0.4014257279586958,-2.3736477827122884,-0.7330382858376184), False)
    r.sleep()

if __name__ == '__main__':
    main(sys.argv)