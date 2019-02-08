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
- [gummi_hardware_luffy](https://nortonkellyboxall.github.io/gummi_hardware_luffy/)

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
COMING SOON 


