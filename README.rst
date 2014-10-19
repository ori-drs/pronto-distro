======================
Pronto State Estimator
======================

.. contents:: Table of Contents

Introduction
============

Pronto is an efficient EKF state estimator for inertial and sensory
motion estimation.

Pronto has been used to localize a Fixed-wing micro aerial vehicles
and quadrotors aggressively flying indoors and outdoors. It has
also been used to provide a 333Hz motion estimate for the Boston Dynamics
Atlas humanoid. 

Pronto has been used with a variety of inputs 
from sensors such as IMUs (Microstrain and Kearfott), laser ranger finders, 
cameras and joint kinematics.

As well as the source code we also provide some data samples
to demonstrate the algorithm working on a fixed wing MAV and a
the Atlas robot walking.

.. image:: http://img.youtube.com/vi/V_DxB76MkE4/0.jpg
   :target: https://www.youtube.com/watch?v=V_DxB76MkE4

`Pronto youtube video <https://www.youtube.com/watch?v=V_DxB76MkE4>`_


Software Overview
-----------------
The algorithms are built primarily in C/C++. A number of the local dependencies
were developed for other robotics projects at MIT.

The software repository consists of two modules:

* externals: required modules such as Eigen, octomap and visualization tools
* pronto: the core estimator library (mav) and an adaptation to for humanoids (motion_estimate)

Building the Code
=================
These compile instructions were tested on a fresh Ubuntu 14.04 install, but is likely to work on earlier versions and MacOS.

Requirements:
-------------

Install these common system dependencies:

::

    apt-get install git build-essential libglib2.0-dev openjdk-6-jdk python-dev cmake gtk-doc-tools libgtkmm-2.4-dev  freeglut3-dev libjpeg-dev libtinyxml-dev libboost-thread-dev libgtk2.0-dev python-gtk2 mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev


Compiling
---------

Check out the source code using git:

::

    cd pronto-distro
    git submodule update --init --recursive

Then start compiling:

::

    make

The compile time is about 4 mins. 

Running Test Examples
=====================

Its not necessary but, we would suggest adding the binary path to your system path:

::

  export PATH=<path-to-your-code>/pronto-distro/build/bin:$PATH

To process a

::

  se-fusion -U model_LN_RN.urdf -P drc_robot_02_mit.cfg
  lcm-logplayer-gui <logfile>
  pronto_viewer
  bot-spy
  octomap-server octomap.bt

All the state estimation is done in se-fusion. It listens to messages published 
from the logplayer. pronto-viewer is a GTK GUI showing the sensor data and 
the position of the robot


blocks3-lcmlog-2014-04-21-18-40-robot-part
longstp-lcmlog-2014-04-21-16-12-robot-part
typical-lcmlog-2014-04-21-15-13-robot-part


se-fusion -U model_LN_RN.urdf -P drc_robot_02_mit.cfg -L ~/logs/pronto_release/blocks3-lcmlog-2014-04-21-18-40-robot-part -pr 0
se-fusion -U model_LN_RN.urdf -P drc_robot_02_mit.cfg -L ~/logs/pronto_release/longstp-lcmlog-2014-04-21-16-12-robot-part -pr 0


Documentation
=============

Technical details about the estimator are to be completed. Please read the attached publications for details.


Humanoid Locomotion
-------------------

Using the estimator with the Atlas Stepping Behaviour
-----------------------------------------------------

**Performance:** With inertial and kinematic input (i.e. no LIDAR input) the drift rate of the 
estimator is **2cm per 10 steps travelled**. We estimate this to be 10 times better 
than the estimator provided by BDI. With the closed-loop LIDAR module, drift is removed entirely.

More specifically, the estimator can walk the robot to the top of a tower of 
cinder blocks, under BDI control - without stopping --- with the only input being
the placement of footsteps. **Recently this was executed 8 times consecutively in a public demo.**

As the estimator was primarily developed for use on Atlas, performance has been heavily tested and 
is very robust. The easiest use case is with BDI retaining lower body control. 
To get started we suggest disabling the LIDAR module, for simplicity.

We estimate the position of the robot with the Pronto position estimator while the BDI estimate
is still used by their system. All operation of the robot is made using the Pronto estimator.

When a set of footsteps are placed near the feet of the Pronto position estimate, the relevant
Pronto-to-BDI transform is used to transmit footsteps to the BDI stepping system. As the robot
walks, only this Pronto-to-BDI transform is changed to ensure that the executed footsteps
truely hit the locations we have chosen.

**Getting Started:** To use the estimator on your robot, you simply need to provide
the required inputs to our system:

* ATLAS_STATE - contains the raw joint position, velocity information
* ATLAS_IMU_BATCH - the raw IMU data
* POSE_BDI - the position and orientation, as estimated by BDI
* STATE_EST_READY - a simple trigger to say where to initialize the robot - usually the origin

Pronto will output: 

* POSE_BODY - the position, orientation and velocity of the robot

Use this pose to render the robot in your system, and maintain the POSE_BDI to POSE_BODY estimate
so as to transform footsteps to the correct positions for the stepping controller.


Using the estimator with a third party controller
-------------------------------------------------

At MIT we use Pronto as our 333Hz Drake controller in a high-rate control loop. Latency
and relability have allowed us to demonstrate challenging motions with the Atlas robot.

If you are interested in using the estimator with your own controller, please get in touch.


About LCM:
----------

Currently Pronto uses LCM to received data and to publish output.

Lightweight Communications and Marshalling (LCM) is a tool for efficient multi-process 
message passing originally developed at MIT for the DARPA Urban Challenge.

To those familiar with ROS, it serves the same purpose as the message passing in ROS: messages are typed like C structures
and code is compiled to allow C/C++, python and Java bindings. Data is received in a process
via network communication and event-based function callbacks.

We provide a LCM-ROS translation bridge to allow easy integration with a ROS-based system.

If you are interested in a native ROS application, please get in touch.

Publications
============

* State Estimation for Aggressive Flight in GPS-Denied Environments Using Onboard Sensing, A. Bry, A. Bachrach, N. Roy, ICRA 2012.
* Drift-Free Humanoid State Estimation fusing Kinematic, Inertial and LIDAR sensing, M. Fallon, M. Antone, N. Roy, S. Teller. Humanoids 2014.

Credits
=======

Originally Developed by Adam Bry, Abe Bachrach and Nicholas Roy of 
the `MIT Robust Robotics Group <http://groups.csail.mit.edu/rrg/>`_.

Extended to support humanoid motion by Maurice Fallon with the help
of the `MIT DARPA Robotics Challenge Team <http://www.drc.mit.edu>`_.

Maurice Fallon. mfallon@mit.edu

