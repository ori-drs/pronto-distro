======================
Pronto State Estimator
======================

.. contents:: Table of Contents

Quick Start
===========

To checkout submodules, and build the code.

::
    cd pronto-distro
    git submodule update --init --recursive
    make

The compile time is about 4 mins


Introduction
============

Pronto is an efficient EKF state estimator for inertial and sensory
motion estimation.

Pronto has been used to localize a Fixed-wing micro aerial vehicles
and quadrotors aggressively flying indoors and outdoors as well as
the Boston Dynamics Atlas humanoid. It has been used with inputs 
from a variety of sensors e.g. IMUs, laser ranger finders, cameras,
kinematics.

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
This repository is supported on Ubuntu 14.04. It was also recently used
on 12.04.

Requirements:
-------------

TODO: Verify from a fresh system

Compiling
---------
Make sure you have checked out the git submodules:

::

    cd pronto-distro
    git submodule update --init --recursive

The start compiling

::

    make

The compile time is about 4 mins.


Running Test Examples
=====================

Its not necessary but, I would suggest adding the repo to your repository:

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


About LCM
=========

Lightweight Communications and Marshalling (LCM) is a tool for comm
The 

To those familiar with ROS, it serves the same as the message passing in ROS: messages are typed like C structures
and code is compiled to allow C/C++, python and Java bindings. Data is received in a process
via network communication and event based function callbacks.

We will provide a LCM-ROS translation bridge: to allow easy integration with
Then we will provide native LCM 

Credits
=======

Originally Developed by Adam Bry, Abe Bachrach and Nicholas Roy.

Extended to support humanoid motion by Maurice Fallon with the help
of the `MIT DARPA Robotics Challenge Team <http://www.drc.mit.edu>`_.
