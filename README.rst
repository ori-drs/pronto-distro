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
* pronto: the core estimator library (mav) and an adaptation for the Atlas robot (motion_estimate).

Building the Code
=================
These compile instructions were tested on a fresh Ubuntu 14.04 install, but is likely to work on other versions of Linux and MacOS.

Requirements:
-------------

Install these common system dependencies:

::

    apt-get install git build-essential libglib2.0-dev openjdk-6-jdk python-dev cmake gtk-doc-tools libgtkmm-2.4-dev  freeglut3-dev libjpeg-dev libtinyxml-dev libboost-thread-dev libgtk2.0-dev python-gtk2 mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev libqwt-dev


Compiling
---------

Check out the source code using git:

::

    git clone https://github.com/ipab-slmc/pronto-distro.git
    cd pronto-distro
    git submodule update --init --recursive

Then start compiling:

::

    make

The compile time is about 4 mins. 

Signal Scope
------------
Signal Scope is a lightweight signal plotting tool. Its invaluable for debugging 
systems such as Pronto. It was developed for the MIT DRC by Pat Marion.

Launch it by pointing it to a python config file in config/signal_scope. 
There are many examples of using it in signal_scope/examples.

Running Test Examples
=====================

.. image:: http://img.youtube.com/vi/OWrzUIH3kUA/0.jpg
   :target: https://www.youtube.com/watch?v=OWrzUIH3kUA

`Running pronto <https://www.youtube.com/watch?v=OWrzUIH3kUA>`_

Some test logs and maps can be downloaded from the following
location. (Atlas Version 5 Logs are preferred)

::

  http://homepages.inf.ed.ac.uk/mfallon2/share/pronto_logs/

To process a log run this process manager:

::

  bot-procman-sheriff -l drc_robot.pmd

Run the script called 'full' which launches all processes including
the estimator. Then run the logplayer tool to play the log-terrain log.


Some notes:

* All the state estimation is done in se-fusion. It listens to messages published 
  from the logplayer and produces POSE_BODY - the position and orientation of the robot's pelvis. 
* pronto-viewer is a GUI showing the sensor data and 
  the position of the robot.
* Make sure that POSE_BODY and STATE_ESTIMATOR_STATE are disabled 
  (they were the position generated during the actual experiment)
* bot-spy is a tool for inspecting the messages.


Laser localization with Gaussian Particle Filter
================================================

In the above the Laser localization module is not running.
You can view the octomap that's being localized against using octomap-server:

::

  octomap-server octomap.bt

There are two other logs that work in the same way:

* longstp-lcmlog-2014-04-21-16-12-robot-part
* blocks3-lcmlog-2014-04-21-18-40-robot-part. TODO: I need a different map for this log.
* NEW 2015: Switch atlas_v3/atlas_v4/atlas_v5 for different Atlas version numbers

Options
-------

All options are read from the cfg file located in pronto-distro slash config. 

* By default, this demos initalizes using vicon data in the log via "init_sensors"
* The Gaussian Particle Filter is disabled by removing it from "active_sensors".
* Its not necessary but, we would suggest adding the binary path to your system path:

::

  export PATH=<path-to-your-code>/pronto-distro/build/bin:$PATH


Documentation
=============

Technical details about the estimator are to be completed. Please read the attached publications for details
or get in touch for support.

Humanoid Locomotion
-------------------

Having tried out the test examples. How can you use Pronto with your robot?

First of all, pronto can be used as an module within your system without any changes. It
simply produces a better state estimator - enabling more rapid walking.

Using the estimator with the Atlas Stepping Behaviour
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Performance:** With inertial and kinematic input (i.e. no LIDAR input) the drift rate of the 
estimator is **2cm per 10 steps travelled**. We estimate this to be 10 times better 
than the estimator provided by BDI. With the closed-loop LIDAR module, drift is removed entirely.

More specifically, the estimator can walk the robot to the top of a tower of 
cinder blocks, under BDI control - without stopping --- with the only input being
the placement of footsteps. **Recently this was executed 8 times consecutively in a public demo.**

As the estimator was primarily developed for use on Atlas, performance has been heavily tested and 
is robust. The easiest use case is with BDI retaining lower body control. 
To get started we suggest disabling the LIDAR module, for simplicity.

We estimate the position of the robot with the Pronto position estimator while the BDI estimate
is still used by their system.

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

* POSE_BODY - the position, orientation and velocity of the robot's pelvis

Use this pose to render the robot in your system, and maintain the relative POSE_BDI-to-POSE_BODY estimate
so as to transform footsteps to the correct positions for the stepping controller.

Using the Estimator With ROS
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We provide a LCM-to-ROS translation bridge to allow easy integration with a ROS-based system.
On ROS Indigo the follow contents should be added to bashrc: 

::

  export PATH=/home/drc/pronto-distro/build/bin:$PATH
  source /opt/ros/indigo/setup.bash
  export PKG_CONFIG_PATH=<your-path-to>/pronto-distro/build/lib/pkgconfig/:<insert-path-to>/pronto-distro/build/lib64/pkgconfig/:$PKG_CONFIG_PATH
  export LD_LIBRARY_PATH=<your-path-to>/pronto-distro/build/lib/:<insert-path-to>/pronto-distro/build/lib64/:$LD_LIBRARY_PATH
  export DRC_BASE=<your-path-to>/pronto-distro

This is a super set, not all of these are required. The package can then be compiled using catkin:

::

  cd <insert-path-to>/pronto-distro/pronto-lcm-ros-translators
  catkin_make
  source <insert-path-to>/pronto-distro/pronto-lcm-ros-translators/devel/setup.bash

And then a translators can be run in each direction:

::
  
  rosrun pronto_translators ros2lcm
  rosrun pronto_translators lcm2ros

You can test this:

* Play back a ROS bag, traffic can be see with the bot-spy tool
* Play back the logs mentioned above and some of the channels can be seen with rostopic

Tested on Ubuntu 14.04 with ROS Indigo.

Modifying the Translator for your system
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
I have provided a skeleton translator which I assume you will need
to modify to use in your system. Get in touch if you would like some help in doing this. These are the required messages:
(to be confirmed if this is exhausive)

BDI's estimate of the Atlas position:

* Source: BDI driver  (pos_est, filtered_imu fields)
* Publish: POSE_BDI (bot_core_pose_t)

The IMU measurements:

* Source: BDI driver (the raw_imu field)
* Publish: ATLAS_IMU_BATCH (atlas_raw_imu_batch_t)

BDI's joint angle velocities, positions and efforts. Also the FT sensors

* Source: BDI driver (jfeed, foot_sensors, wrist_sensors)
* Publish: ATLAS_STATE (atlas_state_t)
* Wrist sensors not used

Ancillary data message from BDI (e.g. pump rpm, air sump pressure)

* Source: BDI driver
* Publish: ATLAS_STATUS (10Hz is fine)
* TODO: revamp this, as I only need the current_behavior field (to distinguish walking and standing)

The Multisense Lidar Scan:

* Source: Multisense driver
* Publish: SCAN (bot_core_planar_lidar_t)

Angle of the Multisense SL Laser:

* Source: both spindleAngleStart and spindleAngleEnd in CRL's lidar header
* Publish: PRE_SPINDLE_TO_POST_SPINDLE (bot_core_rigid_transform_t)

Message to tell SE where in the world to start

* Source: The user: I always use a point above the origin - (0,0,0.85)
* Publish: MAV_STATE_EST_VIEWER_MEASUREMENT (mav_indexed_measurement_t)
* Publish: STATE_EST_READY  (a timestamp)

Simple timestamp messages - used to provide commands:

* STATE_EST_RESTART
* STATE_EST_START_NEW_MAP


Using the estimator with a third party controller
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

At MIT we use Pronto as our 333Hz Drake controller in a high-rate control loop. Latency
and relability have allowed us to demonstrate challenging locomotion using the Atlas robot.

If you are interested in using the estimator with your own controller, please get in touch.

Micro Aerial Vehicles
---------------------

Pronto was originally developed for Micro Aerial Vehicle state estimation.

.. image:: http://img.youtube.com/vi/kYs215TgI7c/0.jpg
   :target: https://www.youtube.com/watch?v=kYs215TgI7c

`Micro aerial vehicle estimation using Pronto <https://www.youtube.com/watch?v=kYs215TgI7c>`_

Log files demonstrating flight with Quadrotators and Fixed-wing RC Planes can
be provided on request.

Supported sensor of interest to aerial flight:

* GPS - x, y, z
* Vicon - x, y, z and orientation
* Laser Scanmatcher - x, y, z and yaw or velocity and yaw rate
* Optical Flow - velocity, yaw rate (downward facing camera)
* Airspeed - forward velocity
* Altimeter - z
* Sideslip - lateral velocity

And example configuration for these sensors is in docs/aerial_sensors_example.cfg

About LCM
=========

Currently Pronto uses LCM to receive data and to publish output.

Lightweight Communications and Marshalling (LCM) is a tool for efficient multi-process 
message passing originally developed at MIT for the DARPA Urban Challenge.

To those familiar with ROS, it serves the same purpose as the message passing in ROS: messages are typed data structures
and code is compiled to allow C/C++, python and Java bindings. Data is received in a process
via network communication and event-based function callbacks.

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

Additional contributions from:

* Andy Barry
* Pat Marion

The License information is available in the LICENSE file attached to this document.

Maurice Fallon, Feb 2015. maurice.fallon@ed.ac.uk

