======================
Pronto State Estimator
======================

.. image:: https://travis-ci.org/ipab-slmc/pronto-distro.svg?branch=master
    :target: https://travis-ci.org/ipab-slmc/pronto-distro


.. contents:: Table of Contents

Introduction
============

Pronto is an efficient EKF state estimator for inertial and sensory
motion estimation. It provided the state estimate that was used by MIT DRC team in the DARPA Robotics Challenge to estimate the position and motion of the Boston Dynamics Atlas robot.

**Performance:** With inertial and kinematic input (i.e. no LIDAR input) the drift rate of the 
estimator is **2cm per 10 steps travelled**. We estimate this to be 10 times better 
than the estimator provided by Boston Dynamics. With the closed-loop LIDAR module, drift is removed entirely.

It has since been adapted to estimate the motion of the NASA Valkyrie robot at University of Edinburgh - just as reliably. As well as the source code we also provide some data samples
to demonstrate the algorithm working with both of these two humanoid robots walking and manipulating.

Pronto has been used with a variety of inputs 
from sensors such as IMUs (Microstrain and Kearfott), laser ranger finders, 
cameras and joint kinematics.


.. image:: http://img.youtube.com/vi/V_DxB76MkE4/0.jpg
   :target: https://www.youtube.com/watch?v=V_DxB76MkE4

`Pronto youtube video <https://www.youtube.com/watch?v=V_DxB76MkE4>`_


Software Overview
-----------------
The algorithms are built primarily in C/C++. A number of the local dependencies
were developed for our robotics projects. The software repository consists of two main modules:

* externals: required modules such as Eigen, octomap and visualization tools
* pronto: the core estimator library.

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
systems such as Pronto. It was developed for the MIT DRC team by Pat Marion.

Launch it by pointing it to a python config file in config/signal_scope. 
There are many examples of using it in signal_scope/examples.

Running Test Examples
=====================

Test LCM logs for both BDI Atlas (v5) and NASA Valkyrie can be downloaded from the following
location. Also included are videos showing the output of the estimator.

::

  http://homepages.inf.ed.ac.uk/mfallon2/share/pronto_logs/

The following commands will process the respective log files.

NASA Valkyrie Logs
------------------

::

  se-fusion -P val/robot.cfg -U val_description/urdf/valkyrie_sim.urdf  -L raluca-turning-180deg-snippet.lcmlog
  robot_model_publisher val_description/urdf/valkyrie_sim.urdf 
  se-state-sync-simple
  pronto-viewer -val/robot.cfg


Boston Dynamics Atlas Logs
--------------------------

::

  se-fusion -P atlas/robot.cfg -U atlas_v5/model_LR_RR.urdf  -L 20160315-walking.lcmlog
  robot_model_publisher model_LR_RR.urdf
  se-state-sync-simple
  pronto-viewer -val/robot.cfg


Some notes:

* All the state estimation is done in se-fusion. It listens to messages published 
  from the log and produces POSE_BODY - the position and orientation of the robot's pelvis. 
* pronto-viewer is a GUI showing the sensor data and 
  the position of the robot.
* Make sure that POSE_BODY and STATE_ESTIMATOR_STATE are disabled 
  (they were the position generated during the actual experiment)
* bot-spy is a tool for inspecting the messages.
* lcm-logplayer-gui is a gui based tool for playing back lcm logs (surprise!), we use it a lot to simulate live receipt of data. The logs can also be processed by playing back the logs from the tool.


Using with your own robot
-------------------------

Having tried out the test examples. How can you use Pronto with your robot?

**Getting Started:** To use the estimator on your robot, you simply need to provide
the required inputs to our system:

* IMU measurements of type ins_t.lcm (ROS: sensor_msgs/Imu)
  * Also support the KVH 1750 IMU which is in the Atlas
* Joint States of type joint_states_t.lcm (ROS: sensor_msgs/JointState)
* Force Torque sensor of type six_axis_force_torque_array_t.lcm (ROS: geometry_msgs/WrenchStamped)

Pronto will output: 

* POSE_BODY - the position, orientation and velocity of the robot's pelvis

Using the Estimator With ROS
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

I have provided a skeleton translator which I assume you will need
to modify to use in your system. Get in touch if you would like some help in doing this.

On ROS Indigo the follow contents should be added to bashrc: 

::

  export PATH=/home/drc/pronto-distro/build/bin:$PATH
  source /opt/ros/indigo/setup.bash
  export PKG_CONFIG_PATH=<your-path-to>/pronto-distro/build/lib/pkgconfig/:<insert-path-to>/pronto-distro/build/lib64/pkgconfig/:$PKG_CONFIG_PATH
  export LD_LIBRARY_PATH=<your-path-to>/pronto-distro/build/lib/:<insert-path-to>/pronto-distro/build/lib64/:$LD_LIBRARY_PATH
  export DRC_BASE=<your-path-to>/pronto-distro

The package can then be compiled using catkin:

::

  cd <insert-path-to>/pronto-distro/pronto-lcm-ros-translators
  catkin_make
  source <insert-path-to>/pronto-distro/pronto-lcm-ros-translators/devel/setup.bash

And then a translators can be run in each direction:

::
  
  rosrun pronto_translators ros2lcm
  rosrun pronto_translators lcm2ros

Tested on Ubuntu 14.04 with ROS Indigo.

Using the estimator with a third party controller
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We have successfully used Pronto with 4 other bipeds (including NASA Valkyrie) and a quadruped. If you are interested in using the estimator with your own controller, please get in touch.

At MIT and Edinburgh we use Pronto as our 333Hz Drake controller in a high-rate control loop. Latency
and relability have allowed us to demonstrate challenging locomotion using the Atlas robot.


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

To those familiar with ROS, it serves the same purpose as the message passing in ROS: messages are typed data structures and code is compiled to allow C/C++, python and Java bindings. Data is received in a process
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

