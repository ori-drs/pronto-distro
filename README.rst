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

Credits
=======

Originally Developed by Adam Bry, Abe Bachrach and Nicholas Roy.

Extended to support humanoid motion by Maurice Fallon with the help
of the `MIT DARPA Robotics Challenge Team <http://www.drc.mit.edu>`_.
