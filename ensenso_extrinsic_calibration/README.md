 [![Institut Maupertuis logo](https://avatars1.githubusercontent.com/u/12760694?v=3&s=80)](http://www.institutmaupertuis.fr) Ensenso extrinsic calibration
=============================

# Travis CI

[![Travis-CI](https://api.travis-ci.org/InstitutMaupertuis/ensenso_extrinsic_calibration.svg?branch=indigo-devel)](https://travis-ci.org/InstitutMaupertuis/ensenso_extrinsic_calibration/branches) 

# General information

This is a ROS package of a Fanuc R1000iA 80f with a grinding end effector and a 3D sensor mounted on the end effector.
This package allows to automatically calibrate the Ensenso on the robot by moving around a target.

<img src="https://raw.githubusercontent.com/InstitutMaupertuis/ensenso_extrinsic_calibration/indigo-devel/documentation/01.png" align="center" height="300">

[This video](https://youtu.be/2g6gdx8fKX8) shows the calibration on the real robot.
The [tutorial](documentation/tutorial.md) folder contains explanations about how to use this package.

Directories in the project
--------------------------

| Directory  | Description
------------ | -----------
`documentation` | Contains the documentation / tutorial
`ensenso_extrinsic_calibration` |  Contains the meta-package files
`ensenso_rviz_plugin` | Contains the definition of an RViz panel
`grinding_ensenso_extrinsic_calibration` | Contains a node and a service definition for the calibration

Dependencies
------------
- [Robot Operating System](http://wiki.ros.org/ROS/Installation)
- [`industrial-core`](http://wiki.ros.org/industrial_core)
- [`fanuc`](https://github.com/InstitutMaupertuis/fanuc.git) :warning: Joint limits have been tweaked
- [`fanuc_experimental`](https://github.com/InstitutMaupertuis/fanuc_experimental.git) :warning: Joint limits have been tweaked
- [`universal_robot`](https://github.com/ros-industrial/universal_robot) :warning: Use the source repository instead of the release
- [`institut_maupertuis_robots_descriptions`](https://github.com/InstitutMaupertuis/institut_maupertuis_robots_descriptions)

This package has been tested with Ubuntu 14.04 and ROS Indigo.
The package was designed to be used with a Fanuc R1000iA 80f robot however it should be easy to port it on other ROS compatible robots; refer to the [last section](https://github.com/InstitutMaupertuis/ensenso_extrinsic_calibration/blob/indigo-devel/README.md#changing-the-setup).

Install
-------
Install the dependencies by following the wiki instructions and cloning the repositories into your catkin workspace.

`cd` to your catkin workspace source directory:
```
git clone https://github.com/InstitutMaupertuis/ensenso_extrinsic_calibration.git &&\
cd .. &&\
catkin_make
```

Generating a target
-------------------
If you didn't purchase a calibration target you can generate one and print it.
Use the `nxCalTab` program to generate a PDF and print it, make sure the scale is right!

Launching
---------
Refer to the [tutorial](documentation/tutorial.md).

Changing the setup
------------------
If you want to use this package with an other robot, an other end effector or an other camera setup (eg: camera is fixed) this is what you should care about:

- Change the [`calibration.launch`](https://github.com/InstitutMaupertuis/ensenso_extrinsic_calibration/blob/indigo-devel/grinding_ensenso_extrinsic_calibration/launch/calibration.launch) file to match your robot definition.
- Define the `ensenso_n10` manipulator in your MoveIt package and `ensenso_n10_tcp` frame, another option is to change them in the [node](https://github.com/InstitutMaupertuis/ensenso_extrinsic_calibration/blob/indigo-devel/grinding_ensenso_extrinsic_calibration/src/ensenso_extrinsic_calibration.cpp#L29-L34).
- Change the calibration parameters in the [node](https://github.com/InstitutMaupertuis/ensenso_extrinsic_calibration/blob/indigo-devel/grinding_ensenso_extrinsic_calibration/src/ensenso_extrinsic_calibration.cpp#L304) (eg: fixed instead of moving)
