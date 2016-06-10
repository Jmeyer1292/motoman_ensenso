# Institut Maupertuis Ensenso extrinsic calibration

Before starting
---------------
You need:
- A robot (here a Fanuc R1000iA 80f).
- An Ensenso camera mounted on the robot flange.
- A calibration plate (use `nxCalTab` to print your own if you don't have one).

ROS must be setup and running on the robot and the computer hosting the Ensenso camera.

Launching
----------
Place the calibration plate into the robot work-cell, move the robot so that the Ensenso points vertically downwards the calibration plate.

```
Running in Auto mode means only the e-stop (on the TP or on the controller cabinet) can be used to stop the robot in an emergency. Always adhere to safety regulations and take proper precautions when working with the real robot.
```

Tweak the following line depending on your robot IP:
`roslaunch grinding_ensenso_extrinsic_calibration calibration.launch robot_ip:=192.168.100.200 sim:=false`

You should get this: (note the initial position of the robot, the Ensenso is vertical to the calibration plate)
<img src="https://raw.githubusercontent.com/InstitutMaupertuis/ensenso_extrinsic_calibration/indigo-devel/documentation/01.png" align="center" height="300">

- `Number of poses`: The quantity of data to acquire for the calibration, the more the better, 12 is often enough.
- `Pattern grid spacing`: The space between the dots on the calibration plate.
- `Calibration pattern distance`: The distance between the calibration plate and the Ensenso, tweak this to move closer or further (should be close to the camera minimum object distance).
- `Store calibration matrix to EEPROM`: Permanently store the calibration matrix.
- Hit the button `Erase calibration (EEPROM)` to reset the calibration.
- Hit the button `Start calibration` to start the procedure, the robot will start moving around the calibration plate.

<img src="https://raw.githubusercontent.com/InstitutMaupertuis/ensenso_extrinsic_calibration/indigo-devel/documentation/02.png" align="center" height="300">
<img src="https://raw.githubusercontent.com/InstitutMaupertuis/ensenso_extrinsic_calibration/indigo-devel/documentation/03.png" align="center" height="300">

The `Status` label informs the user about the progression of the calibration, the computation step should last less than 20 seconds. If it runs more verify your setup (grid spacing ...), there could be something wrong.

When the calibration has run successfully you can close the application.
The point clouds fetched from the device are automatically transformed into the `tool0` frame; this means you don't have to apply any calibration matrix when capturing point clouds from a calibrated Ensenso device.

Testing the calibration
-----------------------
It is always a good idea to test if the calibration results are good, the `Test calibration` button allows to visually test the calibration by capturing several point clouds at different poses.
The result is displayed into RViz as a point cloud, each point cloud acquired has a unique color. If the point clouds are overlapping the calibration was successful:

<img src="https://raw.githubusercontent.com/InstitutMaupertuis/ensenso_extrinsic_calibration/indigo-devel/documentation/04.png" align="center" height="300">

Don't hesitate to put an object on the calibration plate for testing!
