# Vision Setup

To calibrate cameras: <https://docs.photonvision.org/en/latest/docs/quick-start/camera-calibration.html>
Focus the camera: <https://docs.photonvision.org/en/latest/docs/quick-start/camera-focusing.html>
Configuration: <https://docs.photonvision.org/en/latest/docs/quick-start/quick-configure.html>

For the code:

* Camera offset is from x and y of center of the robot and from off the ground for z.
* Positive x is the front of the robot.
* Positive y is the left of the robot.
* Rotation is measured in radians.
* The constructor accepts roll, pitch, and yaw.

```
        +X
         ^
         |
         |
         |
    -----+-----> -Y
         |
         |
         |
```

* Roll: Rotation around the X-axis. Positive roll rotates the camera counter-clockwise (tilting it to the left).
* Pitch: Rotation around the Y-axis. Positive pitch rotates the camera counter-clockwise (tilting it downwards).
* Yaw: Rotation around the Z-axis. Positive yaw rotates the camera counter-clockwise (turning it to the left).