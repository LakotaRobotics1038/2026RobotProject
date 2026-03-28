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

![Yaw, Pitch, and Roll](https://external-content.duckduckgo.com/iu/?u=http%3A%2F%2Fwww.crazepony.com%2Fassets%2Fimg%2Fpitch-yaw-roll.png&f=1&nofb=1&ipt=9ed5966667e8938c18ac0feefb77c9cf00316dcf2c4cda1cd600b0cf9fa03d47)
