# aprilmav - Indoor navigation via Apriltags over MAVLink

This library is a work-in-progress to provide accurate (cm-level)
indoor navigation for MAVlink (APM, etc) vehicles via Apriltags (
developed by [AprilRobotics](https://april.eecs.umich.edu/))

The advantage of this method is that it does not require any expensive
or complicated equipment. All you need is:
- Printed April tags on a ~A4 paper mounted around the areas of travel
- Small embedded computer (Raspberry Pi + PiCam V2) with camera

This library uses the the [Apriltags Python bindings](https://github.com/duckietown/dt-apriltags) by Duckietown

## How to get started

Install OpenCV and Matplotlib: ``sudo apt install python3-matplotlib python3-opencv``.

Note for the Raspberry Pi, use ``pip install opencv-python`` instead.

Install the dt-apriltag, PyYAML and transforms3d libraries: ``pip install dt-apriltags transforms3d PyYAML pymavlink``.

If you are not using a Raspberry Pi Camera V2, you will need to calibrate your
camera:

```
$ CameraCal.py
```

## Performance Testing

There are several scripts for testing the performance - both accuracy and
detected speed:

Speed of camera capture. Useful for confirming if the camera itself is a system
bottleneck. Should be less than 0.1 sec in general
```
$ capture_test.py
```

Speed of camera capture and Apriltag detection. Useful for confirming the camera
calibration via the Apriltag distances from the camera:
```
$ process_test.py
```

Localisation test. Estimates the vehicles current position based on Apriltags. Useful
for confirming the camera calibration.:
```
$ geo_test.py
```
[Use the --gui option to get a plot of the tag and vehicle locations]
