# aprilmav - Indoor navigation via Apriltags over MAVLink

This library is a work-in-progress to provide accurate (cm-level)
indoor navigation for MAVlink (APM, etc) vehicles via Apriltags (
developed by [AprilRobotics](https://april.eecs.umich.edu/))

The advantage of this method is that it does not require any expensive
or complicated equipment. All you need is:
- Printed April tags on a ~A4 paper mounted around the areas of travel
- Small embedded computer (Raspberry Pi or similar) with camera

This library uses the the [Apriltags Python bindings](https://github.com/duckietown/dt-apriltags) by Duckietown

## How to get started

Install OpenCV and Matplotlib: ``sudo apt install python3-matplotlib python3-opencv``.

Note for the Raspberry Pi, use ``pip install opencv-python`` instead.

Note for the ArduCam, use ``sudo apt install i2c-tools`` and ensure ``dtparam=i2c_vc=on`` is in ``/boot/config.txt``

Install the dt-apriltag, PyYAML and transforms3d libraries: ``pip install dt-apriltags transforms3d PyYAML pymavlink``.

If you are not using a Raspberry Pi Camera V2, you will need to calibrate your
camera:

### Note for the Raspberry Pi

In some cases, the default OpenCV for the Raspberry Pi may not process frames fast enough.

See https://qengineering.eu/install-opencv-4.4-on-raspberry-pi-4.html for building an optimised
version of OpenCV.


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

## Running

To send data to ArduPilot in realtime, use ``aprilmav.py``. See source code for the required arguments.

There are options for saving the captured images to file or a live video stream.

Note if you are using the ``--video`` option, you will likely require to build OpenCV from source, as most pre-built
packages do not include GStreamer support. See https://linuxize.com/post/how-to-install-opencv-on-ubuntu-20-04/ for
details.

To check if your OpenCV has GStreamer support, see https://learnopencv.com/get-opencv-build-information-getbuildinformation/

## Camera compatibility:

The following cameras have been tested as compatible with Aprilmav:

- Raspberry Pi Camera V2 (needs to be used in a very well lit room. Is not accurate in Apriltag pose detection)
- Arducam 1MP OV9281 (works quite well). Requires https://github.com/ArduCAM/MIPI_Camera/tree/master/RPI to be installed.

Cameras with global shutters are preferred, as they give much more accurate position solutions.

