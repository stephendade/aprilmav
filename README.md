# aprilmav - Indoor navigation via Apriltags over MAVLink

This library is a work-in-progress to provide accurate (cm-level)
indoor navigation for MAVlink (APM, etc) vehicles via Apriltags (
developed by [AprilRobotics](https://april.eecs.umich.edu/))

The advantage of this method is that it does not require any expensive
or complicated equipment. All you need is:
- Printed April tags on a ~A4 paper mounted around the areas of travel
- Small embedded computer (Raspberry Pi or similar) with camera (Global shutter preferred)

This library uses the the [Apriltags Python bindings](https://github.com/duckietown/dt-apriltags) by Duckietown

## How to get started

Note for the ArduCam, use ``sudo apt install i2c-tools`` and ensure ``dtparam=i2c_vc=on`` is in ``/boot/config.txt``.

Note for using libcamera, ensure that the picamera2 package is installed via ``sudo apt install -y python3-picamera2 --no-install-recommends``
before creating the virtual environment.

Set up the virtual environment and install the required packages:

```
python -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Users running in a headless environment may need to also ``sudo apt install ffmpeg libsm6 libxext6``.

Print the Apriltags (``tagStandard41h12.pdf``) and place on the ceiling of the travel area of the robot. A density of 2-4 tags per m^2 is recommended, the idea being that the camera is able to see 3+ tags at any one time.

The camera should be mounted such that it has an unobstructed view of the tags on the ceiling.

If you are not using one of the currently supported cameras, you will need to calibrate the camera (see Adding a New Camera below).

Run the ``capture_test.py`` to check the image quality, particularly when the vehicle is moving and turning. ``process_test.py`` can then be used to confirm if the Apriltags are detectable.

### Note for the Raspberry Pi

In some cases, the default OpenCV for the Raspberry Pi may not process frames fast enough.

In general, 5+ FPS is enough for a live feed to ArduPilot.

See https://qengineering.eu/install-opencv-4.4-on-raspberry-pi-4.html for building an optimised
version of OpenCV.

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
for confirming the camera calibration.

```
$ geo_test.py
```
[Use the --gui option to get a plot of the tag and vehicle locations]

## Connecting to ArduPilot

```
$ aprilmav.py
```

- ``--tagSize``       Apriltag size in mm
- ``--camera``        Camera profile in camera.yaml
- ``--maxerror``      Maximum pose error to use, in n*E-8 units
- ``--outfile``       Output tag data to this file
- ``--device``        MAVLink connection string
- ``--baud``          MAVLink baud rate, if using serial port in ``--device``
- ``--source-system`` MAVLink Source system
- ``--imageFolder``   Save processed images to this folder
- ``--video``         Output video to port, 0 to disable
- ``--decimation``    Apriltag decimation. Tradeoff against detection speed and accuracy.
    
Captures, processes and localises vehicle position and orientation. Sends this in MAVLink format
to a connected ArduPilot.

It will send a heartbeat, plus the VISION_POSITION_DELTA and VISION_POSITION_ESTIMATE messages. By default, these messages will be sent to ``udp:127.0.0.1:14550``, but can be changed via the ``--device`` argument.

The following parameters will need to be set in ArduPilot:

```
VISO_TYPE        1
VISO_ORIENT      0
EK3_SRC1_POSXY   6
EK3_SRC1_POSZ    1
EK3_SRC1_VELXY   6
EK3_SRC1_VELZ    6
EK3_SRC1_YAW     6
```

Note that, assuming the camera is mounted facing up (to the ceiling) with the bottom of the image towards the front of the vehicle, then ``VISO_ORIENT`` should be 0. The coordinate frame conversion takes places within aprilmav.

The ``VISO_DELAY_MS`` should be set to 1000/framerate (ie 7fps gives a ``VISO_DELAY_MS`` of 142).

The ``--video`` and ``--imageFolder`` options will have a performance impact. It is recommended not to use these options unless for debugging purposes.

Note if you are using the ``--video`` option, you will likely require to build OpenCV from source, as most pre-built
packages do not include GStreamer support. See https://linuxize.com/post/how-to-install-opencv-on-ubuntu-20-04/ for
details.

To check if your OpenCV has GStreamer support, see https://learnopencv.com/get-opencv-build-information-getbuildinformation/

## Camera compatibility

The following cameras have been tested as compatible with Aprilmav:

- Raspberry Pi Camera V2 (needs to be used in a very well lit room. Is not accurate in Apriltag pose detection)
- Arducam 1MP OV9281 (works quite well, as it has a global shutter). Requires specific libraries. Run ``./lib/getArduCamfiles.sh`` to download the files.
- Generic USB webcams. Will need a specific calibration and settings file for each camera model.

Cameras with global shutters are preferred, as they give much more accurate position solutions.

### Adding a new camera

To add a new camera, follow the following steps:

- Create a camera driver in the ``./lib`` directory. Look at ``cameraGenericUSB.py`` as an example.
- Add a camera profile in ``camera.yaml``
- Run ``capture_test.py`` whilst showing a chessboard (https://github.com/opencv/opencv/blob/master/doc/pattern.png) in a variety of orientations and distances
- Run ``cameracal.py`` using the above images and put the resultant camera parameters in in ``camera.yaml``

Note a separate profile will be required for a specific lens and resolution combination.

