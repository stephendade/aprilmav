# aprilmav - Indoor navigation via Apriltags over MAVLink

This library is a work-in-progress to provide accurate (cm-level)
indoor navigation for MAVlink (APM, etc) vehicles via Apriltags (
developed by [AprilRobotics](https://april.eecs.umich.edu/))

The advantage of this method is that it does not require any expensive
or complicated equipment. All you need is:
- Printed April tags on a ~A4 paper mounted around the areas of travel
- Small embedded computer (Raspberry Pi or similar) with camera (Global shutter preferred)

This library uses the the [Apriltags Python bindings](https://github.com/WillB97/pyapriltags).

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

Run the ``capture_test.py`` to check the image quality, particularly when the vehicle is moving and turning. ``process_test.py`` can then be used to confirm if the Apriltags are detectable and system performance.

### Note for the Raspberry Pi

The Raspberry Pi 4 is *just* powerful enough to run aprilmav in realtime (~10fps). It is highly recommended
to use the Raspberry Pi 5 instead, which can give ~30fps with a 1.5MP camera

In some cases, the default OpenCV for the Raspberry Pi 4 may not process frames fast enough.

In general, 5+ FPS is enough for a live feed to ArduPilot.

See https://qengineering.eu/install-opencv-4.4-on-raspberry-pi-4.html for building an optimised
version of OpenCV.

## Performance Testing

There are several scripts for testing the performance - both accuracy and
detected speed:

Speed of camera capture. Useful for confirming if the camera itself is a system
bottleneck. Use ``--outputFolder=xxx`` to save the images to a specific folder, which is
useful for playing back datasets in the other scripts:

```
$ capture_test.py
```

Speed of camera capture and Apriltag detection. Useful for confirming the camera
calibration via the Apriltag distances from the camera:

```
$ process_test.py
```

Localisation test. Estimates the vehicles current position and velocity based on Apriltags and 
outputs to csv. Useful for confirming the camera calibration and data quality. 
Uses many of the same options as ``aprilmav.py``

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
- ``--maxError``      Maximum pose error to use, in n*E-8 units
- ``--outFile``       Output position data to this file in csv format
- ``--device``        MAVLink connection string
- ``--baud``          MAVLink baud rate, if using serial port in ``--device``
- ``--source-system`` MAVLink Source system
- ``--outputFolder``  Save processed images to this folder
- ``--video``         Output an RTP H264 video to this IP:Port, 0 to disable
- ``--decimation``    Apriltag decimation. Tradeoff against detection speed and accuracy.
- ``--extraOpt``      Optimise detected position better. Uses a lot of extra CPU.
- ``--averaging=N``   Use a moving average of N frames for determining position and velocity.
- ``--jetson``        Use hardware accelerator (CUDA, VPI) on NVIDIA Jetson
- ``--tagFamily``     Use this Apriltag family. Defaults to ``tagStandard41h12``
- ``--opencv``        Use OpenCV instead of pyapriltag for decoding. Only works for tag family tagStandard31h11
  
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

Note that coordinate frame conversion from the camera (see [here](https://github.com/AprilRobotics/apriltag#coordinate-system)) to 
vehicle (NED) frames takes place in aprilmav. Use the ``rotationRelVehicle`` in ``camera.yaml`` to define the camera-to-vehicle 
conversion. Thus the ``VISO_ORIENT`` should be 0 in ArduPilot. An example of this is a ``rotationRelVehicle = !!python/tuple [0, 180, 90]`` for a upwards facing camera, with the bottom of the image towards the pack of the vehicle.

The ``VISO_DELAY_MS`` should be set to 1000/framerate (ie 7fps gives a ``VISO_DELAY_MS`` of 142).

The ``--video`` and ``--imageFolder`` options will have a performance impact. It is recommended not to use these options unless for debugging purposes.

Note if you are using the ``--video`` option, you will likely require to build OpenCV from source, as most pre-built
packages do not include GStreamer support. See https://linuxize.com/post/how-to-install-opencv-on-ubuntu-20-04/ for
details.

To check if your OpenCV has GStreamer support, see https://learnopencv.com/get-opencv-build-information-getbuildinformation/

## Camera compatibility

The following cameras have been tested as compatible with Aprilmav:

- Raspberry Pi Camera V2 (needs to be used in a very well lit room. Is not accurate in Apriltag pose detection). Use ``--camera=PiCamV2FullFoVHD``
- Raspberry Pi Camera GS (IMX296). The 6mm lens is quite narrow, so will need a greater density of Apriltags. Use ``--camera=imx296-6mmlens``. 
- [ArduCam UC-717 (IMX296)](https://www.arducam.com/product/1-58mp-imx296-color-global-shutter-camera-module-with-m12-lens-for-raspberry-pi/). The default 6mm lens is too narrow. For a 2.7mm lens use ``--camera=imx296-2.7mmlens``
- Arducam 1MP OV9281 (works quite well, as it has a global shutter). Requires specific libraries. Run ``./lib/getArduCamfiles.sh`` to download the files. Use ``--camera=ArduCamUC580``
- Generic USB webcams. Will need a specific calibration and settings file for each camera model. Use ``--camera=GenericUSB``
- IMX219 Cameras on the Nvidia Jetson at a variety of resolutions: ``--camera=JetsonCameraIMX219-8MP``, ``--camera=JetsonCameraIMX219-6MP`` and ``--camera=JetsonCameraIMX219-2MP``.
- IMX678 Camera (Arducam B0497 USB camera). Works quite well. Requires a high-end platform (Jetson or Laptop) to process in realtime. Use ``--camera=Arducam_B0497``

Cameras with global shutters are preferred, as they give much more accurate position solutions.

### Adding a new camera

To add a new camera, follow the following steps:

- Create a camera driver in the ``./drivers`` directory. Look at ``cameraGenericUSB.py`` as an example.
- Add a camera profile in ``camera.yaml``
- Run ``capture_test.py`` whilst showing a chessboard (https://github.com/opencv/opencv/blob/master/doc/pattern.png) in a variety of orientations and distances
- Run ``cameracal.py`` using the above images and put the resultant camera parameters in in ``camera.yaml``

Note a separate profile will be required for a specific lens and resolution combination, and camera rotation/position.

### Accuracy and Performance

ArduPilot requires a good velocity estimate from AprilMAV. This can be graphed via the xxx MAVLink messages.

``process_test.py`` can used in a static scence to confirm the detected distance is correct and the distance stability.

``geo_test.py`` can be used to output a csv file showing the postion and velocity values for analysis. This analysis is best when with an image capture set of the vehicle moving at a constant velocity in 1 direction.

``geo_test.py`` will output the timing statistics for various stages of the pipeline. This is useful for performance tuning.

If the velocity numbers are too noisy, the following options will help:
- Ensure the camera's focus is as sharp as possible for the typical Apriltag distances
- Decrease exposure time as much as possible to reduce motion blur during sharp turns (<5ms preferred)
- Decrease camera gain to reduce any noise in the images. Apriltags are capabile of being detected in quite low-light environments
- A good camera calibration (if not using one of the supplied calibrations) is essential
- Use the ``--averaging=N`` option to average over N frames. N should be a maximum of camera fps/2
- Ensure at least 3 Apriltags are visible at all times
  
### Running Simulations

A simulated camera is included to test the code. It is contained within ``./drivers/cameraSim.py`` and
produces a series of (noisy) Apriltag images forming a box track over 80 frames.

It can be used via the following arguments in ``process_test.py`` and ``geo_test.py``:

``--camera=SimCamera-720p --loop=80 --tagFamily=tag36h11``
