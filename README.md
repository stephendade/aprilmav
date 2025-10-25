# aprilmav - Indoor navigation via Apriltags over MAVLink

AprilMAV provides accurate (5cm-level) indoor navigation for MAVLink vehicles (ArduPilot, PX4) using AprilTags. This is a low-effective alternative to expensive indoor positioning systems.

A YouTube video explaing how AprilMAV works is available from https://youtu.be/JC1D4SzYrkI.

## What You Need

**Hardware:**
- Printed AprilTags (included in this repo as `tagStandard41h12.pdf`)
- Small embedded computer (Raspberry Pi 5 recommended)
- Camera with global shutter (preferred) or rolling shutter
- Vehicle running ArduPilot/PX4

**No expensive equipment required!** Just paper tags and a basic camera setup.

## Quick Start Guide

### 1. Hardware Setup

**Print and Mount Tags:**
- Print the AprilTags from `tagStandard41h12.pdf` on paper
- Mount tags on the ceiling or walls of your flight area
- The tags should be arranged out such the the camera can see 3+ tags at all times

**Camera Suggestions:**
- Global shutter cameras (like the IMX296) work best for moving vehicles
- Rolling shutter cameras (like the Pi Camera HQ camera or IMX678) work but need good lighting

### 2. Software Installation

```bash
# Install system dependencies (Ubuntu/Raspberry Pi OS)
sudo apt update
sudo apt install python3-pip python3-venv python3-opencv python3-matplotlib

# For Raspberry Pi Camera support
sudo apt install -y python3-picamera2 --no-install-recommends

# Clone and setup
git clone https://github.com/stephendade/aprilmav.git
cd aprilmav

# Create virtual environment
python -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 3. Test Your Setup

Check the ``camera.yaml`` for a list of supported cameras. To use
a supported camera, note the ``--camera`` argument to load it.

**Test camera capture:**
```bash
# Check if camera works and measure frame rate
python capture_test.py --camera=<cameraname>  # your camera profile
```

**Test AprilTag detection:**
```bash
# Check if tags are detected properly
python process_test.py --camera=<cameraname> --tagSize=100  # tag size in mm
```

**Test position estimation:**
```bash
# Test full positioning system
python geo_test.py --camera=<cameraname> --tagSize=100 --gui
```

### 4. Connect to ArduPilot

**Configure ArduPilot parameters:**
```
VISO_TYPE        1     # Enable visual odometry
VISO_ORIENT      0     # No rotation (aprilmav handles this)
EK3_SRC1_POSXY   6     # Use vision for XY position
EK3_SRC1_POSZ    1     # Use barometer for Z (altitude)
EK3_SRC1_VELXY   6     # Use vision for XY velocity  
EK3_SRC1_VELZ    6     # Use vision for Z velocity
EK3_SRC1_YAW     6     # Use vision for heading
VISO_DELAY_MS    200   # Typical delay (adjust based on your system)
```

**Run AprilMAV:**
```bash
# Basic connection to ArduPilot
python aprilmav.py --camera=<cameraname> --tagSize=100

# Connect to specific MAVLink address, assuming a mavlink connection on 192.168.1.100:14550
python aprilmav.py --camera=<cameraname> --tagSize=100 --device=udp:192.168.1.100:14550
```

## Supported Cameras

Note some cameras have multiple profiles, depending on which camera lens is used.

| Camera | Profile Name | Notes |
|--------|-------------|-------|
| Raspberry Pi Camera V2 | `PiCamV2FullFoVHD` | Needs very good lighting, rolling shutter |
| Pi Camera GS (IMX296) with wide angle lens | `imx296-6mmlens` | Global shutter, excellent quality |
| ArduCam B0445 (IMX296) | `imx296-2.7mmlens` `imx296-2.1mmlens` | Global shutter, wide angle lens |
| ArduCam OV9281 | `ArduCamUC580` | Global shutter, good performance |
| Generic USB Webcam | `GenericUSB` | Requires calibration |
| ArduCam B0497 (IMX678) | `Arducam_B0497` | Rolling shutter. Requires a Jetson Orin NX for realtime processing |

For detailed camera setup, see the [Camera Compatibility](#camera-compatibility) section below.

## Performance Requirements

**System Requirements:**
- Raspberry Pi 4 or 5, NVIDIA Jetson, laptop, or similar device.
- Good lighting for rolling shutter cameras
- Ubuntu or similar installed on device

10+ FPS is required for smooth velocity estimation. This will be dependent
of the camera resolution and the processing capacity of the device.

In general, a Raspberry Pi5 can process a 2MPixel camera at 20FPS and
a NVIDIA Orin NX can process a 8MPixel camera at 15 FPS.

## Troubleshooting

**Tags not detected:**
- Ensure tags are printed clearly and flat
- Check lighting conditions
- Verify tag size parameter matches actual printed size
- Use `process_test.py` to debug detection

**Poor position accuracy:**
- Calibrate camera if using new hardware
- Ensure 3+ tags visible at all times
- Check for motion blur (reduce exposure time)
- Verify tag placement is level and secure

**Connection issues with ArduPilot:**
- Check MAVLink connection string
- Verify ArduPilot parameters are set correctly
- Monitor MAVLink traffic with QGroundControl or Mission Planner

## Advanced Usage

### Custom Camera Calibration

If using an unsupported camera:

1. Capture calibration images:
```bash
python capture_test.py --camera=GenericUSB --outputFolder=calibration_images
```

2. Run calibration (show chessboard pattern to camera):
```bash
python cameracal.py --inputFolder=calibration_images
```

3. Add results to `camera.yaml`

### Performance Tuning

**Speed up detection:**
- Increase `--decimation` (reduces max detection distance)
- Use `--cuda` on NVIDIA Jetson systems
- Use `--tagEngine=JetsonPVA` on Jetson for hardware acceleration

**Improve accuracy:**
- Decrease `--maxError` to reject poor detections
- Use `--extraOpt` for better position optimization (uses more CPU)
- Tune EKF parameters: `--R`, `--Ppos`, `--PVel`, `--PAccel`

### Development and Testing

**Record datasets:**
```bash
python capture_test.py --outputFolder=dataset1
python geo_test.py --inputFolder=dataset1 --camera=YourCamera
```

**Generate performance reports:**
```bash
python geo_test.py --camera=YourCamera --outFile=results.csv
```
In tuning the EKF, the following arguments should be set:
- ``--R``: The noise in the measured position (m). Set higher to weight in favour of the measured position and velocity
- ``--PVel``: Process uncertainty for velocity (m/s). The typical velocity of the vehicle
- ``--PAccel``: Process uncertainty for acceleration (m/s^2). The typical acceleration of the vehicle

### Latency

Aprilmav's latency is typcially 4 frames, due to the in-built Kalman filter. This should be multiplied by the average processing
time (as per the ``geo_test.py`` summary). For example, for a Raspberry Pi5 with the IMX298 camera, the average processing time is 50ms per
frame (20fps). Multiplying by 4 for the Kalman filter gives a total 200ms delay.

### Hardware Acceleration

If running on a NVIDIA Jetson, hardware acceleration of some parts of the detection pipeline are available:

- Use ``--cuda`` to use CUDA-accelerated fisheye undistortion
- Use ``--tagEngine=JetsonPVA`` to use the Jetson PVA processor for Apriltag detection

To use the ``JetsonPVA`` detector, the custom C++ bindings need to be built. This can 
be done via the following commands:

```
sudo apt install python3-pybind11 libnvvpi3 vpi3-dev
cd ./vpi_apriltaglib
mkdir build && cd build
cmake ..
make
```
  
### Running Simulations

A simulated camera is included to test the code. It is contained within ``./drivers/cameraSim.py`` and
produces a series of (noisy) Apriltag images forming a box track over 80 frames.

It can be used via the following arguments in ``process_test.py`` and ``geo_test.py``:

``--camera=SimCamera-720p --loop=80 --tagFamily=tag36h11``
