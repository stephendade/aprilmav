'''
Commonly used functions in AprilMAV
'''
import sys
from importlib import import_module

import yaml


def loadCameras(multiCamera, singleCameraProfile, inputFolder, jetson):
    '''
    Load camera parameters from the camera_params.yaml file and initialize the cameras.

    Parameters:
    multiCamera (str or None): Path to the YAML file containing multiple camera profiles. If None, singleCameraProfile is used.
    singleCameraProfile (str): The profile name to be used from the YAML file when multiCamera is None.
    inputFolder (str or None): Path to the folder containing input files for the camera. If None, live camera feed is used.
    jetson (bool): Flag indicating whether the system is running on a Jetson device.

    Returns:
    list: A list of initialized camera objects.

    Raises:
    ImportError: If the specified camera driver module cannot be imported.
    KeyError: If the specified camera profile is not found in the YAML file.
    SystemExit: If no camera with the specified name is found.
    '''
    # Open camera settings
    camProfile = []
    if not multiCamera:
        with open('camera.yaml', 'r', encoding="utf-8") as stream:
            parameters = yaml.load(stream, Loader=yaml.FullLoader)
        camProfile.append((parameters[singleCameraProfile], singleCameraProfile))
    else:
        with open(multiCamera, 'r', encoding="utf-8") as stream:
            parameters = yaml.load(stream, Loader=yaml.FullLoader)
        for camera in parameters:
            camProfile.append((parameters[camera], camera))

    # initialize the camera(s)
    CAMERAS = []
    for camParam, camName in camProfile:
        if inputFolder:
            from drivers import cameraFile
            CAMERAS.append(cameraFile.FileCamera(camParam, inputFolder, jetson, camName))
            print("Camera {0} initialized (driver: {1})".format(camName, "cameraFile"))
        else:
            try:
                mod = import_module("drivers." + camParam['cam_driver'])
                CAMERAS.append(mod.camera(camParam, jetson, camName))
            except (ImportError, KeyError):
                print('No camera with the name {0}, exiting'.format(camName))
                sys.exit(0)
            print("Camera {0} initialized (driver: {1})".format(camName, camParam['cam_driver']))
    return CAMERAS
