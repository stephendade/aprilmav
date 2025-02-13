'''
Commonly used functions in AprilMAV
'''
import sys
from importlib import import_module
import concurrent.futures

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
            except KeyError:
                print('No camera with the name {0}, exiting'.format(camName))
                sys.exit(0)
            except ImportError as e:
                print('Error importing camera driver {0}: {1}'.format(camParam['cam_driver'], e))
                sys.exit(0)
            print("Camera {0} initialized (driver: {1})".format(camName, camParam['cam_driver']))
    return CAMERAS


def capture_image(CAMERA, get_raw=False):
    """
    Captures an image using the provided CAMERA object. Used in seperate threads

    Args:
        CAMERA: An object representing the camera

    Returns:
        tuple: A tuple containing:
            - camName (str): The name of the camera.
            - imageBW (numpy.ndarray or None): The captured image in black and white, or None if no image is captured.
            - timestamp (float or None): The timestamp of the captured image, or None if no image is captured.
            - filename (str or None): The filename of the captured image, or None if live camera feed is used.
    """
    filename = CAMERA.getFileName()
    (imageBW, timestamp) = CAMERA.getImage(get_raw)

    # we're out of images
    if imageBW is None:
        return CAMERA.camName, None, None

    return CAMERA.camName, imageBW, timestamp, filename


def do_multi_capture(CAMERAS, get_raw=False):
    """
    Captures images from multiple cameras simultaneously using thread pooling.
    Args:
        CAMERAS (list): A list of camera objects to capture from.
    Returns:
        dict: A dictionary containing captured images and metadata for each camera:
            - Key: Camera name/identifier
            - Value: Tuple containing:
                - imageBW: Grayscale image captured from the camera
                - timestamp: Time when image was captured
                - filename: Name of file where image was saved
    Notes:
        - Uses ThreadPoolExecutor for parallel image capture
        - If any camera capture fails (returns None), the function will break early
    """
    img_by_cam = {}
    with concurrent.futures.ThreadPoolExecutor() as executor:
        futures = {executor.submit(capture_image, CAMERA, get_raw): CAMERA for CAMERA in CAMERAS}
        for future in concurrent.futures.as_completed(futures):
            cam_name, imageBW, timestamp, filename = future.result()
            if imageBW is not None:
                img_by_cam[cam_name] = (imageBW, timestamp, filename)
                # print("Camera {0} capture time is {1:.1f}ms".format(cam_name, 1000*(time.time() - timestamp)))
            else:
                break
    return img_by_cam


def get_average_timestamps(img_by_cam):
    """
    Get the average timestamp from a list of timestamps

    Args:
        timestamps (list): A list of timestamps

    Returns:
        float: The average timestamp
    """
    timestamps = [img_by_cam[cam][1] for cam in img_by_cam]
    return sum(timestamps)/len(timestamps)
