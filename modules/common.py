'''
Commonly used functions in AprilMAV
'''
import os
import sys
from importlib import import_module
import concurrent.futures

import cv2
import yaml

from drivers import cameraFile


def loadCameras(multiCamera, singleCameraProfile, inputFolder, jetson):
    '''
    Load camera parameters from the camera_params.yaml file and initialize the cameras.

    Parameters:
    multiCamera (str or None): Path to the YAML file containing multiple camera profiles
    singleCameraProfile (str): The profile name to be used from the YAML file when multiCamera is None.
    inputFolder (str or None): Path to the folder containing input files for the camera
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
        # 3 options here: folder single, folder multi and live camera
        if inputFolder:
            imageFolder = inputFolder
            if multiCamera:
                imageFolder = os.path.join(inputFolder, camName)
            CAMERAS.append(cameraFile.FileCamera(camParam, imageFolder, jetson, camName))
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


def get_num_images(CAMERAS, loop):
    """
    Get the number of images available in the file camera(s)

    Args:
        CAMERAS (list): A list of camera objects to capture from.

    Returns:
        int: The number of images available in the file camera(s), Otherwise loop
    """
    num_images = CAMERAS[0].getNumberImages()
    for CAMERA in CAMERAS:
        if CAMERA.getNumberImages():
            num_images = min(num_images, CAMERA.getNumberImages())
    if num_images is None:
        num_images = loop
    return num_images


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


def labelTags( image, tags):
    """
    Label the tags in the image
    """
    # loop over the AprilTag detection results
    for r in tags:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag ID
        scale = image.shape[1] / 1000
        cv2.putText(image, str(r.tag_id), (ptA[0] + int(10 * scale), ptA[1] - int(10 * scale)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7 * scale, (0, 255, 0), int(1 * scale))
    return image
