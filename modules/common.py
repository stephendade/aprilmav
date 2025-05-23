'''
Commonly used functions in AprilMAV
'''
import os
import sys
from importlib import import_module
import concurrent.futures

import numpy
import cv2
import yaml

from transforms3d.euler import mat2euler

from drivers import cameraFile


def loadCameras(multiCamera, singleCameraProfile, inputFolder, use_cuda=False, tagSize=0.1, tagFamily=None,
                decimation=None, tagEngine=None):
    '''
    Load camera parameters from the camera_params.yaml file and initialize the cameras.

    Parameters:
    multiCamera (str or None): Path to the YAML file containing multiple camera profiles
    singleCameraProfile (str): The profile name to be used from the YAML file when multiCamera is None.
    inputFolder (str or None): Path to the folder containing input files for the camera
    use_cuda (bool): Flag indicating whether to use OpenCV CUDA extensions

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
            CAMERAS.append(cameraFile.FileCamera(camParam, tagSize, tagFamily, decimation, tagEngine,
                                                 imageFolder, use_cuda, camName))
            print("Camera {0} initialized (driver: {1})".format(camName, "cameraFile"))
        else:
            try:
                mod = import_module("drivers." + camParam['cam_driver'])
                CAMERAS.append(mod.camera(camParam, tagSize, tagFamily, decimation, tagEngine, use_cuda, camName))
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


def capture_detect_image(CAMERA, get_raw=False, do_detect=False):
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
            - capture_time (float or None): The time taken to capture the image, or None if no image is captured.
            - rectify_time (float or None): The time taken to rectify the image, or None if no image is captured.
    """
    filename = CAMERA.getFileName()
    (imageBW, timestamp, capture_time, rectify_time) = CAMERA.getImage(get_raw)
    if do_detect:
        (tags, detect_time) = CAMERA.doDetect(imageBW)
    else:
        tags = None
        detect_time = None

    # we're out of images
    if imageBW is None:
        return CAMERA.camName, None, None, None, None, None, None, None

    return CAMERA.camName, imageBW, timestamp, filename, tags, capture_time, rectify_time, detect_time


def do_multi_capture_detection(CAMERAS, get_raw=False, do_detect=False):
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
                - capture_time: Time taken to capture the image (sec)
                - rectify_time: Time taken to rectify the image (sec)
    Notes:
        - Uses ThreadPoolExecutor for parallel image capture
        - If any camera capture fails (returns None), the function will break early
    """
    img_tags_by_cam = {}
    if len(CAMERAS) == 1:
        # if only one camera, no need for threads
        cam_name, imageBW, timestamp, filename, tags, capture_time, rectify_time, detect_time = capture_detect_image(
            CAMERAS[0], get_raw, do_detect)
        img_tags_by_cam[cam_name] = (imageBW, timestamp, filename, tags, capture_time,
                                     rectify_time, detect_time)
    # if multiple cameras, use threads
    else:
        with concurrent.futures.ThreadPoolExecutor() as executor:
            futures = {executor.submit(capture_detect_image, CAMERA, get_raw, do_detect): CAMERA for CAMERA in CAMERAS}
            for future in concurrent.futures.as_completed(futures):
                cam_name, imageBW, timestamp, filename, tags, capture_time, rectify_time, detect_time = future.result()
                if imageBW is not None:
                    img_tags_by_cam[cam_name] = (imageBW, timestamp, filename, tags, capture_time,
                                                 rectify_time, detect_time)
                    # print("Camera {0} capture time is {1:.1f}ms".format(cam_name, 1000*(time.time() - timestamp)))
                else:
                    # print("Bad capture")
                    img_tags_by_cam[cam_name] = (None, timestamp, filename, tags, capture_time,
                                                 rectify_time, detect_time)
                    break
    return img_tags_by_cam


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


def labelTags(image, tags, thickness=1, fontsize=1):
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
        cv2.line(image, ptA, ptB, (0, 255, 0), thickness)
        cv2.line(image, ptB, ptC, (0, 255, 0), thickness)
        cv2.line(image, ptC, ptD, (0, 255, 0), thickness)
        cv2.line(image, ptD, ptA, (0, 255, 0), thickness)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 10, (0, 0, 255), -1)
        # draw the tag ID
        cv2.putText(image, str(r.tag_id), (ptA[0] + int(10 * thickness), ptA[1] - int(10 * thickness)),
                    cv2.FONT_HERSHEY_SIMPLEX, fontsize, (0, 255, 0), thickness)
    return image


def getFontSize(image):
    """
    Calculate the optimal font size for the camera name to be displayed on the image

    Args:
        image: OpenCV image

    Returns:
        tuple: A tuple containing: font_scale, thickness, text_height
    """
    text_height = None
    font_scale = 1

    img_height, img_width = image.shape[:2]
    thickness = int(img_width/500)
    target_width = img_width * 0.25  # 25% of image width
    testText = "Test1_1234567890"

    # Binary search to find optimal font scale
    (text_width, text_height), _ = cv2.getTextSize(
        testText, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)

    while text_width < target_width:
        font_scale *= 1.1
        (text_width, text_height), _ = cv2.getTextSize(
            testText, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)

    return font_scale, thickness, text_height


def getTransform(tag):
    '''tag pose to transformation matrix'''
    T_Tag = numpy.array(numpy.eye((4)))
    T_Tag[0:3, 0:3] = numpy.array(tag.pose_R)
    pose_t = numpy.array(tag.pose_t)
    T_Tag[0:3, 3] = pose_t.reshape(3)

    # flip x axis
    # T_Tag = [[-1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]] @ T_Tag
    #
    # now convert to NED coord frame
    # so x-90, y+90, z in order
    # https://www.andre-gaschler.com/rotationconverter/
    # [[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]]
    # T_Tag = [[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1]] @ T_Tag
    return T_Tag


def getPos(T_tag):
    '''output the transformation matrix position as xyz tuple'''
    return numpy.array([T_tag[0, 3], T_tag[1, 3], T_tag[2, 3]])


def getRotation(T_Tag, useRadians=False):
    '''Get the vehicle's current rotation in Euler XYZ degrees'''
    if useRadians:
        return numpy.array(mat2euler(T_Tag[0:3][0:3], 'sxyz'))
    return numpy.array(numpy.rad2deg(mat2euler(T_Tag[0:3][0:3], 'sxyz')))


def tryCheckCuda(useCuda):
    '''Check if CUDA is available'''
    if useCuda:
        try:
            if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                return True
            print("Error: No CUDA devices found or OpenCV not compiled with CUDA support")
            sys.exit(0)
        except cv2.error:
            print("Error: No CUDA devices found or OpenCV not compiled with CUDA support")
            sys.exit(0)
        except AttributeError:
            print("Error: No CUDA devices found or OpenCV not compiled with CUDA support")
            sys.exit(0)
    else:
        return True
