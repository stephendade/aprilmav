'''
Testing for geo and process
'''
import argparse
import os
import geo_test
import process_test
import cameracal

def test_geo_execution():
    """
    Test the execution of the geo_test module with various parameters.

    This function sets up an argument parser to handle command-line arguments
    for testing the geo_test module. It includes options for tag size, camera
    profile, number of frames to process, maximum pose error, input folder,
    output file, GUI display, apriltag decimation, maximum position change
    between frames, moving average of frames, and folder to save processed
    images. After parsing the arguments, it calls the main function of the
    geo_test module and asserts that the output file is created.

    Command-line Arguments:
        --tagSize (int): Apriltag size in mm (default: 96).
        --camera (str): Camera profile in camera.yaml (default: "SimCamera-720p").
        --loop (int): Number of frames to capture and process (default: 20).
        --maxerror (int): Maximum pose error to use, in n*E-8 units (default: 400).
        --folder (str): Folder of images to use instead of camera (default: "").
        --outfile (str): File to output tag data (default: "geo_test_results.csv").
        --gui (bool): Display GUI (default: False).
        --decimation (int): Apriltag decimation (default: 2).
        --outliers (int): Reject any outlier positions, based on last N frames (default: 5).
        --imageFolder (str): Folder to save processed images (default: "").
        --extraopt (bool): Optimise best position better (default: False).

    Raises:
        AssertionError: If the output file "geo_test_results.csv" does not exist.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--tagSize", type=int, default=96,
                        help="Apriltag size in mm")
    parser.add_argument("--camera", type=str, default="SimCamera-720p",
                        help="Camera profile in camera.yaml")
    parser.add_argument("--loop", type=int, default=20,
                        help="Capture and process this many frames")
    parser.add_argument("--maxError", type=int, default=400,
                        help="Maximum pose error to use, in n*E-8 units")
    parser.add_argument("--inputFolder", type=str, default="",
                        help="Use a folder of images instead of camera")
    parser.add_argument("--outFile", type=str, default="geo_test_results.csv",
                        help="Output tag data to this file")
    parser.add_argument('--gui', dest='gui',
                        default=False, action='store_true')
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
    parser.add_argument("--outliers", type=int,
                        default=5, help="Reject any outlier positions, based on last N frames")
    parser.add_argument("--outputFolder", type=str, default="./test1",
                        help="Save processed images to this folder")
    parser.add_argument('--extraOpt', dest='extraOpt', help="Optimise best position better",
                        default=True, action='store_true')
    parser.add_argument('--cuda', dest='cuda', help="Use OpenCV CUDA Extensions",
                        default=False, action='store_true')
    parser.add_argument("--tagFamily", type=str, default="tag36h11",
                        help="Apriltag family")
    parser.add_argument("--multiCamera", type=str, default=None,
                        help="multiple cameras using the specified yaml file")
    parser.add_argument('--tagEngine', dest='tagEngine', help="Tag detector engine",
                        default='PyAprilTags', choices=['OpenCV', 'PyAprilTags', 'JetsonPVA'])
    parser.add_argument('--R', type=float, default=0.06,
                        help="EKF measurement uncertainty, in m")
    parser.add_argument('--Ppos', type=float, default=0.05,
                        help="EKF position uncertainty, in m")
    parser.add_argument('--PVel', type=float, default=0.1,
                        help="EKF velocity uncertainty, in m/s")
    parser.add_argument('--PAccel', type=float, default=0.1,
                        help="EKF acceleration uncertainty, in m/s^2")
    args = parser.parse_args()
    geo_test.main(args)
    assert os.path.exists("geo_test_results.csv")


def test_process_execution_single():
    """
    Test the execution of the process_test with sinle camera
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=str, default="SimCamera-720p",
                        help="Camera profile in camera.yaml")
    parser.add_argument("--loop", type=int, default=20,
                        help="Process this many frames")
    parser.add_argument("--tagSize", type=int, default=94,
                        help="Apriltag size in mm")
    parser.add_argument("--inputFolder", type=str, default=None,
                        help="Use a folder of images instead of live camera")
    parser.add_argument("--outFile", type=str, default="processed.csv",
                        help="Output tag data to this file")
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
    parser.add_argument('--cuda', dest='cuda', help="Use OpenCV CUDA Extensions",
                        default=False, action='store_true')
    parser.add_argument("--tagFamily", type=str, default="tag36h11",
                        help="Apriltag family")
    parser.add_argument("--multiCamera", type=str, default=None,
                        help="multiple cameras using the specified yaml file")
    parser.add_argument('--tagEngine', dest='tagEngine', help="Tag detector engine",
                        default='PyAprilTags', choices=['OpenCV', 'PyAprilTags', 'JetsonPVA'])
    parser.add_argument('--R', type=float, default=0.06,
                        help="EKF measurement uncertainty, in m")
    parser.add_argument('--Ppos', type=float, default=0.05,
                        help="EKF position uncertainty, in m")
    parser.add_argument('--PVel', type=float, default=0.1,
                        help="EKF velocity uncertainty, in m/s")
    parser.add_argument('--PAccel', type=float, default=0.1,
                        help="EKF acceleration uncertainty, in m/s^2")
    args = parser.parse_args()
    process_test.main(args)
    assert os.path.exists("processed.csv")


def test_process_execution_multi():
    """
    Test the execution of the process_test with multiple cameras and opencv
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=str, default="",
                        help="Camera profile in camera.yaml")
    parser.add_argument("--loop", type=int, default=20,
                        help="Process this many frames")
    parser.add_argument("--tagSize", type=int, default=94,
                        help="Apriltag size in mm")
    parser.add_argument("--inputFolder", type=str, default=None,
                        help="Use a folder of images instead of live camera")
    parser.add_argument("--outFile", type=str, default="processed.csv",
                        help="Output tag data to this file")
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
    parser.add_argument('--cuda', dest='cuda', help="Use OpenCV CUDA Extensions",
                        default=False, action='store_true')
    parser.add_argument("--tagFamily", type=str, default="tag36h11",
                        help="Apriltag family")
    parser.add_argument("--multiCamera", type=str, default="camera-multi.yaml",
                        help="multiple cameras using the specified yaml file")
    parser.add_argument('--tagEngine', dest='tagEngine', help="Tag detector engine",
                        default='OpenCV', choices=['OpenCV', 'PyAprilTags', 'JetsonPVA'])
    args = parser.parse_args()
    process_test.main(args)
    assert os.path.exists("processed.csv")


def test_camera_calibration():
    """
    Test the camera calibration process.

    This function sets up an argument parser to handle command-line arguments
    for camera calibration. It includes options for chessboard columns, rows,
    folder of images, fisheye lens, half resolution, and GUI display. After
    parsing the arguments, it calls the main function of the cameracal module.

    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--cbcol", type=int, default=6,
                        help="Number of chessboard columns-1")
    parser.add_argument("--cbrow", type=int, default=9,
                        help="Number of chessboard rows-1")
    parser.add_argument("--folder", type=str, default="./caldataArduCam",
                        help="Use a folder of images")
    parser.add_argument("--fisheye", action="store_true",
                        help="Use fisheye lens model", default=True)
    parser.add_argument("--halfres", action="store_true",
                        help="Use half resolution images")
    parser.add_argument('--gui', dest='gui',
                        default=False, action='store_true')
    args = parser.parse_args()

    # Assuming cameracal is a module that contains the main function for calibration
    cameracal.run_camera_calibration(args)
