'''
Testing for geo and process
'''
import argparse
import os
import geo_test
import process_test


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
        --averaging (int): Use moving average of N frames (default: 5).
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
    parser.add_argument("--loop", type=int, default=80,
                        help="Capture and process this many frames")
    parser.add_argument("--maxerror", type=int, default=400,
                        help="Maximum pose error to use, in n*E-8 units")
    parser.add_argument("--folder", type=str, default="",
                        help="Use a folder of images instead of camera")
    parser.add_argument("--outfile", type=str, default="geo_test_results.csv",
                        help="Output tag data to this file")
    parser.add_argument('--gui', dest='gui',
                        default=False, action='store_true')
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
    parser.add_argument("--averaging", type=int,
                        default=5, help="Use moving average of N frames")
    parser.add_argument("--imageFolder", type=str, default="",
                        help="Save processed images to this folder")
    parser.add_argument('--extraopt', dest='extraopt', help="Optimise best position better",
                        default=True, action='store_true')
    parser.add_argument('--jetson', dest='jetson', help="Use Jetson hardware acceleration",
                        default=False, action='store_true')
    parser.add_argument("--tagFamily", type=str, default="tag36h11",
                        help="Apriltag family")
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
    parser.add_argument("--loop", type=int, default=80,
                        help="Process this many frames")
    parser.add_argument("--tagSize", type=int, default=94,
                        help="Apriltag size in mm")
    parser.add_argument("--inputFolder", type=str, default=None,
                        help="Use a folder of images instead of live camera")
    parser.add_argument("--outFile", type=str, default="processed.csv",
                        help="Output tag data to this file")
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
    parser.add_argument('--jetson', dest='jetson', help="Use Jetson hardware acceleration",
                        default=False, action='store_true')
    parser.add_argument("--tagFamily", type=str, default="tag36h11",
                        help="Apriltag family")
    parser.add_argument("--multiCamera", type=str, default=None,
                        help="multiple cameras using the specified yaml file")
    args = parser.parse_args()
    process_test.main(args)
    assert os.path.exists("processed.csv")


def test_process_execution_multi():
    """
    Test the execution of the process_test with sinle camera
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=str, default="",
                        help="Camera profile in camera.yaml")
    parser.add_argument("--loop", type=int, default=80,
                        help="Process this many frames")
    parser.add_argument("--tagSize", type=int, default=94,
                        help="Apriltag size in mm")
    parser.add_argument("--inputFolder", type=str, default=None,
                        help="Use a folder of images instead of live camera")
    parser.add_argument("--outFile", type=str, default="processed.csv",
                        help="Output tag data to this file")
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
    parser.add_argument('--jetson', dest='jetson', help="Use Jetson hardware acceleration",
                        default=False, action='store_true')
    parser.add_argument("--tagFamily", type=str, default="tag36h11",
                        help="Apriltag family")
    parser.add_argument("--multiCamera", type=str, default="camera-multi.yaml",
                        help="multiple cameras using the specified yaml file")
    args = parser.parse_args()
    process_test.main(args)
    assert os.path.exists("processed.csv")