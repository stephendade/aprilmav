'''
Testing for geo and process
'''
import argparse
import os
import pytest
import geo_test
import process_test


def test_geo_execution():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tagSize", type=int, default=96,
                        help="Apriltag size in mm")
    parser.add_argument("--camera", type=str, default="ArduCamUC580",
                        help="Camera profile in camera.yaml")
    parser.add_argument("--loop", type=int, default=20,
                        help="Capture and process this many frames")
    parser.add_argument("--maxerror", type=int, default=400,
                        help="Maximum pose error to use, in n*E-8 units")
    parser.add_argument("--folder", type=str, default="test_data",
                        help="Use a folder of images instead of camera")
    parser.add_argument("--outfile", type=str, default="geo_test_results.csv",
                        help="Output tag data to this file")
    parser.add_argument('--gui', dest='gui',
                        default=False, action='store_true')
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
    parser.add_argument("--maxjump", type=int,
                        default=0.5, help="Maximum position change allowed between frames in cm")
    parser.add_argument("--averaging", type=int,
                        default=5, help="Use moving average of N frames")
    parser.add_argument("--imageFolder", type=str, default="",
                        help="Save processed images to this folder")
    args = parser.parse_args()
    geo_test.main(args)
    assert os.path.exists("geo_test_results.csv")


def test_process_execution():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=str, default="ArduCamUC580",
                        help="Camera profile in camera.yaml")
    parser.add_argument("--loop", type=int, default=10,
                        help="Process this many frames")
    parser.add_argument("--tagSize", type=int, default=94,
                        help="Apriltag size in mm")
    parser.add_argument("--folder", type=str, default="test_data",
                        help="Use a folder of images instead of camera")
    parser.add_argument("--outfile", type=str, default="processed.csv",
                        help="Output tag data to this file")
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
    args = parser.parse_args()
    process_test.main(args)
    assert os.path.exists("processed.csv")
