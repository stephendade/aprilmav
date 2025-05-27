#!/usr/bin/env python3
"""
Example of using the VPI Apriltag detector

Requires a NVIDIA Jetson with VPI 3.2

Ensure the C++ bindings are built before running this script

"""
import sys
import time
import cv2
from build.apriltagVPI import ApriltagVPI


# Example usage
if __name__ == "__main__":

    # Create detector
    detector = ApriltagVPI(family="tagStandard41h12", hamming=0, width=3840, height=2160)

    # Load an image
    img = cv2.imread("example.jpg", cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("Error: Could not load test image")
        sys.exit(1)

    # print image dimensions
    print(f"Image loaded with dimensions: {img.shape[1]}x{img.shape[0]}")

    # Detect tags in loop for averaging performance
    alltimes = []
    for i in range(100):
        time_start = time.time()
        detections = detector.detect(img, tagSize=0.1, fx=2514.4519625308344, fy=2514.7614761401096,
                                     cx=1858.299271966736, cy=1033.4589590209453)
        time_end = time.time()
        alltimes.append(time_end - time_start)
    mean_time = sum(alltimes) / len(alltimes)
    print(f"Detection time avg: {1000*(mean_time):.0f} ms")

    # Print results
    print(f"Found {len(detections)} tags")
    for det in detections:
        print(f"Tag ID: {det.tag_id}")
        print(f"  Center: ({det.center.x:.2f}, {det.center.y:.2f})")
        print(f"  Decision margin: {det.decisionMargin:.2f}")
        print(f"  Corners: {det.corners}")
        print(f"  Translation: {det.pose_t}")
        print(f"  Rotation: {det.pose_R}")
        print(f"  Pose error 1E5: {1E5 * det.pose_err}")

    print("Done")
