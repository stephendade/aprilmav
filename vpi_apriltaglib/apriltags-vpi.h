/*
 * AprilTag VPI Python Bindings
 *
 * Copyright (C) 2025 Stephen Dade <stephen_dade@hotmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#ifndef APRILTAGS_VPI_H
#define APRILTAGS_VPI_H

namespace apriltags_vpi {

/**
 * @brief Detected AprilTag information
 */
struct TagDetection {
    int tag_id;                      // Tag ID
    cv::Point2f center;          // Tag center in image coordinates
    std::vector<cv::Point2f> corners; // Tag corners in image coordinates (clockwise)
    py::array_t<double> pose_R;         // Rotation matrix
    py::array_t<double> pose_t;      // Translation vector
    double decisionMargin;     // Decision margin for tag detection
    double pose_err;              // Object-space error of the pose estimation
};

/**
 * @brief AprilTag detector class using VPI acceleration
 */
class ApriltagDetectorVPI {
public:
    /**
     * @brief Constructor
    * @param family Tag family (e.g., "tagStandard41h12")
    * @param hamming Maximum Hamming distance for detection
    * @param width Image width
    * @param height Image height
     */
    explicit ApriltagDetectorVPI(std::string family, int hamming, int width, int height);

    /**
     * @brief Destructor
     */
    ~ApriltagDetectorVPI();

    /**
     * @brief Detect AprilTags in the input image
     * @param img Input image (grayscale)
     * @param tagSize Size of the tag in meters
     * @param fx Focal length in x direction
     * @param fy Focal length in y direction
     * @param cx Optical center x coordinate
     * @param cy Optical center y coordinate
     * @return Vector of detected tags
     */
     std::vector<TagDetection> detect(py::array_t<uint8_t>& img, float tagSize, float fx, float fy, float cx, float cy);

private:
    VPIStream stream = NULL;
    VPIImage vpi_img = nullptr;
    VPIPayload payload;
    VPIArray detections;
    VPIArray poses;
    const int maxDetections = 64;

    // Private helper methods can be added here
};

} // namespace apriltags_vpi

#endif // APRILTAGS_VPI_H