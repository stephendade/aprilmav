/**
 * @file apriltags-vpi.h
 * @brief AprilTag detection library using NVIDIA VPI acceleration
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
    int id;                      // Tag ID
    cv::Point2f center;          // Tag center in image coordinates
    std::vector<cv::Point2f> corners; // Tag corners in image coordinates (clockwise)
    py::array_t<double> rotation;         // Rotation matrix
    py::array_t<double> translation;      // Translation vector
    double decisionMargin;     // Decision margin for tag detection
    double error;              // Object-space error of the pose estimation
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