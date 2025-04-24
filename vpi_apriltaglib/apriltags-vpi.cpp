/*
Software library for detecting Apriltags using the NVIDIA VPI (Vision Programming Interface) library.

Requires a Jetson with VPI support to run

*/
#include <iostream>

#include <vpi/OpenCVInterop.hpp>

#include <vpi/Image.h>
#include <vpi/Stream.h>
#include <vpi/algo/AprilTags.h>
#include <vpi/Array.h>
#include <vpi/ArrayType.h>

#include "apriltags-vpi.h"

namespace py = pybind11;


#define CHECK_STATUS(STMT)                                    \
    do                                                        \
    {                                                         \
        VPIStatus status = (STMT);                            \
        if (status != VPI_SUCCESS)                            \
        {                                                     \
            char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH];       \
            vpiGetLastStatusMessage(buffer, sizeof(buffer));  \
            std::ostringstream ss;                            \
            ss << vpiStatusGetName(status) << ": " << buffer; \
            throw std::runtime_error(ss.str());               \
        }                                                     \
    } while (0);

namespace apriltags_vpi {
// Implementation

ApriltagDetectorVPI::ApriltagDetectorVPI(std::string strFamily, int hamming, int width, int height)
    {
    // Initialize VPI
    CHECK_STATUS(vpiStreamCreate(VPI_BACKEND_CUDA | VPI_BACKEND_CPU | VPI_BACKEND_PVA, &stream));

    // convert the config tag family to VPIAprilTagFamily
    VPIAprilTagFamily family;
    if (strFamily == "tag36h11") {
        family = VPI_APRILTAG_36H11;
    } else if (strFamily == "tag25h9") {
        family = VPI_APRILTAG_25H9;
    } else if (strFamily == "tag16h5") {
        family = VPI_APRILTAG_16H5;
    } else if (strFamily == "tagCircle21h7") {
        family = VPI_APRILTAG_CIRCLE21H7;
    } else if (strFamily == "tagCircle49h12") {
        family = VPI_APRILTAG_CIRCLE49H12;
    } else if (strFamily == "tagStandard41h12") {
        family = VPI_APRILTAG_STANDARD41H12;
    } else {
        throw std::runtime_error("Unknown tag family: " + strFamily);
    }
    VPIAprilTagDecodeParams params = {NULL, 0, hamming, family};
    vpiCreateAprilTagDetector(VPI_BACKEND_CPU, width, height, &params, &payload);
    const int maxDetections = 64;
   
    CHECK_STATUS(vpiArrayCreate(maxDetections, VPI_ARRAY_TYPE_APRILTAG_DETECTION, VPI_BACKEND_CPU, &detections));
    CHECK_STATUS(vpiArrayCreate(maxDetections, VPI_ARRAY_TYPE_POSE, VPI_BACKEND_CPU, &poses));
    }

ApriltagDetectorVPI::~ApriltagDetectorVPI() {
    vpiStreamDestroy(stream);
    vpiImageDestroy(vpi_img);

    vpiArrayDestroy(detections);
    vpiArrayDestroy(poses);
    vpiPayloadDestroy(payload);
}

std::vector<TagDetection> ApriltagDetectorVPI::detect(py::array_t<uint8_t>& img, float tagSize, float fx, float fy, float cx, float cy) {
    std::vector<TagDetection> detections_out;

    //check if img is valid
    if (img.size() == 0) {
        std::cerr << "Input image is empty." << std::endl;
        return detections_out;
    }
    // Check if the image is grayscale
    if (img.ndim() != 2) {
        std::cerr << "Input image must be grayscale." << std::endl;
        return detections_out;
    }

    auto rows = img.shape(0);
    auto cols = img.shape(1);
    //std::cout << "rows: " << rows << " cols: " << cols << std::endl;
    auto type = CV_8UC1;
    cv::Mat cv_image(rows, cols, type, (unsigned char*)img.data());

    // Check if the cv image is valid
    if (cv_image.empty()) {
        std::cerr << "Input image is empty." << std::endl;
        return detections_out;
    }
    // Check if the cv is grayscale
    if (cv_image.type() != CV_8UC1) {
        std::cerr << "Input image must be grayscale." << std::endl;
        return detections_out;
    }

    // Wrap it into a VPIImage
    if (vpi_img == nullptr)
    {
        // Now create a VPIImage that wraps it.
        CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(cv_image, 0, &vpi_img));
    }
    else
    {
        CHECK_STATUS(vpiImageSetWrappedOpenCVMat(vpi_img, cv_image));
    }
    
    // Do april tag detection here
    CHECK_STATUS(vpiSubmitAprilTagDetector(stream, VPI_BACKEND_CPU, payload, maxDetections, vpi_img, detections));
    const VPICameraIntrinsic vpi_intrinsics = {{fx, 0.0f, cx}, {0.0f, fy, cy}};
    CHECK_STATUS(vpiSubmitAprilTagPoseEstimation(stream, VPI_BACKEND_CPU, detections, vpi_intrinsics, tagSize, poses));
    // Wait until conversion finishes.
    CHECK_STATUS(vpiStreamSync(stream));

    // print out the detected tags
    int32_t num_detections = 0;
    CHECK_STATUS(vpiArrayGetSize(detections, &num_detections));
    VPIArrayData detections_data;
    VPIArrayData poses_data;
    CHECK_STATUS(
        vpiArrayLockData(
        detections, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS,
        &detections_data));
    CHECK_STATUS(
        vpiArrayLockData(
            poses, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS,
            &poses_data));
    VPIAprilTagDetection * detection_conv =
        reinterpret_cast<VPIAprilTagDetection *>(detections_data.buffer.aos.data);
        VPIPose * pose_conv =
        reinterpret_cast<VPIPose *>(poses_data.buffer.aos.data);

    //std::cout << "Number of detections: " << num_detections << std::endl;

    // convert from VPI to output format
    for (int32_t i = 0; i < num_detections; i++) {
        const VPIAprilTagDetection & detection = detection_conv[i];
        const VPIPose & pose = pose_conv[i];

        // Create a TagDetection object and fill it with the data
        TagDetection tag_detection;
        tag_detection.tag_id = detection.id;
        tag_detection.decisionMargin = detection.decisionMargin;
        tag_detection.pose_err = pose.error;
        tag_detection.corners = std::vector<cv::Point2f>(4);
        // Copy the corners from the VPIAprilTagDetection to the TagDetection
        tag_detection.corners[0] = cv::Point2f(detection.corners[0].x, detection.corners[0].y);
        tag_detection.corners[1] = cv::Point2f(detection.corners[1].x, detection.corners[1].y);
        tag_detection.corners[2] = cv::Point2f(detection.corners[2].x, detection.corners[2].y);
        tag_detection.corners[3] = cv::Point2f(detection.corners[3].x, detection.corners[3].y);
        tag_detection.center = cv::Point2f(detection.center.x, detection.center.y);

        // Create a 2D array with proper shape
        std::array<size_t, 2> shape = {3, 3};
        py::array_t<double> rotation(shape);

        // Get a mutable pointer to the data buffer
        py::buffer_info buf = rotation.request();
        double* ptr = static_cast<double*>(buf.ptr);

        // Fill the 3x3 rotation matrix correctly using direct buffer access
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                // Row-major indexing: i*ncols + j
                ptr[i*3 + j] = pose.transform[i][j];
            }
        }

        // Now assign this properly-filled array to the tag_detection
        tag_detection.pose_R = rotation;

        // Create a 1D array for translation
        py::array_t<double> translation({3});

        // Fill using unchecked accessor
        auto t = translation.mutable_unchecked<1>();
        t(0) = pose.transform[0][3];
        t(1) = pose.transform[1][3];
        t(2) = pose.transform[2][3];

        // Assign to tag_detection
        tag_detection.pose_t = translation;

        // Add the tag detection to the output vector
        detections_out.push_back(tag_detection);
    }
    vpiArrayUnlock(detections);
    vpiArrayUnlock(poses);
    
    return detections_out;
}

} // namespace apriltags_vpi