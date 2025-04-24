/*
PyBind11 bindings for ApriltagVPI
*/

#define NPY_NO_DEPRECATED_API NPY_API_VERSION
#include <iostream>

#include <stdbool.h>
#include <Python.h>
#include <structmember.h>
#include <signal.h>

#include <vpi/OpenCVInterop.hpp>

#include "apriltags-vpi.h"

namespace apriltagvpi = pybind11;

PYBIND11_MODULE(apriltagVPI, m) {
    m.doc() = "Apriltag-VPI bindings";

    py::class_<cv::Point2f>(m, "Point2f")
    .def(py::init<>())
    .def(py::init<float, float>())
    .def_readwrite("x", &cv::Point2f::x)
    .def_readwrite("y", &cv::Point2f::y)
    .def("__repr__", [](const cv::Point2f& p) {
        return "<Point2f x=" + std::to_string(p.x) + ", y=" + std::to_string(p.y) + ">";
    });

    apriltagvpi::class_<apriltags_vpi::TagDetection>(m, "TagDetection")
        .def_readwrite("id", &apriltags_vpi::TagDetection::id)
        .def_readwrite("center", &apriltags_vpi::TagDetection::center)
        .def_readwrite("corners", &apriltags_vpi::TagDetection::corners)
        .def_readwrite("decisionMargin", &apriltags_vpi::TagDetection::decisionMargin)
        .def_readwrite("rotation", &apriltags_vpi::TagDetection::rotation)
        .def_readwrite("translation", &apriltags_vpi::TagDetection::translation)
        .def_readwrite("error", &apriltags_vpi::TagDetection::error)
        .def("__repr__", [](const apriltags_vpi::TagDetection& tag) {
            return "<TagDetection id=" + std::to_string(tag.id) + ">";
        });

    //PYBIND11_NUMPY_DTYPE(B, z, a);

    apriltagvpi::class_<apriltags_vpi::ApriltagDetectorVPI>(m, "ApriltagVPI")
        .def(apriltagvpi::init<std::string, int, int, int>(),
                apriltagvpi::arg("family") = "tag36h11",
                apriltagvpi::arg("hamming") = 0,
                apriltagvpi::arg("width") = 640,
                apriltagvpi::arg("height") = 480)
        .def("detect", &apriltags_vpi::ApriltagDetectorVPI::detect,
            apriltagvpi::arg("image"),
            apriltagvpi::arg("tagSize") = 0.05f,
            apriltagvpi::arg("fx") = 0.0f,
            apriltagvpi::arg("fy") = 0.0f,
            apriltagvpi::arg("cx") = 0.0f,
            apriltagvpi::arg("cy") = 0.0f);
}

