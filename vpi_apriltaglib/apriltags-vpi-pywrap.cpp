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

#define NPY_NO_DEPRECATED_API NPY_API_VERSION
#include <iostream>

#include <stdbool.h>
#include <Python.h>
#include <structmember.h>
#include <signal.h>

#include <vpi/OpenCVInterop.hpp>

#include "apriltags-vpi.h"

// Because I'm lazy and there is no risk of name collision
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
        .def_readwrite("tag_id", &apriltags_vpi::TagDetection::tag_id)
        .def_readwrite("center", &apriltags_vpi::TagDetection::center)
        .def_readwrite("corners", &apriltags_vpi::TagDetection::corners)
        .def_readwrite("decisionMargin", &apriltags_vpi::TagDetection::decisionMargin)
        .def_readwrite("pose_R", &apriltags_vpi::TagDetection::pose_R)
        .def_readwrite("pose_t", &apriltags_vpi::TagDetection::pose_t)
        .def_readwrite("pose_err", &apriltags_vpi::TagDetection::pose_err)
        .def("__repr__", [](const apriltags_vpi::TagDetection& tag) {
            return "<TagDetection tag_id=" + std::to_string(tag.tag_id) + ">";
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

