#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  
#include "../include/RealSense.hpp"

namespace py = pybind11;

PYBIND11_MODULE(realsense, m) {
    m.doc() = "Python bindings for RealSense class - Usine Robots";

    // bind the DetectedObject 
    py::class_<DetectedObject>(m, "DetectedObject")
        .def_readwrite("color",    &DetectedObject::color)
        .def_readwrite("cx",       &DetectedObject::cx)
        .def_readwrite("cy",       &DetectedObject::cy)
        .def_readwrite("radius",   &DetectedObject::radius)
        .def_readwrite("avgDepth", &DetectedObject::avgDepth)
        .def_readwrite("Xc",       &DetectedObject::Xc)
        .def_readwrite("Yc",       &DetectedObject::Yc)
        .def_readwrite("Zc",       &DetectedObject::Zc)
        .def_readwrite("orientation", &DetectedObject::orientation)
        .def_readwrite("width",    &DetectedObject::width)
        .def_readwrite("height",   &DetectedObject::height)
        .def_readwrite("shape",    &DetectedObject::shape);

    
    py::class_<RealSense>(m, "RealSense")
        .def(py::init<int,int,int>(), py::arg("width")=640, py::arg("height")=480, py::arg("fps")=30)
        .def("start", &RealSense::start)
        .def("stop",  &RealSense::stop)
        .def("scan",  &RealSense::scan)
        .def("getObjectCount", &RealSense::getObjectCount)
        .def("getObject",      &RealSense::getObject);
}
