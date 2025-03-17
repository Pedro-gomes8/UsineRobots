#pragma once

#include <opencv2/opencv.hpp>
#include <string>

enum class ShapeType {
    SQUARE,
    CIRCLE
};


struct DetectedObject {
    ShapeType shape;
    std::string color;  

    float cx, cy; // Center of object in image pixels
    float avgDepth; // average depth (meters) of object
    float orientation; // Orientation of object [0 .. 180] degrees. Important for squares
    // Orientation is used to allow the gripper to correctly place itself to the sides of the square.
    float width, height; // Width and height of object in pixels

    float radius; // Radius of object in pixels (for circles)

    float Xc, Yc, Zc; // 3D coordinates of object in meters


};

struct ColorRange {
    cv::Scalar lower;
    cv::Scalar upper; 
};