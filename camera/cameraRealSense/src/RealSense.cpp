#include "../include/RealSense.hpp"
#include "../include/types.hpp"

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>


RealSense::RealSense(int width, int height, int fps): WIDTH(width), HEIGHT(height), FPS(fps) {
    this->RED_RANGE_1 = {cv::Scalar(0,   100, 100), cv::Scalar(10,  255, 255)};  
    this->RED_RANGE_2 = {cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255)};
    this->BLUE_RANGE = {cv::Scalar(100, 65, 65), cv::Scalar(140, 255, 255)}; 
    this->GREEN_RANGE = {cv::Scalar(50, 100, 100), cv::Scalar(90, 255, 255)};

    this->running = false;
    this->haveIntrinsics = false;

    
    // Start Real Sense Pipeline
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);

    align_to_color = new rs2::align(RS2_STREAM_COLOR);

}

// Destructor
RealSense::~RealSense() {
    stop();
    if (align_to_color){
        delete align_to_color;
    }
    align_to_color = nullptr;
    
}

/* ###############################################
*  ################### METHODS ###################
*  ###############################################
*/


// #################### START ####################

bool RealSense::start(){

    if (running) {
        return false;
    }
    pipe.start(cfg);
    running = true;
    return true;
}

// #################### STOP ####################

void RealSense::stop(){
    if (!running) {
        return;
    }
    pipe.stop();
    running = false;
    haveIntrinsics = false;
}

// #################### THRESHOLD MASK ####################

cv::Mat RealSense::thresholdColor(const cv::Mat &bgr, const ColorRange &range) {
    cv::Mat hsv, mask;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, range.lower, range.upper, mask);
    return mask;
}

// #################### GET OBJECT COUNT ####################

int RealSense::getObjectCount(){
    return static_cast<int>(objects.size());
}


// #################### GET OBJECT BY INDEX #################
DetectedObject RealSense::getObject(int index){
    if (index < 0 || index >= (int)objects.size()) {
        throw std::out_of_range("Invalid object index");
    }
    return objects[index];
}




// #################### SCAN ####################
int RealSense::scan(){
    // int maxScans = 30;
    int scan = 0;
    while(true){
    rs2::frameset frames = pipe.wait_for_frames();

    // Align depth to color
    frames = align_to_color->process(frames);

    rs2::video_frame colorFrame = frames.get_color_frame();
    rs2::depth_frame depthFrame = frames.get_depth_frame();

    if (!colorFrame || !depthFrame) {
        std::cerr << "No frame captured" << std::endl;
        return 0;
    }

    if (!haveIntrinsics) {
        // This is used for 3d projection later on. Instrinsics include focal length and optical center [docs.opencv.org]  
        auto stream_profile = colorFrame.get_profile().as<rs2::video_stream_profile>();
        intrinsics = stream_profile.get_intrinsics();
        haveIntrinsics = true;
    }

    // Create cv frame
    cv::Mat colorMat(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);

    detectObjects(colorMat, depthFrame);

    cv::imshow("Debug View", colorMat);
    
        if (cv::waitKey(1) == 27) { // ESC
            break;
        }
        scan++;
    }

    return static_cast<int>(objects.size());
}



DetectedObject RealSense::processContour(
    std::vector<cv::Point> &contour,
    cv::Mat &colorMat,
    rs2::depth_frame &depthFrame
) {
    DetectedObject obj;


    double area = cv::contourArea(contour);

    // Check bounding rectangle
    cv::RotatedRect rRect = cv::minAreaRect(contour);
    float w = rRect.size.width;
    float h = rRect.size.height;
    float angle = rRect.angle;

    // Standardize angle in [0..180)
    if (w < h) {
        std::swap(w, h);
        angle += 90.f;
    }
    if (angle < 0) angle += 180.f;

    // Check min enclosing circle
    cv::Point2f circleCenter;
    float circleRadius = 0.f;
    cv::minEnclosingCircle(contour, circleCenter, circleRadius);

    // Evaluate "circularity" [STACKOVERFLOW]
    double circleArea = CV_PI * circleRadius * circleRadius;
    double circularity = (circleArea > 0.0) ? (area / circleArea) : 0.0;

    // Heuristic: if 0.8 < circularity < 1.2 => treat as circle [STACKOVERFLOW]
    bool isCircle = (circularity > 0.8 && circularity < 1.2);

    obj.shape = (isCircle ? ShapeType::CIRCLE : ShapeType::SQUARE);

    // Final 2D center
    float cx=0, cy=0;
    if (isCircle) {
        cx = circleCenter.x;
        cy = circleCenter.y;
    } else {
        cx = rRect.center.x;
        cy = rRect.center.y;
    }

    // Depth extraction (average around center)
    int centerX = static_cast<int>(cx);
    int centerY = static_cast<int>(cy);
    float sumDepth = 0.f;
    int depthCount = 0;
    int radiusForDepth = 5;
    // Takes average depth in a 'radiusforDepth' radius around the center
    for (int dy = -radiusForDepth; dy <= radiusForDepth; dy++) {
        for (int dx = -radiusForDepth; dx <= radiusForDepth; dx++) {
            int xx = centerX + dx;
            int yy = centerY + dy;
            if (xx >= 0 && xx < colorMat.cols && yy >= 0 && yy < colorMat.rows) {
                float dist = depthFrame.get_distance(xx, yy);
                if (dist > 0.f && dist < 30.f) {
                    sumDepth += dist;
                    depthCount++;
                }
            }
        }
    }
    float avgDepth = (depthCount > 0) ? (sumDepth / depthCount) : 0.f;

    // Deproject to 3D
    float pixel[2]  = { cx, cy };
    float point3D[3]= { 0,0,0 };
    if (haveIntrinsics && avgDepth > 0.f) {
        rs2_deproject_pixel_to_point(point3D, &intrinsics, pixel, avgDepth);
    }

    // Fill out the rest
    obj.cx = cx; 
    obj.cy = cy;
    obj.avgDepth = avgDepth;
    obj.Xc = point3D[0];
    obj.Yc = point3D[1];
    obj.Zc = point3D[2];

    if (isCircle) {
        obj.orientation = 0.f;   // no orientation for circle
        obj.radius = circleRadius;
        obj.width  = circleRadius * 2.f;
        obj.height = circleRadius * 2.f;
    } else {
        // It's a "square" (or rectangle)
        obj.orientation = angle;
        obj.width  = w;
        obj.height = h;
        obj.radius = 0.f;
    }

    return obj;
}



void RealSense::getContours(cv::Mat &mask, std::string maskColor, cv::Mat &colorMat, rs2::depth_frame &depthFrame){
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);


    for (auto &c: contours){
        double area = cv::contourArea(c);
        if (area < 400.0) {
            continue;
        }

        DetectedObject obj = processContour(c, colorMat, depthFrame);
        obj.color = maskColor;
        objects.push_back(obj);
    }
}

void RealSense::detectObjects(cv::Mat &colorMat, rs2::depth_frame &depthFrame){
    // Clear objects vector 
    objects.clear();

    cv::Mat redMask1 = thresholdColor(colorMat, RED_RANGE_1);
    cv::Mat redMask2 = thresholdColor(colorMat, RED_RANGE_2);
    cv::Mat redMask  = redMask1 | redMask2; // combine (wrap around HSV at H = 180)

    cv::Mat blueMask = thresholdColor(colorMat, BLUE_RANGE);
    cv::Mat greenMask = thresholdColor(colorMat, GREEN_RANGE);

    // Cleaning
    cv::erode(redMask, redMask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(redMask, redMask, cv::Mat(), cv::Point(-1,-1), 1);

    cv::erode(blueMask, blueMask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(blueMask, blueMask, cv::Mat(), cv::Point(-1,-1), 1);

    cv::erode(greenMask, greenMask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(greenMask, greenMask, cv::Mat(), cv::Point(-1,-1), 1);

    getContours(redMask, "red", colorMat, depthFrame);
    getContours(blueMask, "blue", colorMat, depthFrame);
    getContours(greenMask, "green", colorMat, depthFrame);
 
    // DEBUG: Draw objects
    for (auto &obj : objects) {
    
        cv::Scalar drawColor = (obj.color == "red") ? cv::Scalar(0,0,255) : cv::Scalar(255,0,0);

        // Draw center
        cv::Point centerPt(static_cast<int>(obj.cx), static_cast<int>(obj.cy));
        cv::circle(colorMat, centerPt, 5, drawColor, -1);

        // If it's a square, also draw orientation
        if (obj.shape == ShapeType::SQUARE) {
            // minAreaRect data is in obj.width / obj.height / obj.orientation
            
            float angle = obj.orientation; 
            // For a quick line to show orientation:
            float length = 50.0f; // line length
            float rad    = angle * CV_PI / 180.0f;
            cv::Point endPt(
                centerPt.x + static_cast<int>(length * std::cos(rad)),
                centerPt.y - static_cast<int>(length * std::sin(rad)) // y minus because screen coords invert angle
            );
            cv::line(colorMat, centerPt, endPt, drawColor, 2);
        }
        
        char text[128];
        if (obj.shape == ShapeType::SQUARE) {
            std::snprintf(text, 128, "%s Square [%.1f deg], Depth=%.2f", 
                          obj.color.c_str(), obj.orientation, obj.avgDepth);
        } else {
            std::snprintf(text, 128, "%s Circle R=%.1f, Depth=%.2f", 
                          obj.color.c_str(), obj.radius, obj.avgDepth);
        }
        cv::putText(colorMat, text, centerPt + cv::Point(5, -5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);
    }
 }