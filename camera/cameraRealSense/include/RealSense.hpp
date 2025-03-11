#pragma once

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "types.hpp"



class RealSense {
    private:
    
    // Camera parameters
    int WIDTH;
    int HEIGHT;
    int FPS;

    // Threshold Values
    ColorRange RED_RANGE_1;
    ColorRange RED_RANGE_2;
    ColorRange BLUE_RANGE;
    ColorRange GREEN_RANGE;

    // Camera pipeline
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align *align_to_color;
    rs2_intrinsics intrinsics;
    bool haveIntrinsics = false;

    // Running
    bool running = false;

    // Objects
    std::vector<DetectedObject> objects;

    // Private methods

    DetectedObject processContour(std::vector<cv::Point> &contour,
        cv::Mat &colorMat,
        rs2::depth_frame &depthFrame);

    
    void getContours(cv::Mat &mask, std::string maskColor, cv::Mat &colorMat, rs2::depth_frame &depthFrame);

    public:
    RealSense(int width = 640, int height = 480, int fps = 30);
    ~RealSense();


    /**
     * @brief Apply color thresholding to an image and return the resulting binary mask
     * @param bgr Input image
     * @param range Color range
     * @return Binary mask
     */
    cv::Mat thresholdColor(const cv::Mat &bgr, const ColorRange &range);

    /**
     * @brief Start the camera
     */ 
    bool start();

    /**
     * @brief Stop the camera
     */
    void stop();

    /**
     * @brief Scan the environment and captures frames
     * @return Number of detected objects
     */
    int scan();


    /**
     * @brief Get the number of detected objects in the last scan
     * @return Number of detected objects
     */
    int getObjectCount();


    /**
     * @brief Get the detected object at the specified index
     * @param index Index of the object
     * @return Detected object 
     */
    DetectedObject getObject(int index);


    void detectObjects(cv::Mat &colorMat, rs2::depth_frame &depthFrame);




    

};

