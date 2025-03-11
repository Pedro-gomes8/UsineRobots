#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

static const int WIDTH  = 1280;
static const int HEIGHT = 720;
static const int FPS    = 30;

// HSV color range for thresholding
struct ColorRange {
    cv::Scalar lower;
    cv::Scalar upper; 
};

// NOTE: Red in HSV often needs two ranges due to wrap-around near Hue=180 [STACKOVERFLOW]
ColorRange RED_RANGE_1   = {cv::Scalar(0,   130, 130), cv::Scalar(10,  255, 255)};  
ColorRange RED_RANGE_2   = {cv::Scalar(170, 130, 130), cv::Scalar(180, 255, 255)};
ColorRange BLUE_RANGE    = {cv::Scalar(100, 150, 100), cv::Scalar(140, 255, 255)}; 

// Apply color thresholding to an image and return the resulting binary mask
cv::Mat thresholdColor(const cv::Mat &bgr, const ColorRange &range) {
    cv::Mat hsv, mask;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, range.lower, range.upper, mask);
    return mask;
}

int main() {
    try {
        // Real sense Setup
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);
        cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);
        pipe.start(cfg);

        
        rs2::align align_to_color(RS2_STREAM_COLOR);

        // Get camera intrinsics (for 3D deprojection)
        rs2_intrinsics intrinsics;
        bool intrinsicsAcquired = false;

        std::cout << "Press ESC to exit.\n";

        while (true) {
            // Capture frames from camera. Align depth to color
            rs2::frameset frames = pipe.wait_for_frames();
            frames = align_to_color.process(frames);

            rs2::video_frame colorFrame = frames.get_color_frame();
            rs2::depth_frame depthFrame = frames.get_depth_frame();

            if (!colorFrame || !depthFrame) continue;

            // Acquire intrinsics once
            if (!intrinsicsAcquired) {
                auto stream_profile = colorFrame.get_profile().as<rs2::video_stream_profile>();
                intrinsics = stream_profile.get_intrinsics();
                intrinsicsAcquired = true;
            }

            // Convert color frame to OpenCV Mat
            cv::Mat colorMat(cv::Size(WIDTH, HEIGHT), CV_8UC3,
                             (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);

            // B) Create masks for red and blue
            // For red, combine two masks (lower range + upper range) (FROM the WRAP AROUND around 180)
            cv::Mat redMask1 = thresholdColor(colorMat, RED_RANGE_1);
            cv::Mat redMask2 = thresholdColor(colorMat, RED_RANGE_2);
            cv::Mat redMask  = redMask1 | redMask2; // combine

            // For blue
            cv::Mat blueMask = thresholdColor(colorMat, BLUE_RANGE);

            // Reduce noise
            //  we use a kernel of 3x3 -> cv::Mat() and a anchor point in the middle -> cv::Point(-1,-1)
            // We apply erode and dilate only once.
            // Erode will shrink the white regions (to remove small noises)
            // Dilate will expand the white regions (to make them more expressive)
            cv::erode(redMask, redMask, cv::Mat(), cv::Point(-1,-1), 1);
            cv::dilate(redMask, redMask, cv::Mat(), cv::Point(-1,-1), 1);
            
            // Do the same for blue
            cv::erode(blueMask, blueMask, cv::Mat(), cv::Point(-1,-1), 1);
            cv::dilate(blueMask, blueMask, cv::Mat(), cv::Point(-1,-1), 1);

            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;

            cv::findContours(redMask, contours, hierarchy, 
                             cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

            // For each contour, compute bounding circle or bounding box
            for (auto &contour : contours) {
                double area = cv::contourArea(contour);
                if (area < 230.0) { // skip small noise
                    continue;
                }

                cv::Moments m = cv::moments(contour);
                if (m.m00 > 0) {

                    // Gets X coordinate of centroid: given by m.m10/m.m00
                    // Gets Y coordinate of centroid: given by m.m01/m.m00
                    int cx = (int)(m.m10 / m.m00);
                    int cy = (int)(m.m01 / m.m00);

                    // Get average depth in a small region around the centroid of radius sumDepth
                    
                    float sumDepth   = 0.0f;
                    int depthCount = 0;
                    int radiusForDepth = 7; // e.g. 5-pixel 'radius'

                    // Iterates through a square region around the centroid (cx, cy)
                    for (int dy = -radiusForDepth; dy <= radiusForDepth; dy++) {
                        for (int dx = -radiusForDepth; dx <= radiusForDepth; dx++) {
                            int xx = cx + dx;
                            int yy = cy + dy;
                            
                            // opencv has origin in the upper left corner 0,0. So we need to filter out negative values
                            if (xx >= 0 && xx < WIDTH && yy >= 0 && yy < HEIGHT) {
                                float dist = depthFrame.get_distance(xx, yy);
                                if (dist > 0.0f && dist < 50.0f) {
                                    sumDepth += dist;
                                    depthCount++;
                                }
                            }
                        }
                    }
                    // Finally we get the average depth
                    float avgDepth = (depthCount > 0) ? (sumDepth / depthCount) : 0.0f;

                    // Convert (cx, cy, avgDepth) -> 3D (X, Y, Z) 
                    float pixel[2]  = { (float)cx, (float)cy };
                    float point[3]  = {0, 0, 0};
                    if (avgDepth > 0.0f) {
                        rs2_deproject_pixel_to_point(point, &intrinsics, pixel, avgDepth);
                    }

                    // Draw results on image
                    cv::circle(colorMat, cv::Point(cx,cy), 5, cv::Scalar(0,255,0), -1);
                    std::string info = "R: (" + std::to_string(cx) + "," + std::to_string(cy) + ") Depth: " + 
                                       std::to_string(avgDepth) + "m";
                    cv::putText(colorMat, info, cv::Point(cx+5, cy),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1);
                    // cv::Vec3b hsv = colorMat.at<cv::Vec3b>(cx,cy);
                    cv::drawContours(colorMat, contours, -1, cv::Scalar(0,255,0), 5);
                }
            }

            // Repeat for blue objects
            cv::findContours(blueMask, contours, hierarchy,
                             cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            for (auto &contour : contours) {
                double area = cv::contourArea(contour);
                if (area < 150.0) continue;

                cv::Moments m = cv::moments(contour);
                if (m.m00 > 0) {
                    int cx = (int)(m.m10 / m.m00);
                    int cy = (int)(m.m01 / m.m00);

                    // Average depth
                    float sumDepth   = 0.0f;
                    int   depthCount = 0;
                    int radiusForDepth = 7;
                    for (int dy = -radiusForDepth; dy <= radiusForDepth; dy++) {
                        for (int dx = -radiusForDepth; dx <= radiusForDepth; dx++) {
                            int xx = cx + dx;
                            int yy = cy + dy;
                            if (xx >= 0 && xx < WIDTH && yy >= 0 && yy < HEIGHT) {
                                float dist = depthFrame.get_distance(xx, yy);
                                if (dist > 0.0f && dist < 50.0f) {
                                    sumDepth += dist;
                                    depthCount++;
                                }
                            }
                        }
                    }
                    float avgDepth = (depthCount > 0) ? (sumDepth / depthCount) : 0.0f;

                    // 3D coordinate
                    float pixel[2]  = { (float)cx, (float)cy };
                    float point[3]  = {0,0,0};
                    if (avgDepth > 0.0f) {
                        rs2_deproject_pixel_to_point(point, &intrinsics, pixel, avgDepth);
                    }

                    // Draw
                    cv::circle(colorMat, cv::Point(cx,cy), 5, cv::Scalar(255,0,0), -1);
                    std::string info = "B: (" + std::to_string(cx) + "," + std::to_string(cy) + ") Depth: " + 
                                       std::to_string(avgDepth) + "m";
                    cv::putText(colorMat, info, cv::Point(cx+5, cy),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0), 1);
                    cv::drawContours(colorMat, contours, -1, cv::Scalar(255,0,0), 5);

                }
            }

            // Show result
            cv::imshow("Color + Depth Debug", colorMat);
            if (cv::waitKey(1) == 27) { // ESC
                break;
            }
        }
    }
    catch(const rs2::error &e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch(const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
