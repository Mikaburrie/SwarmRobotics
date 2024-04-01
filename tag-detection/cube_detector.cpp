
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "LogitechC270HD.hpp"
#include "ColorRange.hpp"
#include "ContourFeatures.hpp"
#include "TagCandidate.hpp"

// Measures in inches
#define TAG_SIZE 5.0

LogitechC270HD camera(0);

struct Configuration {
    int selectedColor = 0;
    std::vector<ColorRange> colors {
        ColorRange(170, 15, 120, 120), // Red
        ColorRange(8, 7, 110, 240), // Orange
        ColorRange(21, 7, 140, 190), // Yellow
        ColorRange(82, 18, 60, 100), // Green
        ColorRange(111, 20, 170, 200), // Blue
        ColorRange(133, 10, 130, 160) // Purple / violet
    };
    double areaThreshold = 25;
    double ellipseThreshold = 0.85;
    double triangleThreshold = 0.85;
    double cornerDistanceErrorLimit = 0.3;
    double cornerAreaScaleLimit = 2.5;
    double cornerAngleErrorLimit = CV_PI*25/180;

    ColorRange& color() { return colors.at(selectedColor); }
} config;

void makeWindows();
void processFrame(const cv::Mat&);

int main(int argc, char** argv) {
    // Creates windows for displaying images
    makeWindows();

    int key = cv::waitKey(16);
    while (key != 27) { // Exit if Escape is pressed
        // Get frame from camera and display
        cv::Mat frame;
        camera >> frame;

        processFrame(frame);

        key = cv::waitKey(16);
    }
}

void makeWindows() {
    // Create camera frame window and associated trackbars
    std::string cameraWindow = "camera";
    cv::namedWindow(cameraWindow);
    cv::moveWindow(cameraWindow, 0, 0);

    cv::createTrackbar("exposure", cameraWindow, NULL, 1000, [](int exposure, void* data=0) {
        camera.setExposure(exposure);
    });
    cv::setTrackbarPos("exposure", cameraWindow, camera.getExposure());

    cv::createTrackbar("enable auto exposure", cameraWindow, NULL, 1, [](int enabled, void* data=0) {
        camera.setAutoExposure(enabled);
        cv::setTrackbarPos("exposure", "camera", camera.getExposure());
    });
    cv::setTrackbarPos("enable auto exposure", cameraWindow, camera.getAutoExposure());

    cv::createTrackbar("auto white balance", cameraWindow, NULL, 1, [](int enabled, void* data=0) {
        camera.setAutoWhiteBalance(enabled);
    });
    cv::setTrackbarPos("auto white balance", cameraWindow, camera.getAutoWhiteBalance());

    // Create masking window and associated trackbars
    std::string maskingWindow = "masking";
    cv::namedWindow(maskingWindow);
    cv::moveWindow(maskingWindow, 720, 0);

    cv::createTrackbar("color", maskingWindow, NULL, 5, [](int newColor, void* data = 0) {
        config.selectedColor = newColor;
        std::string maskingWindow = "masking";
        cv::setTrackbarPos("hue", maskingWindow, config.color().hue);
        cv::setTrackbarPos("range", maskingWindow, config.color().range);
        cv::setTrackbarPos("saturation", maskingWindow, config.color().saturation);
        cv::setTrackbarPos("value", maskingWindow, config.color().value);
    });
    cv::setTrackbarPos("color", maskingWindow, config.selectedColor);

    cv::createTrackbar("hue", maskingWindow, NULL, 180, [](int hue, void* data=0) {
        config.color().setHue(hue);
    });
    cv::setTrackbarPos("hue", maskingWindow, config.color().hue);

    cv::createTrackbar("range", maskingWindow, NULL, 90, [](int range, void* data=0) {
        config.color().setRange(range);
    });
    cv::setTrackbarPos("range", maskingWindow, config.color().range);

    cv::createTrackbar("saturation", maskingWindow, NULL, 255, [](int saturation, void* data=0) {
        config.color().setSaturation(saturation);
    });
    cv::setTrackbarPos("saturation", maskingWindow, config.color().saturation);

    cv::createTrackbar("value", maskingWindow, NULL, 255, [](int value, void* data=0) {
        config.color().setValue(value);
    });
    cv::setTrackbarPos("value", maskingWindow, config.color().value);

    // Create feature window and associated trackbars
    std::string featureWindow = "features";
    cv::namedWindow(featureWindow);
    cv::moveWindow(featureWindow, 1080, 0);

    cv::createTrackbar("area threshold", featureWindow, NULL, 400, [](int threshold, void* data=0) {
        config.areaThreshold = (double) threshold;
    });
    cv::setTrackbarPos("area threshold", featureWindow, config.areaThreshold);

    cv::createTrackbar("ellipse threshold", featureWindow, NULL, 100, [](int threshold, void* data=0) {
        config.ellipseThreshold = threshold/100.0;
    });
    cv::setTrackbarPos("ellipse threshold", featureWindow, config.ellipseThreshold*100);

    cv::createTrackbar("triangle threshold", featureWindow, NULL, 100, [](int threshold, void* data=0) {
        config.triangleThreshold = threshold/100.0;
    });
    cv::setTrackbarPos("triangle threshold", featureWindow, config.triangleThreshold*100);

    // Create candidate window and associated trackbars
    std::string candidateWindow = "candidates";
    cv::namedWindow(candidateWindow);
    cv::moveWindow(candidateWindow, 1080, 485);

    cv::createTrackbar("corner distance error limit", candidateWindow, NULL, 200, [](int limit, void* data=0) {
        config.cornerDistanceErrorLimit = limit/100.0;
    });
    cv::setTrackbarPos("corner distance error limit", candidateWindow, config.cornerDistanceErrorLimit*100);

    cv::createTrackbar("corner area scale limit", candidateWindow, NULL, 1000, [](int threshold, void* data=0) {
        config.cornerAreaScaleLimit = threshold/100.0;
    });
    cv::setTrackbarPos("corner area scale limit", candidateWindow, config.cornerAreaScaleLimit*100);

    cv::createTrackbar("corner angle error limit", candidateWindow, NULL, 45, [](int threshold, void* data=0) {
        config.cornerAngleErrorLimit = threshold*CV_PI/180;
    });
    cv::setTrackbarPos("corner angle error limit", candidateWindow, config.cornerAngleErrorLimit*180/CV_PI);
}

void processFrame(const cv::Mat& frame) {
    // Downscale image and convert from RGB to HSV
    cv::Mat blurFrame;
    cv::Mat quarterFrame;
    cv::Mat hsv;
    cv::GaussianBlur(frame, blurFrame, cv::Size(3, 3), 1, 1);
    cv::pyrDown(blurFrame, quarterFrame, cv::Size(frame.cols/2, frame.rows/2));
    cv::cvtColor(quarterFrame, hsv, cv::COLOR_BGR2HSV);

    // Create mats for displaying
    cv::Mat allMask(quarterFrame.rows, quarterFrame.cols, CV_8U);
    cv::Mat displayFrame;
    allMask = 0;

    // Extract all color features
    std::vector<std::vector<ContourFeatures>> features;
    for (int i = 0; i < config.colors.size(); i++) {
        // Mask image with current color
        cv::Mat colorMask;
        config.colors.at(i).inRange(hsv, colorMask);

        // Extract contour features from mask
        std::vector<ContourFeatures> colorFeatures;
        getContourFeatures(colorMask, colorFeatures, config.areaThreshold);
        features.push_back(std::move(colorFeatures));

        // Add to total mask and display in masking window if color is selected
        allMask += colorMask;
        if (i == config.selectedColor) {
            cv::bitwise_and(quarterFrame, quarterFrame, displayFrame, colorMask);
            cv::imshow("masking", displayFrame);
        }
    }

    // Put all masked colors in featureFrame
    cv::bitwise_and(quarterFrame, quarterFrame, displayFrame, allMask);

    // Draw all contour features
    for (int color = 0; color < config.colors.size(); color++) {
        for (int i = 0; i < features.at(color).size(); i++) {
            ContourFeatures& cf = features.at(color).at(i);
            bool isEllipse = cf.ellipseness > config.ellipseThreshold;
            bool isTriangle = cf.triangularity > config.triangleThreshold;

            if (isEllipse) {
                cf.drawEllipse(displayFrame, cv::Scalar(0, 0, 255));
            }

            if (isTriangle) {
                cf.drawTriangle(displayFrame, cv::Scalar(0, 255, 0));
            }

            if (!isEllipse && !isTriangle) {
                cf.drawHull(displayFrame, cv::Scalar(255, 0, 0));
            }
        }

    }

    // Show masked colors and features
    cv::imshow("features", displayFrame);
    
    // Put all masked colors in featureFrame
    cv::bitwise_and(quarterFrame, 0, displayFrame);
    cv::bitwise_and(quarterFrame, quarterFrame, displayFrame, allMask);

    // Get tag candidates from contours
    std::vector<TagCandidate> candidates;
    getTagCandidates(displayFrame, features.at(0), features.at(3), candidates, config.ellipseThreshold, config.cornerAreaScaleLimit, config.cornerDistanceErrorLimit, config.cornerAngleErrorLimit);

    // Draw candidates
    for (TagCandidate c: candidates) {
        ContourFeatures& ellipse = features.at(0).at(c.ellipseIndex);
        ellipse.drawEllipse(displayFrame, cv::Scalar(255, 255, 255));
        for (int i = 0; i < c.corners.size(); i++) {
            ContourFeatures& triangle = features.at(3).at(c.corners.at(i).triangleIndex);
            cv::putText(displayFrame, std::to_string(triangle.triangularity), triangle.minTriangleCenter, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
        //    cv::line(displayFrame, ellipse.ellipseRect.center, triangle.minTriangleCenter, cv::Scalar(255, 0, 0));

            for (int i = 0; i < 3; i++) {
                cv::Point diff = triangle.minTriangle.at((i + 1)%3) - triangle.minTriangle.at(i);
                cv::line(displayFrame, triangle.minTriangle.at(i) - diff, triangle.minTriangle.at(i) + 2*diff, cv::Scalar(0, 255, 0));
            }

            // Draw outline on input frame
            if (c.corners.size() == 4) {
                ContourFeatures& tri2 = features.at(3).at(c.corners.at((i + 1)%4).triangleIndex);
                cv::line(frame, 2*triangle.minTriangle.at(c.corners.at(i).farIndex), 2*tri2.minTriangle.at(c.corners.at((i + 1)%4).farIndex), cv::Scalar(255, 255, 255));
            }
        }
    }

    cv::imshow("candidates", displayFrame);

    // Show input frame
    cv::imshow("camera", frame);

    //    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
}
