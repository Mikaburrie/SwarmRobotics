
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "LogitechC270HD.hpp"
#include "OctoTagDetector.hpp"

#include "../motor-control/motor_command.h"

LogitechC270HD camera(0);
OctoTagConfiguration config(camera.parameters);

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
    config.maskingWindow = "masking";
    cv::namedWindow(config.maskingWindow);
    cv::moveWindow(config.maskingWindow, 720, 0);

    cv::createTrackbar("color", config.maskingWindow, NULL, 5, [](int newColor, void* data = 0) {
        config.selectedColor = newColor;
        cv::setTrackbarPos("hue", config.maskingWindow, config.getSelectedColor().hue);
        cv::setTrackbarPos("range", config.maskingWindow, config.getSelectedColor().range);
        cv::setTrackbarPos("saturation", config.maskingWindow, config.getSelectedColor().saturation);
        cv::setTrackbarPos("value", config.maskingWindow, config.getSelectedColor().value);
    });
    cv::setTrackbarPos("color", config.maskingWindow, config.selectedColor);

    cv::createTrackbar("hue", config.maskingWindow, NULL, 180, [](int hue, void* data=0) {
        config.getSelectedColor().setHue(hue);
    });
    cv::setTrackbarPos("hue", config.maskingWindow, config.getSelectedColor().hue);

    cv::createTrackbar("range", config.maskingWindow, NULL, 90, [](int range, void* data=0) {
        config.getSelectedColor().setRange(range);
    });
    cv::setTrackbarPos("range", config.maskingWindow, config.getSelectedColor().range);

    cv::createTrackbar("saturation", config.maskingWindow, NULL, 255, [](int saturation, void* data=0) {
        config.getSelectedColor().setSaturation(saturation);
    });
    cv::setTrackbarPos("saturation", config.maskingWindow, config.getSelectedColor().saturation);

    cv::createTrackbar("value", config.maskingWindow, NULL, 255, [](int value, void* data=0) {
        config.getSelectedColor().setValue(value);
    });
    cv::setTrackbarPos("value", config.maskingWindow, config.getSelectedColor().value);

    // Create feature window and associated trackbars
    config.featureWindow = "features";
    cv::namedWindow(config.featureWindow);
    cv::moveWindow(config.featureWindow, 1080, 0);

    cv::createTrackbar("area threshold", config.featureWindow, NULL, 400, [](int threshold, void* data=0) {
        config.areaThreshold = (double) threshold;
    });
    cv::setTrackbarPos("area threshold", config.featureWindow, config.areaThreshold);

    cv::createTrackbar("ellipse threshold", config.featureWindow, NULL, 100, [](int threshold, void* data=0) {
        config.ellipseThreshold = threshold/100.0;
    });
    cv::setTrackbarPos("ellipse threshold", config.featureWindow, config.ellipseThreshold*100);

    cv::createTrackbar("triangle threshold", config.featureWindow, NULL, 100, [](int threshold, void* data=0) {
        config.triangleThreshold = threshold/100.0;
    });
    cv::setTrackbarPos("triangle threshold", config.featureWindow, config.triangleThreshold*100);

    // Create candidate window and associated trackbars
    config.candidateWindow = "candidates";
    cv::namedWindow(config.candidateWindow);
    cv::moveWindow(config.candidateWindow, 1080, 485);

    cv::createTrackbar("corner distance error limit", config.candidateWindow, NULL, 200, [](int limit, void* data=0) {
        config.cornerDistanceErrorLimit = limit/100.0;
    });
    cv::setTrackbarPos("corner distance error limit", config.candidateWindow, config.cornerDistanceErrorLimit*100);

    cv::createTrackbar("corner area scale limit", config.candidateWindow, NULL, 1000, [](int threshold, void* data=0) {
        config.cornerAreaScaleLimit = threshold/100.0;
    });
    cv::setTrackbarPos("corner area scale limit", config.candidateWindow, config.cornerAreaScaleLimit*100);

    cv::createTrackbar("corner angle error limit", config.candidateWindow, NULL, 45, [](int threshold, void* data=0) {
        config.cornerAngleErrorLimit = threshold*CV_PI/180;
    });
    cv::setTrackbarPos("corner angle error limit", config.candidateWindow, config.cornerAngleErrorLimit*180/CV_PI);
}

void followTheLeader(double distance, double angle) {
    
    int LMS = 0;
    int RMS = 0;
    
    double turn = 1 - abs(angle / 2.0);
    int forward = 0;
    
    if (distance > 25) {
        forward = (int) (distance * 7);
        if (forward > 200) {
            forward = 200;
        }
    }
    
    LMS = forward;
    RMS = forward;
    
    if (angle > 0) {
        RMS *= turn;
    } else {
        LMS *= turn;
    }
    
    std::cout << "RMS = " << RMS << " | LMS =  " << LMS << std::endl;
    sendMotorCommand(RMS, -1 * LMS);
}

int main(int argc, char** argv) {
    // Creates windows for displaying images
    makeWindows();
    
    // Open FIFO pipe for motor control
    openMotorControlFifo();

    int key = cv::waitKey(16);
    while (key != 27) { // Exit if Escape is pressed
        // Get frame from camera
        cv::Mat frame;
        camera >> frame;
        
        // Skip if frame is empty
        if (frame.empty()) {
            key = cv::waitKey(16);
            continue;
        }

        // Get tags in frame
        std::vector<OctoTag> tags;
        detectOctoTags(frame, config, tags);

        // Print distance and horizontal angle to camera
        for (OctoTag tag: tags) {
            tag.drawPose(frame, config);
            double angle = atan2(tag.tvec.at(0), tag.tvec.at(2));
            std::cout << "color " << tag.color << " target at (" << tag.tvec.at(2) << " cm, " << angle << " rad)" <<  std::endl;
        }

        // Drive motors if tag is detected
        if (tags.size() > 0) {
            double distance = tags[0].tvec.at(2);
            double angle = atan2(tags[0].tvec.at(0), distance);
            followTheLeader(distance, angle);
        } else sendMotorCommand(0, 0);

        std::cout << std::endl;

        cv::imshow("camera", frame);

        key = cv::waitKey(16);
    }
    
    // Close FIFO pipe
    closeMotorControlFifo();
}
