#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

const char ASCII_ESCAPE = 27;

enum Color {black, red, green, blue, yellow, cyan, magenta, white};

cv::VideoCapture camera;

void setBrightness(int newValue, void* data = 0) {
    camera.set(cv::CAP_PROP_BRIGHTNESS, newValue);
}

void setContrast(int newValue, void* data = 0) {
    camera.set(cv::CAP_PROP_CONTRAST, newValue);
}

void setSaturation(int newValue, void* data = 0) {
    camera.set(cv::CAP_PROP_SATURATION, newValue);
}

void setSharpness(int newValue, void* data = 0) {
    camera.set(cv::CAP_PROP_SHARPNESS, newValue);
}

void setExposure(int newValue, void* data = 0) {
    camera.set(cv::CAP_PROP_EXPOSURE, newValue);
}

void setAutoWhiteBalance(int newValue, void* data = 0) {
    camera.set(cv::CAP_PROP_AUTO_WB, newValue);
}

int hue = 0;
void setHue(int newValue, void* data = 0) {
    hue = newValue;
}

int range = 0;
void setRange(int newValue, void* data = 0) {
    range = newValue;
}

int saturation = 0;
void setMaskSaturation(int newValue, void* data = 0) {
    saturation = newValue;
}

int value = 0;
void setMaskValue(int newValue, void* data = 0) {
    value = newValue;
}

void makeWindows() {
    std::string cameraWindow = "camera";
    cv::namedWindow(cameraWindow);
    cv::createTrackbar("brightness", cameraWindow, NULL, 255, setBrightness);
    cv::createTrackbar("contrast", cameraWindow, NULL, 255, setContrast);
    cv::createTrackbar("saturation", cameraWindow, NULL, 255, setSaturation);
    cv::createTrackbar("sharpness", cameraWindow, NULL, 255, setSharpness);
    cv::createTrackbar("exposure", cameraWindow, NULL, 1000, setExposure);
    cv::createTrackbar("white balance", cameraWindow, NULL, 1, setAutoWhiteBalance);

    std::string maskedWindow = "masked";
    cv::namedWindow(maskedWindow);
    cv::createTrackbar("hue", maskedWindow, NULL, 179, setHue);
    cv::createTrackbar("range", maskedWindow, NULL, 90, setRange);
    cv::createTrackbar("saturation", maskedWindow, NULL, 255, setMaskSaturation);
    cv::createTrackbar("value", maskedWindow, NULL, 255, setMaskValue);
}

bool initializeCameraVideoCapture(int deviceID) {
    camera.open(deviceID, 0);

    // Wait until camera is opened, fail if escape is pressed
    int key;
    while (!camera.isOpened()) {
        key = cv::waitKey(100);
        if (key == ASCII_ESCAPE) {
            return false;
        }
    }

    cv::Mat pic;
    camera >> pic;
    camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);
    camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);

    return true;
}

std::vector<cv::Scalar> findAndGroupCorners() {
    return static_cast<std::vector<cv::Scalar>>(0);
}

int main(int argc, char** argv) {
    initializeCameraVideoCapture(2);

    makeWindows();

    int key = cv::waitKey(16);
    while (key != ASCII_ESCAPE) {
        cv::Mat pic;
        camera >> pic;

        cv::Mat downscaled;
        cv::pyrDown(pic, downscaled, cv::Size(pic.cols / 2, pic.rows / 2));

        cv::Mat hsv;
        cv::cvtColor(downscaled, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask;
        cv::Mat mask1;
        cv::inRange(hsv, cv::Scalar(hue - range, 0, value), cv::Scalar(hue + range, saturation, 255), mask);
        
        // if (hue - range < 0) {
        //     cv::inRange(hsv, cv::Scalar(hue - range + 180, 0, 150), cv::Scalar(hue + range + 180, 255, 255), mask1);
        //     mask = mask1 + mask;
        // } else if (hue + range > 180) {
        //     cv::inRange(hsv, cv::Scalar(hue - range - 180, 0, 150), cv::Scalar(hue + range - 180, 255, 255), mask1);
        //     mask = mask1 + mask;
        // }

        cv::Mat colorRanged;
        cv::bitwise_and(downscaled, downscaled, colorRanged, mask);
        cv::imshow("masked", colorRanged);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

        //for ()
        //cv::ellipse
        cv::imshow("camera", pic);

        key = cv::waitKey(16);
    }
}