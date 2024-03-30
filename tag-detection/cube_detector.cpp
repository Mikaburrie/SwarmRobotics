
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "ColorRange.hpp"
#include "ContourFeatures.hpp"
#include "LogitechC270HD.hpp"

// Measures in inches
#define TAG_SIZE 5.0
#define TAG_CORNER_LENGTH 1.5
#define TAG_CORNER_AREA (TAG_CORNER_LENGTH*TAG_CORNER_LENGTH/2)
#define TAG_CENTER_AREA (TAG_SIZE*TAG_SIZE - 4*TAG_CORNER_AREA)
#define TAG_CORNER_CENTER_RATIO (TAG_CORNER_AREA/TAG_CENTER_AREA)

#define MINIMUM_AREA 50
#define ELLIPSE_THRESHOLD 0.85
#define TRIANGLE_THRESHOLD 0.85
#define CORNER_RADIUS_THRESHOLD 0.2
#define CORNER_AREA_LIMIT 2

void processFrame(const cv::Mat&);
void drawFeatures(cv::InputArray, ContourFeatures&);
void makeWindows();

LogitechC270HD camera(2);

int selectedColor = 0;
std::vector<ColorRange> colors {
    ColorRange(170, 15, 120, 120), // Red
    ColorRange(8, 7, 110, 240), // Orange
    ColorRange(21, 7, 140, 190), // Yellow
    ColorRange(82, 18, 60, 100), // Green
    ColorRange(111, 20, 170, 200), // Blue
    ColorRange(133, 10, 130, 160) // Purple / violet
};

int main(int argc, char** argv) {
    const char ASCII_ESCAPE = 27;

    // Creates windows for displaying images
    makeWindows();

    int key = cv::waitKey(16);
    while (key != ASCII_ESCAPE) {
        // Get frame from camera and display
        cv::Mat frame;
        camera >> frame;
        cv::imshow("camera", frame);

        processFrame(frame);

        key = cv::waitKey(16);
    }
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
    for (int i = 0; i < colors.size(); i++) {
        // Mask image with current color
        cv::Mat colorMask;
        colors.at(i).inRange(hsv, colorMask);

        // Extract contour features from mask
        std::vector<ContourFeatures> colorFeatures;
        getContourFeatures(colorMask, colorFeatures, MINIMUM_AREA);
        features.push_back(std::move(colorFeatures));

        // Add to total mask and display in masking window if color is selected
        allMask += colorMask;
        if (i == selectedColor) {
            cv::bitwise_and(quarterFrame, quarterFrame, displayFrame, colorMask);
            cv::imshow("masking", displayFrame);
        }
    }

    // Put all masked colors in featureFrame
    cv::bitwise_and(quarterFrame, quarterFrame, displayFrame, allMask);

    // Find tag
    for (int i = 0; i < features.at(0).size(); i++) {
        ContourFeatures ellipse = features.at(0).at(i);

        // Discard if not ellipsoid enough
        if (ellipse.ellipseness < ELLIPSE_THRESHOLD) continue;

        // Determine basis vectors of ellipse
        cv::RotatedRect rect = ellipse.ellipseRect;
        double ellipseAngle = rect.angle*CV_PI/180;
        cv::Point2f ellipseBasis[] {
            cv::Point2f(cos(ellipseAngle), sin(ellipseAngle)),
            cv::Point2f(-sin(ellipseAngle), cos(ellipseAngle))
        };

        double triangleMaxArea = ellipse.area*TAG_CORNER_CENTER_RATIO*CORNER_AREA_LIMIT;
        ellipse.drawEllipse(displayFrame, cv::Scalar(0, 0, 255));

        // Vector storing indicies and angles
        std::vector<std::pair<int, double>> candidates;

        for (int j = 0; j < features.at(3).size(); j++) {
            ContourFeatures triangle = features.at(3).at(j);

            // Discard if too big
            if (triangle.area > triangleMaxArea) continue;

            // Calculate unit vector pointing from ellipse to triangle
            cv::Point2f diff = triangle.minTriangleCenter - rect.center;
            double measuredRadius = cv::norm(diff);
            diff /= measuredRadius;

            // Find radius of ellipse
            double dx = diff.dot(ellipseBasis[0])*rect.size.height;
            double dy = diff.dot(ellipseBasis[1])*rect.size.width;
            double angle = atan2(dy, dx);
            double x = rect.size.width/2*cos(angle);
            double y = rect.size.height/2*sin(angle);
            double ellipseRadius = sqrt(x*x + y*y);

            // Discard if too far away from ellipse
            double radiusError = (measuredRadius - ellipseRadius)/ellipseRadius;
            if (radiusError > CORNER_RADIUS_THRESHOLD) continue;

            // Store candidate
            candidates.push_back(std::pair<int, double>(j, angle));
            
            triangle.drawTriangle(displayFrame, cv::Scalar(0, 255, 0));
            cv::line(displayFrame, rect.center, rect.center + diff*measuredRadius, cv::Scalar(255, 0, 0));
            cv::drawMarker(displayFrame, rect.center + diff*ellipseRadius, cv::Scalar(255, 0, 0), cv::MARKER_CROSS);
        }

        // Find two pairs of triangles with opposite angles
        std::tuple<int, int, double> pairs[2] {
            {-1, -1, 100.0},
            {-1, -1, 100.0}
        };

        for (int j = 0; j < candidates.size(); j++) {
            for (int k = j + 1; k < candidates.size(); k++) {
                double diff = candidates.at(k).second - candidates.at(j).second;
                double error = abs(abs(diff) - CV_PI);

                int index = -1;

                if (error < std::get<2>(pairs[0])) {
                    index = 0;
                    pairs[1].swap(pairs[0]);
                } else if (error < std::get<2>(pairs[1])) {
                    index = 1;
                } else continue;

                std::get<0>(pairs[index]) = j;
                std::get<1>(pairs[index]) = k;
                std::get<2>(pairs[index]) = error;
            }
        }

        std::cout << std::get<2>(pairs[0]) << " " << std::get<2>(pairs[1]);

        std::cout << std::endl;
    }

    cv::imshow("aggregate", displayFrame);
    
    // Put all masked colors in featureFrame
    cv::bitwise_and(quarterFrame, 0, displayFrame);
    cv::bitwise_and(quarterFrame, quarterFrame, displayFrame, allMask);

    for (int color = 0; color < colors.size(); color++) {
        for (int i = 0; i < features.at(color).size(); i++) {
            ContourFeatures cf = features.at(color).at(i);
            bool isEllipse = cf.ellipseness > ELLIPSE_THRESHOLD;
            bool isTriangle = cf.triangularity > TRIANGLE_THRESHOLD;

            // Draw contour features
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

    //    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
}

void makeWindows() {
    // Create camera frame window and associated trackbars
    std::string cameraWindow = "camera";
    cv::namedWindow(cameraWindow);
    cv::moveWindow(cameraWindow, 0, 0);

    cv::createTrackbar("exposure", cameraWindow, NULL, 1000, [](int exposure, void* data=0) {
        camera.setExposure(exposure);
    });

    cv::createTrackbar("enable auto exposure", cameraWindow, NULL, 1, [](int enabled, void* data=0) {
        camera.setAutoExposure(enabled);
    });

    cv::createTrackbar("white balance", cameraWindow, NULL, 1, [](int enabled, void* data=0) {
        camera.setAutoWhiteBalance(enabled);
    });

    // Create masking window and associated trackbars
    std::string maskingWindow = "masking";
    cv::namedWindow(maskingWindow);
    cv::moveWindow(maskingWindow, 720, 0);

    cv::createTrackbar("color", maskingWindow, NULL, 5, [](int newColor, void* data = 0) {
        selectedColor = newColor;
        std::string maskingWindow = "masking";
        cv::setTrackbarPos("hue", maskingWindow, colors[selectedColor].hue);
        cv::setTrackbarPos("range", maskingWindow, colors[selectedColor].range);
        cv::setTrackbarPos("saturation", maskingWindow, colors[selectedColor].saturation);
        cv::setTrackbarPos("value", maskingWindow, colors[selectedColor].value);
    });

    cv::createTrackbar("hue", maskingWindow, NULL, 180, [](int newHue, void* data=0) {
        colors[selectedColor].setHue(newHue);
    });

    cv::createTrackbar("range", maskingWindow, NULL, 90, [](int newRange, void* data=0) {
        colors[selectedColor].setRange(newRange);
    });

    cv::createTrackbar("saturation", maskingWindow, NULL, 255, [](int newSaturation, void* data=0) {
        colors[selectedColor].setSaturation(newSaturation);
    });

    cv::createTrackbar("value", maskingWindow, NULL, 255, [](int newValue, void* data=0) {
        colors[selectedColor].setValue(newValue);
    });

    std::string featureWindow = "features";
    cv::namedWindow(featureWindow);
    cv::moveWindow(featureWindow, 1080, 0);

    std::string aggregateWindow = "aggregate";
    cv::namedWindow(aggregateWindow);
    cv::moveWindow(aggregateWindow, 1080, 400);
}
