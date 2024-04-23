
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "LogitechC270HD.hpp"
#include "HSVColorRange.hpp"
#include "ContourFeatures.hpp"
#include "TagCandidate.hpp"

// Measure in centimeters
#define TAG_SIZE 12.7

LogitechC270HD camera(0);

struct Configuration {
    int selectedColor = 0;
    std::vector<HSVColorRange> colors {
        HSVColorRange(170, 15, 120, 120), // Red
        HSVColorRange(14, 7, 110, 160), // Orange
        HSVColorRange(27, 7, 120, 160), // Yellow
        HSVColorRange(82, 18, 60, 100), // Green
        HSVColorRange(111, 20, 130, 160), // Blue
        HSVColorRange(140, 10, 60, 110) // Purple / violet
    };
    double areaThreshold = 25;
    double ellipseThreshold = 0.85;
    double triangleThreshold = 0.85;
    double cornerDistanceErrorLimit = 0.3;
    double cornerAreaScaleLimit = 2.5;
    double cornerAngleErrorLimit = CV_PI*25/180;

    HSVColorRange& color() { return colors.at(selectedColor); }
} config;

const std::vector<cv::Point3d> tagObjectPoints {
    cv::Point3d(-TAG_SIZE/2, TAG_SIZE/2, 0),
    cv::Point3d(TAG_SIZE/2, TAG_SIZE/2, 0),
    cv::Point3d(TAG_SIZE/2, -TAG_SIZE/2, 0),
    cv::Point3d(-TAG_SIZE/2, -TAG_SIZE/2, 0)
};

float cameraParameters[] = {
    6.7361123292017487e+02, 0.,                     3.1950000000000000e+02,
    0.,                     6.7361123292017487e+02, 2.3950000000000000e+02,
    0.,                     0.,                     1.
};
cv::Mat cameraMatrix(3, 3, CV_32F, cameraParameters);

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
    std::vector<std::vector<ContourFeatures>> colorFeatures;
    for (int i = 0; i < config.colors.size(); i++) {
        // Mask image with current color
        cv::Mat colorMask;
        config.colors.at(i).inRange(hsv, colorMask);

        // Extract contour features from mask
        std::vector<ContourFeatures> features;
        getContourFeatures(colorMask, features, config.areaThreshold);
        colorFeatures.push_back(std::move(features));

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
        std::vector<ContourFeatures>& features = colorFeatures.at(color);
        for (int i = 0; i < features.size(); i++) {
            ContourFeatures& cf = features.at(i);
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
    std::vector<std::vector<TagCandidate>> tagCandidates {
        std::vector<TagCandidate>(),
        std::vector<TagCandidate>(),
        std::vector<TagCandidate>(),
        std::vector<TagCandidate>(),
        std::vector<TagCandidate>(),
        std::vector<TagCandidate>()
    };
    for (int i = 0; i < 3; i++)
        getTagCandidates(displayFrame, colorFeatures.at(i), colorFeatures.at(i + 3), tagCandidates.at(i), tagCandidates.at(i + 3), config.ellipseThreshold, config.cornerAreaScaleLimit, config.cornerDistanceErrorLimit, config.cornerAngleErrorLimit);

    // Vector for storing poses of tags with 4 detected corners
    std::vector<std::pair<std::vector<double>, std::vector<double>>> tagPoses;

    // For each color
    for (int color = 0; color < 6; color++) {
        // Get candidates and features for one color
        std::vector<TagCandidate>& tcs = tagCandidates.at(color);
        std::vector<ContourFeatures>& ellipseFeatures = colorFeatures.at(color);
        std::vector<ContourFeatures>& cornerFeatures = colorFeatures.at((color + 3)%6);

        // Draw candidates and poses
        for (TagCandidate tc: tcs) {
            // Stores corners of tag
            std::vector<cv::Point2d> cornerPoints;

            // Draw tag center
            ContourFeatures& ellipse = ellipseFeatures.at(tc.ellipseIndex);
            ellipse.drawEllipse(displayFrame, cv::Scalar(255, 255, 255));

            // Draw tag corners and outline
            for (int i = 0; i < tc.corners.size(); i++) {
                CornerCandidate& corner = tc.corners.at(i);
                ContourFeatures& tri = cornerFeatures.at(corner.triangleIndex);
                cv::putText(displayFrame, std::to_string((int) (100*tri.triangularity)), tri.minTriangleCenter, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
                cv::line(displayFrame, ellipse.ellipseRect.center, tri.minTriangleCenter, cv::Scalar(255, 0, 0));

                // Draw corner triangle
                for (int j = 0; j < 3; j++) {
                    cv::Point& p0 = tri.minTriangle.at(j);
                    cv::Point& p1 = tri.minTriangle.at((j + 1)%3);
                    cv::Point diff = p1 - p0;
                    double dist = tc.corners.at(i).edgeDistance[j]*3000;
                    cv::line(displayFrame, p0 - diff, p0 + 2*diff, cv::Scalar(-100 - dist, 400 - dist, dist - 100));
                }

                // Draw tag outline on input frame
                if (tc.corners.size() == 4) {
                    CornerCandidate& corner2 = tc.corners.at((i + 1)%4);
                    ContourFeatures& tri2 = cornerFeatures.at(corner2.triangleIndex);
                    cv::line(frame, 2*tri.minTriangle.at(corner.farIndex), 2*tri2.minTriangle.at(corner2.farIndex), cv::Scalar(0, 255, 255));
                    
                    // Add to cornerPoints if first tag
                    cornerPoints.push_back(2*tri.minTriangle.at(corner.farIndex));
                }
            }

            // If all four corners are present
            if (cornerPoints.size() == 4) {
                // Get pose of tag in camera frame
                std::vector<double> rvec;
                std::vector<double> tvec;
                solvePnP(tagObjectPoints, cornerPoints, cameraMatrix, std::vector<double>(), rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

                // Store pose
                std::pair<std::vector<double>, std::vector<double>> newPose(std::move(rvec), std::move(tvec));
                tagPoses.push_back(std::move(newPose));
            }
        }
    }

    // Display pose of tag
    for (auto pose: tagPoses) {
        // Draw projection
        std::vector<cv::Point2d> projectedPoints;
        cv::projectPoints(tagObjectPoints, pose.first, pose.second, cameraMatrix, std::vector<double>(), projectedPoints);
        for (int i = 0; i < 4; i++)
            cv::line(frame, projectedPoints.at(i), projectedPoints.at((i + 1)%4), cv::Scalar(255, 255, 255));
        cv::drawFrameAxes(frame, cameraMatrix, std::vector<double>(), pose.first, pose.second, 60/pose.second.at(2));

        // Print distance and horizontal angle to camera
        double angle = atan2(pose.second.at(0), pose.second.at(2));
        std::cout << "target at (" << pose.second.at(2) << " cm, " << angle << " rad)" <<  std::endl;
    }

    std::cout << std::endl;

    // Show candidates and geometry
    cv::imshow("candidates", displayFrame);

    // Show input frame
    cv::imshow("camera", frame);
}
