#ifndef OCTO_TAG_DETECTOR_HPP
#define OCTO_TAG_DETECTOR_HPP

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "HSVColorRange.hpp"
#include "ContourFeatures.hpp"
#include "TagCandidate.hpp"

/*
    Configuration struct for OctoTag detection
*/
struct OctoTagConfiguration {
    // Settings for color masking
    int selectedColor = 0;
    std::vector<HSVColorRange> colors {
        HSVColorRange(175, 15, 120, 120), // Red
        HSVColorRange(14, 4, 130, 160), // Orange
        HSVColorRange(27, 7, 120, 160), // Yellow
        HSVColorRange(60, 18, 60, 100), // Green
        HSVColorRange(111, 20, 70, 120), // Blue
        HSVColorRange(155, 20, 55, 110) // Purple / violet
    };

    // Settings for detecting geometry
    double areaThreshold = 25;
    double ellipseThreshold = 0.85;
    double triangleThreshold = 0.85;

    // Setting for correlating tag geometry
    double cornerDistanceErrorLimit = 0.3;
    double cornerAreaScaleLimit = 2.5;
    double cornerAngleErrorLimit = CV_PI*25/180;

    // Settings for pose calculation
    cv::Mat cameraMatrix;
    double tagSize; // default 5 inches (12.7 cm)
    const std::vector<cv::Point3d> tagObjectPoints {
        cv::Point3d(-tagSize/2, tagSize/2, 0),
        cv::Point3d(tagSize/2, tagSize/2, 0),
        cv::Point3d(tagSize/2, -tagSize/2, 0),
        cv::Point3d(-tagSize/2, -tagSize/2, 0)
    };

    // Names for display windows
    std::string maskingWindow;
    std::string featureWindow;
    std::string candidateWindow;
    bool drawTagPoses;

    // Constructor that accepts camera parameter matrix and tagSize in cm
    OctoTagConfiguration(float cameraParameters[9], double tagSize = 12.7) :
    cameraMatrix(3, 3, CV_32F, cameraParameters),
    tagSize(tagSize),
    tagObjectPoints({
        cv::Point3d(-tagSize/2, tagSize/2, 0),
        cv::Point3d(tagSize/2, tagSize/2, 0),
        cv::Point3d(tagSize/2, -tagSize/2, 0),
        cv::Point3d(-tagSize/2, -tagSize/2, 0)
    })
    {}

    HSVColorRange& getSelectedColor() { return colors.at(selectedColor); }
};

/*
    Struct for detected OctoTag
*/
struct OctoTag {
    int color;
    std::vector<double> rvec;
    std::vector<double> tvec;
    cv::Point3d normal;

    // Draw detected pose
    void drawPose(cv::InputOutputArray img, const OctoTagConfiguration& config) {
        std::vector<cv::Point2d> projectedPoints;
        cv::projectPoints(config.tagObjectPoints, rvec, tvec, config.cameraMatrix, std::vector<double>(), projectedPoints);
        for (int i = 0; i < 4; i++)
            cv::line(img, projectedPoints.at(i), projectedPoints.at((i + 1)%4), cv::Scalar(255, 255, 255));
        cv::drawFrameAxes(img, config.cameraMatrix, std::vector<double>(), rvec, tvec, 60/tvec.at(2));
    }

};

/*
    Detects OctoTags in a frame
*/
void detectOctoTags(const cv::Mat& frame, const OctoTagConfiguration& config, std::vector<OctoTag>& tags) {
    bool drawMasking = config.maskingWindow.size() > 0;
    bool drawFeatures = config.featureWindow.size() > 0;
    bool drawCandidates = config.candidateWindow.size() > 0;

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

        // Add to total mask
        allMask += colorMask;

        // Display in masking window if color is selected
        if (i == config.selectedColor && drawMasking) {
            cv::bitwise_and(quarterFrame, quarterFrame, displayFrame, colorMask);
            cv::imshow(config.maskingWindow, displayFrame);
        }
    }

    // Draw all ContourFeatures if featureWindow is provided 
    if (drawFeatures) {
        // Put all masked colors in displayFrame
        cv::bitwise_and(quarterFrame, quarterFrame, displayFrame, allMask);

        // Draw all contour features
        for (int color = 0; color < config.colors.size(); color++) {
            std::vector<ContourFeatures>& features = colorFeatures.at(color);
            for (int i = 0; i < features.size(); i++) {
                ContourFeatures& cf = features.at(i);
                bool isEllipse = cf.ellipseness > config.ellipseThreshold;
                bool isTriangle = cf.triangularity > config.triangleThreshold;

                if (isEllipse) cf.drawEllipse(displayFrame, cv::Scalar(0, 0, 255));
                if (isTriangle) cf.drawTriangle(displayFrame, cv::Scalar(0, 255, 0));
                if (!isEllipse && !isTriangle) cf.drawHull(displayFrame, cv::Scalar(255, 0, 0));
            }
        }

        // Show masked colors and features
        cv::imshow(config.featureWindow, displayFrame);
    }
    
    // Put all masked colors in displayFrame
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
        getTagCandidates(colorFeatures.at(i), colorFeatures.at(i + 3), tagCandidates.at(i), tagCandidates.at(i + 3), config.ellipseThreshold, config.cornerAreaScaleLimit, config.cornerDistanceErrorLimit, config.cornerAngleErrorLimit);

    // For each color
    for (int color = 0; color < 6; color++) {
        // Get candidates and features for one color
        std::vector<TagCandidate>& tcs = tagCandidates.at(color);
        std::vector<ContourFeatures>& ellipseFeatures = colorFeatures.at(color);
        std::vector<ContourFeatures>& cornerFeatures = colorFeatures.at((color + 3)%6);

        // Get poses and draw candidates
        for (TagCandidate tc: tcs) {
    
            // Get tag pose if four corners are detected
            if (tc.corners.size() == 4) {
                // Get corner points of tag
                std::vector<cv::Point2d> cornerPoints;
                for (int i = 0; i < 4; i++) {
                    CornerCandidate& corner = tc.corners.at(i);
                    ContourFeatures& tri = cornerFeatures.at(corner.triangleIndex);
                    cornerPoints.push_back(2*tri.minTriangle.at(corner.farIndex));
                }

                // Create tag object
                OctoTag tag;
                tag.color = color;

                // Get pose of tag in camera frame
                solvePnP(config.tagObjectPoints, cornerPoints, config.cameraMatrix, std::vector<double>(), tag.rvec, tag.tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

                // Get rotation angle and axis (kvec) from rvec
                double theta = cv::norm(tag.rvec);
                cv::Point3d kvec(
                    tag.rvec.at(0)/theta,
                    tag.rvec.at(1)/theta,
                    tag.rvec.at(2)/theta
                );

                // Perform rotation to find normal
                cv::Point3d normal(0, 0, 1);
                tag.normal = normal*cos(theta) + kvec.cross(normal)*sin(theta) + kvec*kvec.dot(normal)*(1 - cos(theta));

                // Store tag
                tags.push_back(std::move(tag));
            }

            // Draw candidates if candidateWindow is provided
            if (drawCandidates) {
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
                }
            }
        }
    }
    
    // Show candidates
    if (drawCandidates) cv::imshow(config.candidateWindow, displayFrame);
}

#endif