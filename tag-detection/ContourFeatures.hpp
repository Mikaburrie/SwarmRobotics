#ifndef CONTOUR_FEATURES_HPP
#define CONTOUR_FEATURES_HPP

#include "opencv2/imgproc.hpp"

// Declarations
struct ContourFeatures;
void getMaskContourFeatures(cv::InputArray, std::vector<ContourFeatures>&);

/*
Struct for storing features of a contour.
Calculates triangularity and 'ellipseness' (0.0 to 1.0) of a given contour
as well as the best fit triangle and ellipse.
*/
struct ContourFeatures {
    double area = 0;
    double triangularity = 0;
    double ellipseness = 0;
    cv::Point hullCenter;
    cv::Point2f minTriangleCenter;
    cv::RotatedRect ellipseRect;
    std::vector<cv::Point> hull;
    std::vector<cv::Point> minTriangle;

    ContourFeatures() : area(0), triangularity(0), ellipseness(0) {}

    ContourFeatures(std::vector<cv::Point> contour, double _area = -1) {
        area = (_area == -1 ? cv::contourArea(contour) : _area);
        triangularity = 0;
        ellipseness = 0;

        // Find convex hull around contour
        cv::convexHull(contour, hull);

        // Find center of hull
        hullCenter.x = 0;
        hullCenter.y = 0;
        for (int i = 0; i < hull.size(); i++)
            hullCenter += hull.at(i);

        hullCenter /= (float) hull.size();

        // minEnclosingTriangle requires 3 points
        if (hull.size() < 3) return;

        // Find minimum enclosing triangle and trianglularity of contour
        cv::minEnclosingTriangle(hull, minTriangle);
        if (minTriangle.size() == 3) {
            minTriangleCenter = (minTriangle.at(0) + minTriangle.at(1) + minTriangle.at(2))/3.0;
            triangularity = area/cv::contourArea(minTriangle);
        }
        
        // fitEllipseAMS requires 5 points
        if (hull.size() < 5) return;

        // Fit ellipse to hull
        // Effective at predicting partially occluded sides.
        // TODO: expand this to detect partial sides.
        std::vector<cv::Point2f> vertices;
        ellipseRect = cv::fitEllipseAMS(hull);

        // Check ellipseness of hull (ratio of area to area of ellipse approximation)
        ellipseness = area*4/(CV_PI*ellipseRect.size.area());
    }

    void drawEllipse(cv::InputOutputArray img, const cv::Scalar& color) {
        cv::ellipse(img, ellipseRect, color);
        cv::drawMarker(img, ellipseRect.center, color, cv::MARKER_STAR);
    }

    void drawTriangle(cv::InputOutputArray img, const cv::Scalar& color) {
        if (minTriangle.size() != 3) return;

        // Draw triangle and center point
        for (int i = 0; i < 3; i++)
            cv::line(img, minTriangle.at(i), minTriangle.at((i + 1)%3), color);

        cv::drawMarker(img, minTriangleCenter, color, cv::MARKER_TRIANGLE_UP);
    }

    void drawHull(cv::InputOutputArray img, const cv::Scalar& color) {
        // Draw hull and center point
        for (int i = 0; i < hull.size(); i++)
            cv::line(img, hull.at(i), hull.at((i + 1)%hull.size()), color);

        cv::drawMarker(img, hullCenter, color, cv::MARKER_TILTED_CROSS);
    }
};

/*
Extracts contour features from a given input image.
    in - The input image.
    features - The output vector.
    minimumArea - Controls the minimum contour size
*/
void getContourFeatures(cv::InputArray in, std::vector<ContourFeatures>& features, double minimumArea=25) {
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(in, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    
    for (int i = 0; i < contours.size(); i++) {
        // Remove holes and small contours
        std::vector<cv::Point> points = contours.at(i);
        double area = cv::contourArea(points);
        if (hierarchy.at(i)[3] > -1 || points.size() < 3 || area < minimumArea) continue;

        // Store contour features
        features.push_back(ContourFeatures(points, area));
    }

    // Sort contours by area
    std::sort(features.begin(), features.end(), [](ContourFeatures a, ContourFeatures b) {
        return a.area > b.area;
    });
}

#endif