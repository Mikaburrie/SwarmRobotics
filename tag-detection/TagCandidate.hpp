#ifndef TAG_CANDIDATE_HPP
#define TAG_CANDIDATE_HPP

#include "ContourFeatures.hpp"

#define TAG_CORNER_CENTER_RATIO (1/(8 + 8*sqrt(2)))

struct TagCandidate;
struct CornerCandidate;
void getTagCandidates(cv::Mat& displayFrame, const std::vector<ContourFeatures>&, const std::vector<ContourFeatures>&, std::vector<TagCandidate>&, std::vector<TagCandidate>&, double, double, double, double);
void getCornerCandidates(const ContourFeatures&, const std::vector<ContourFeatures>&, std::vector<CornerCandidate>&, double, double, double);
std::pair<int, double> classifyAngleDirection(double, double);
double wrapToPi(double angle);

struct TagCandidate {
    int vec;
    int ellipseIndex;
    double score = 0;
    std::vector<CornerCandidate> corners;
};

struct CornerCandidate {
    int triangleIndex = -1;
    int farIndex = -1;
    double farAngle;
    double meanAngleOffset = 0;
    double pointDistance[3];
    double edgeDistance[3];

    CornerCandidate(std::vector<cv::Point> triangle, const cv::RotatedRect& ellipse, cv::Point2f& ellipseAngle) {
        assert((triangle.size() == 3) && "CornerCandidate triangle.size() != 3");

        // Transform points to ellipse basis and find farthest (largest error)
        double pointDistanceMax = -1;
        cv::Point2f points[3];
        double angles[3];
        for (int i = 0; i < 3; i++) {
            // Calculate vector from ellipse to triangle and find ratio to ellipse radius
            cv::Point2f diff = ((cv::Point2f) triangle.at(i)) - ellipse.center;

            // Find distance error from ellipse edge (point)
            points[i].x = ellipseAngle.dot(diff);
            points[i].y = ellipseAngle.cross(diff);
            angles[i] = atan2(points[i].y*ellipse.size.width, points[i].x*ellipse.size.height);
            cv::Point2f edge(cos(angles[i])*ellipse.size.width, sin(angles[i])*ellipse.size.height);
            pointDistance[i] = 2*sqrt(diff.dot(diff)/edge.dot(edge)) - 1;

            // Determine farthest
            if (pointDistance[i] > pointDistanceMax) {
                pointDistanceMax = pointDistance[i];
                farIndex = i;
            }
        }

        // Determine edge errors and angles
        for (int i = 0; i < 3; i++) {
            // Find distance ratio from ellipse edge (line)
            cv::Point2f dir = points[(i + 1)%3] - points[i];
            double theta = atan2(-ellipse.size.height*dir.x, ellipse.size.width*dir.y);
            cv::Point2f edge(ellipse.size.width/2*cos(theta), ellipse.size.height/2*sin(theta));
            edgeDistance[i] = abs(dir.cross(points[i])/dir.cross(edge)) - 1;

            // Add to angle error if not center point
            if (i == farIndex) farAngle = angles[farIndex];
            else meanAngleOffset += wrapToPi(angles[i] - angles[farIndex]);
        }
    }
};

// Finds tags from contours
void getTagCandidates(cv::Mat& displayFrame, const std::vector<ContourFeatures>& cf0, const std::vector<ContourFeatures>& cf1, std::vector<TagCandidate>& tags0, std::vector<TagCandidate>& tags1, double ellipseThreshold=0.85, double cornerAreaScaleLimit=2.5, double cornerDistanceErrorLimit=0.3, double cornerAngleErrorLimit=(CV_PI/9)) {
    for (int i = 0; i < cf0.size(); i++) {
        // Discard if not ellipsoid enough
        const ContourFeatures& ellipse = cf0.at(i);
        if (ellipse.ellipseness < ellipseThreshold) continue;

        // Create candidate
        TagCandidate tag;
        tag.vec = 0;
        tag.ellipseIndex = i;

        // Get corner candidates for ellipse
        getCornerCandidates(ellipse, cf1, tag.corners, cornerAreaScaleLimit, cornerDistanceErrorLimit, cornerAngleErrorLimit);

        // Store tag
        tags0.push_back(std::move(tag));
    }

    // Same thing but vectors are swapped
    for (int i = 0; i < cf1.size(); i++) {
        const ContourFeatures& ellipse = cf1.at(i);
        if (ellipse.ellipseness < ellipseThreshold) continue;

        TagCandidate tag;
        tag.vec = 0;
        tag.ellipseIndex = i;

        getCornerCandidates(ellipse, cf0, tag.corners, cornerAreaScaleLimit, cornerDistanceErrorLimit, cornerAngleErrorLimit);
        tags1.push_back(std::move(tag));
    }
}

void getCornerCandidates(const ContourFeatures& ellipse, const std::vector<ContourFeatures>& triangles, std::vector<CornerCandidate>& corners, double cornerAreaScaleLimit=2.5, double cornerDistanceErrorLimit=0.3, double cornerAngleErrorLimit=(CV_PI/9)) {
    // Calculate corner area size limit
    double cornerAreaLimit = ellipse.area*cornerAreaScaleLimit*TAG_CORNER_CENTER_RATIO;

    // Determine basis vectors of ellipse
    double ellipseRadians = ellipse.ellipseRect.angle*CV_PI/180;
    cv::Point2f ellipseAngle(cos(ellipseRadians), sin(ellipseRadians));

    // Vector for storing candidates
    std::vector<CornerCandidate> candidates;

    // Vector storing the best corner candidates
    double bestCandidatesError = 4;
    int bestCandidates[4] {-1, -1, -1, -1};

    // Find corner candidates
    for (int triangleIndex = 0; triangleIndex < triangles.size(); triangleIndex++) {
        const ContourFeatures& triangle = triangles.at(triangleIndex);

        // Discard if too big
        if (triangle.area > cornerAreaLimit || triangle.minTriangle.size() != 3) continue;

        // Create candidate to calculate suitability of corner
        CornerCandidate corner(triangle.minTriangle, ellipse.ellipseRect, ellipseAngle);
        corner.triangleIndex = triangleIndex;

        // Discard if too far away from ellipse
        double radiusError = 0;
        for (int i = 0; i < 3; i++) {
            if (i != corner.farIndex) radiusError += corner.pointDistance[i];
        }
        if (radiusError/2 > cornerDistanceErrorLimit) continue;

        // TODO: Improve candidate filtering

        // Store candidate
        int tri1 = (int) candidates.size();
        candidates.push_back(corner);

        // Arrays for storing candidate combinations and angles
        int currentCandidates[4] {tri1, -1, -1, -1};
        double angles[tri1];

        // Determine error and store candidates if new best
        double error1 = 3;
        if (error1 < bestCandidatesError) {
            bestCandidates[0] = currentCandidates[0];
            bestCandidatesError = error1;
        }
        
        // Find second corner from other candidates
        for (int tri2 = 0; tri2 < tri1; tri2++) {
            // Classify angle to tri2. Continue if none, store index if valid
            angles[tri2] = wrapToPi(candidates.at(tri2).farAngle - corner.farAngle);
            std::pair<int, double> tri2dir = classifyAngleDirection(angles[tri2], cornerAngleErrorLimit);
            if (tri2dir.first == -1) continue;
            currentCandidates[tri2dir.first + 1] = tri2;

            // Determine error and store candidates if new best
            double error2 = 2 + tri2dir.second;
            if (error2 < bestCandidatesError) {
                std::copy(std::begin(currentCandidates), std::end(currentCandidates), std::begin(bestCandidates));
                bestCandidatesError = error2;
            }

            // Find third corner from remaining candidates
            for (int tri3 = tri2 - 1; tri3 >= 0; tri3--) {
                // Classify angle. Continue if none or taken, store index if valid
                std::pair<int, double> tri3dir = classifyAngleDirection(angles[tri3], cornerAngleErrorLimit);
                if (tri3dir.first == -1 || tri3dir.first == tri2dir.first) continue;
                currentCandidates[tri3dir.first + 1] = tri3;

                // Determine error and store candidates if new best
                double error3 = 1 + tri2dir.second + tri3dir.second;
                if (error3 < bestCandidatesError) {
                    std::copy(std::begin(currentCandidates), std::end(currentCandidates), std::begin(bestCandidates));
                    bestCandidatesError = error3;
                }

                // Find fourth corner from remaining candidates
                for (int tri4 = tri3 - 1; tri4 >= 0; tri4--) {
                    // Classify angle. Continue if none or taken, store index if valid
                    std::pair<int, double> tri4dir = classifyAngleDirection(angles[tri4], cornerAngleErrorLimit);
                    if (tri4dir.first != 3 - tri3dir.first - tri2dir.first) continue;
                    currentCandidates[tri4dir.first + 1] = tri4;

                    // Determine error and store candidates if new best
                    double error4 = tri2dir.second + tri3dir.second + tri4dir.second;
                    if (error4 < bestCandidatesError) {
                        std::copy(std::begin(currentCandidates), std::end(currentCandidates), std::begin(bestCandidates));
                        bestCandidatesError = error4;
                    }

                    // Unmark candidate
                    currentCandidates[tri4dir.first + 1] = -1;
                }

                // Unmark candidate
                currentCandidates[tri3dir.first + 1] = -1;
            }

            // Unmark candidate
            currentCandidates[tri2dir.first + 1] = -1;
        }
    }

    // Output best candidates
    for (int i = 0; i < 4; i++) {
        if (bestCandidates[i] == -1) continue;
        corners.push_back(std::move(candidates.at(bestCandidates[i])));
    }
}

// Classifies angles into one of three directions
// Returns <int direction, double error>
// -1 -> no direction
// 0 -> +90
// 1 -> +-180
// 2 -> -90
std::pair<int, double> classifyAngleDirection(double angle, double threshold) {
    // Calculate error from ideal angles
    double errors[3] {
        abs(angle - CV_PI/2),
        CV_PI - abs(angle),
        abs(angle + CV_PI/2)
    };

    // Determine classification and return -1 if none found
    int classification;
    for (classification = 2; classification >= 0; classification--)
        if (errors[classification] < threshold) break;
    
    return std::pair<int, double>(classification, (classification == -1 ? -1 : errors[classification]));
}

double wrapToPi(double angle) {
    return angle - 2*CV_PI*round(angle/CV_PI/2);
}

#endif