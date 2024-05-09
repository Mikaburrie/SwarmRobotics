
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

struct Robot {

    int tagCount = 0;
    bool sides[4] {false, false, false, false};
    cv::Point3d center {0, 0, 0};
    cv::Point3d direction {0, 0, 0};

    double distance; //  d
    double angle; // theta
    double heading; // phi

    Robot(OctoTag& tag, float tagSize) {
        addTag(tag, tagSize, true);
    }

    bool addTag(OctoTag& tag, float tagSize, bool bypassChecks = false) {
        // Ensure yellow and purple tags are not passed
        assert(tag.color != 2 && tag.color != 5);

        // Determine robot center and forward direction from tag color
        int index = tag.color - (tag.color > 2);
        cv::Point3d tagCenter = cv::Point3d(tag.tvec.at(0), tag.tvec.at(1), tag.tvec.at(2)) - tag.normal*tagSize/2;
        cv::Point3d tagDirection;
        cv::Point3d up(0, 1, 0);
        switch (index) {
            case 0: tagDirection = tag.normal;           break;
            case 1: tagDirection = tag.normal.cross(up); break;
            case 2: tagDirection = -tag.normal;          break;
            case 3: tagDirection = up.cross(tag.normal); break;
        }

        if (!bypassChecks) {
            // Return if side color is already detected
            int index = tag.color - (tag.color > 2);
            if (sides[index]) return false;

            // Return if center is not close enough
            cv::Point3d diff = tagCenter - center;
            if (diff.dot(diff) > tagSize*tagSize/4) return false;

            // Return if normal directon is not within 60 degrees of robot direction
            if (direction.dot(tagDirection) < 0.5) return false;
        }


        // Update center and direction
        center = (center*tagCount + tagCenter)/(tagCount + 1);
        direction = (direction*tagCount + tagDirection)/(tagCount + 1);

        // Update distance, angle, and heading
        distance = cv::norm(center);
        angle = atan2(center.x, center.z);
        heading = atan2(direction.x, direction.z);

        // Mark side index and increment tag count
        sides[index] = true;
        tagCount++;
        return true;
    }

};

struct Wall {

    int tagCount = 0;
    int color;
    cv::Point3d normal {0, 0, 0};
    cv::Point3d center {0, 0, 0};

    double distance;
    double angle;

    Wall(OctoTag& tag) {
        color = tag.color;
        addTag(tag, true);
    }

    bool addTag(OctoTag& tag, bool bypassChecks = false) {
        if (!bypassChecks) {
            // Return if colors mismatch
            if (color != tag.color) return false;

            // Return if normals differ by more than 60 degrees
            if (normal.dot(tag.normal) < 0.5) return false;

            // Return if the new tag's center is in front or behind the wall plane
            cv::Point3d tagCenter(tag.tvec.at(0), tag.tvec.at(1), tag.tvec.at(2));
            if (abs(normal.dot(tagCenter - center)) > 10) return false;
        }

        // Update center and normal
        cv::Point3d tagCenter(tag.tvec.at(0), tag.tvec.at(1), tag.tvec.at(2));
        center = (center*tagCount + tagCenter)/(tagCount + 1);
        normal = (normal*tagCount + tag.normal)/(tagCount + 1);

        // Update angle and distance
        cv::Point3d forward(0, 0, 1);

        // Increment tag count and return
        tagCount++;
        return true;
    }
};

void drive() {
    
    //sendMotorCommand(80, -80);
    //sleep(1);
    sendMotorCommand(35, 65);
}

// Slow down this robot based on the number of robots
// within the minimum distance
void seperation(std::vector<Robot> robots, int minDistR, int maxDist, int &LMS, int &RMS) {
    int close = 0;
    int sep = 0;
    for (Robot robot: robots) {
        int robotDist = robot.distance;
        if (robotDist < minDistR) {
            LMS *= -1; //reverse direction
            RMS *= -1;
            break;
        } else if (robotDist > maxDist) {
            LMS += 5; //speed up a lil :3 HEHEHEHA GRRRRRR
            RMS += 5;
        }
    }
    close = (close / (minDistR*3)) * sep; //do i need this?
    
}

// Turn towards average angle of observed robots
void cohesion(std::vector<Robot> robots, int &LMS, int &RMS) {
    double angle = 0;
    int turn = 3;
    for (Robot robot: robots) {
        angle += robot.angle;
    }
    if (angle < -0.05) {
        LMS -= turn;
        RMS += turn;
    } else if (angle > 0.05) {
        LMS += turn; 
        RMS -= turn;
    }
}

// Avoid da walls
void avoidWalls(std::vector<Wall> walls, int minDistW, int &LMS, int &RMS) {
    double angle = 0;
    int turn = 3;
    for (Wall wall: walls) {
        int wallDist = wall.distance;
        if (wallDist < minDistW) {
            angle += wall.angle;
        }
    }
    if (angle < -0.05) {
        LMS += turn;
        RMS -= turn;
    }  else if (angle > 0.05) {
        LMS -= turn; 
        RMS += turn;
    }
}

// Delete if only used once
// Used in cohesion function, turns towards average angle of robots
void turnTowards(double angle, int &LMS, int &RMS) {
    if (angle < 0.0) {
        LMS += 2;
        RMS -= 2;
    } else {
        LMS -= 2; 
        RMS += 2;
    }
}

// Delete if only used once
// Used in avoidWalls function to turn away from walls
void turnAway(double angle, int &LMS, int &RMS) {
    if (angle < 0.0) {
        LMS -= 2;
        RMS += 2;
    } else {
        LMS += 2; 
        RMS -= 2;
    }
}

// Check if any wall tags are less than the minimum allowed distance
// returns true if too close
bool wallTooClose(std::vector<Wall> walls, int minDistW) {
    for (Wall wall: walls) {
        int wallDist = wall.distance;
        if (wallDist < minDistW) {
            return true;
        }
    }
    return false;
}

void boids(std::vector<Wall> walls, std::vector<Robot> robots, int minDistR, int minDistW) {
    int LMS = 60;
    int RMS = 60;
    
    if (wallTooClose(walls, minDistW)) {
        avoidWalls(walls, 30, LMS, RMS);
    } else {
        seperation(robots, 30, 80, LMS, RMS);
        cohesion(robots, LMS, RMS);
    }
    
    sendMotorCommand(LMS, RMS);
    std::cout << "LMS = " << LMS << " | RMS =  " << RMS << std::endl;
}

void detectRobotsAndWalls(cv::Mat& frame, const OctoTagConfiguration& config, std::vector<Robot>& robots, std::vector<Wall>& walls) {
    // Get tags in frame
    std::vector<OctoTag> tags;
    detectOctoTags(frame, config, tags);
    
    // Process tag detections
    for (OctoTag tag: tags) {
        tag.drawPose(frame, config);

        if (tag.color == 2 || tag.color == 5) {
            // Attempt to add tag to existing walls
            bool addedToWall = false;
            for (Wall& wall: walls) {
                addedToWall = wall.addTag(tag);
                if (addedToWall) break;
            }
            
            // Create new wall if no existing walls contain tag
            if (!addedToWall) walls.push_back(Wall(tag));
        } else {
            // Attempt to add tag to existing robot
            bool addedToRobot = false;
            for (Robot& robot: robots) {
                addedToRobot = robot.addTag(tag, config.tagSize);
                if (addedToRobot) break;
            }
            
            // Create new robot if no existing robots contain tag
            if (!addedToRobot) robots.push_back(Robot(tag, config.tagSize));
        }
    }
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
        // std::vector<OctoTag> tags;
        // detectOctoTags(frame, config, tags);
        
        std::vector<Robot> robots;
        std::vector<Wall> walls;
        detectRobotsAndWalls(frame, config, robots, walls);

        std::cout << "Robots: " << std::endl;
        for (Robot robot: robots) {
            std::cout << robot.tagCount << " " << robot.distance << " " << robot.angle << " " << robot.heading << std::endl;
        }
        std::cout << std::endl;

        std::cout << "Walls: " << std::endl;
        for (Wall wall: walls) {
            std::cout << wall.tagCount << " " << wall.distance << " " << wall.angle << std::endl;
        }
        
        if (robots.size() == 0 && !wallTooClose(walls, 30)) {
            drive();
            std::cout << " ggggaewrggwgqw" << std::endl;
        } else {
            boids(walls, robots, 30,30);
        }

        //std::cout << std::endl;

        cv::imshow("camera", frame);

        key = cv::waitKey(16);
    }
    
    // Close FIFO pipe
    closeMotorControlFifo();
}
