#ifndef HSV_COLOR_RANGE_HPP
#define HSV_COLOR_RANGE_HPP

/*
Defines HSVColorRange struct - a configurable object for ranging hsv opencv arrays
*/

#include "opencv2/core.hpp"

struct HSVColorRange {
    int hue;
    int range;
    int saturation;
    int value;
    cv::Scalar low;
    cv::Scalar high;

    HSVColorRange(int hue, int range, int saturation, int value) :
    hue(hue), range(range), saturation(saturation), value(value)
    { update(); }

    void update() {
        int lowHue = hue - range;
        int highHue = hue + range - 1;
        low = cv::Scalar(lowHue + (lowHue < 0 ? 180 : 0), saturation, value);
        high = cv::Scalar(highHue - (highHue >= 180 ? 180 : 0), 255, 255);
    }

    void inRange(cv::InputArray in, cv::OutputArray out) {
        if (low[0] <= high[0] || range == 0) cv::inRange(in, low, high, out);
        else {
            cv::Mat out1;
            cv::Mat out2;
            cv::inRange(in, cv::Scalar(0, saturation, value), high, out1);
            cv::inRange(in, low, cv::Scalar(180, 255, 255), out2);
            out.assign(out1 + out2);
        }
    }

    void setHue(int newHue) {
        hue = newHue;
        update();
    }

    void setRange(int newRange) {
        range = newRange;
        update();
    }

    void setSaturation(int newSaturation) {
        saturation = newSaturation;
        update();
    }

    void setValue(int newValue) {
        value = newValue;
        update();
    }
};

#endif