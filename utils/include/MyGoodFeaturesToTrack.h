#ifndef MY_GOOD_FEATURE_TO_TRACK_H
#define MY_GOOD_FEATURE_TO_TRACK_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#define CUSTOM_OVERRIDE

template<typename T> struct greaterThanPtr
{
    bool operator()(const T* a, const T* b) const { return *a > *b; }
};

void my_goodFeaturesToTrack(cv::InputArray _image, cv::OutputArray _corners,
                              int maxCorners, double qualityLevel, double minDistance,
                              cv::InputArray _mask = cv::noArray(), int blockSize = 3);

float minEig(int16_t a, int16_t b, int16_t c);

#endif // MY_GOOD_FEATURE_TO_TRACK_H