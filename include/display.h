#ifndef DISPLAY_H
#define DISPLAY_H

#include "common_include.h"
#include <opencv2/imgproc.hpp>
// #include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

void display(cv::Mat img, Eigen::Vector3d trans,int k, long dt);

#endif