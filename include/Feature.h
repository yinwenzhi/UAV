//
// Created by lizechuan on 20-7-13.
//

#ifndef FEATURE_H
#define FEATURE_H

#include "common_include.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#define PI 3.1415926
namespace myslam
{
    class Feature
            {
    public:
        typedef std::shared_ptr<Feature> Ptr;
        //Point and lines of an image
        vector<Vec4f> _lines;
        vector<Point2f> _corners;

    public:
        Feature();

        void GetCrossPoint(Mat img);

        // KeyPoints数量


        int GetNp() {
            return _corners.size();
        }

        // line数量
        int GetNl() {
            return _lines.size();
        }
    };
}
#endif //FEATURE_H
