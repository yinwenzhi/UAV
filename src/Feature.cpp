//
// Created by lizechuan on 20-7-13.
//

#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include "Feature.h"
using namespace std;
using namespace cv;
#define PI 3.1415926
namespace myslam
{
//两点连线斜率
    Feature:: Feature()
    {

    }
    float kPoint(int x1, int y1, int x2, int y2) {
        if ((x2 - x1) != 0) {
            return double((y2 - y1)) / double((x2 - x1));
        } else {
            return 100000;
        }
    }

//两点间距离
    float dist2Point(int x1, int y1, int x2, int y2) {
        return std::sqrt(double(x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

//直线斜率和截距
//a,b分别是两个点
    bool cmp0(Vec4f a, Vec4f b) {
        return dist2Point(a[0], a[1], a[2], a[3]) > dist2Point(b[0], b[1], b[2], b[3]);//从大到小排>，若要从小到大排则<
    }

    struct LinePara {
        float k;
        float b;

    };

    void getLinePara(float &x1, float &y1, float &x2, float &y2, LinePara &LP) {
        double m = 0;

        // 计算分子
        m = x2 - x1;

        if (m == 0) {
            LP.k = 10000.0;
            LP.b = y1 - LP.k * x1;
        } else {
            LP.k = (y2 - y1) / (x2 - x1);
            LP.b = y1 - LP.k * x1;
        }

    }

    bool getCross(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4, Point2f &pt) {

        LinePara para1, para2;
        getLinePara(x1, y1, x2, y2, para1);
        getLinePara(x3, y3, x4, y4, para2);

        // 判断是否平行
        if (abs(para1.k - para2.k) > 0.5) {
            pt.x = (para2.b - para1.b) / (para1.k - para2.k);
            pt.y = para1.k * pt.x + para1.b;

            return true;

        } else {
            return false;
        }

    }

    void nihe(vector<Vec4f> &C, Vec4f &best) {

        vector<Vec4f> linestemp;
        Vec4f linestempp;
        vector<Point2f> point0;
        Point2f tempp;
        for (int j = 0; j < C.size(); ++j) {
            tempp.x = C[j][0];
            tempp.y = C[j][1];
            point0.push_back(tempp);
            tempp.x = C[j][2];
            tempp.y = C[j][3];
            point0.push_back(tempp);
        }
        for (int m = 0; m < point0.size() - 1; ++m) {
            for (int n = 0; n < point0.size() - 1 - m; ++n) {
                linestempp[0] = point0[m].x;
                linestempp[1] = point0[m].y;
                linestempp[2] = point0[n + 1 + m].x;
                linestempp[3] = point0[n + 1 + m].y;
                linestemp.push_back(linestempp);
            }
        }
        sort(linestemp.begin(), linestemp.end(), cmp0);
        best[0] = linestemp[0][0];
        best[1] = linestemp[0][1];
        best[2] = linestemp[0][2];
        best[3] = linestemp[0][3];
    }

    vector<Vec4f> fun(vector<Vec4f> &goodlines1, int &k) {
        if (k == 0)
            return goodlines1;
        vector<Vec4f> A;
        vector<Vec4f> B;
        Vec4f temp;
        A.push_back(goodlines1[0]);
        for (int j = 1; j < goodlines1.size(); j++) {
            float deta1 =
                    atan(kPoint(goodlines1[0][0], goodlines1[0][1], goodlines1[0][2], goodlines1[0][3])) * 180 / PI;
            float deta2 =
                    atan(kPoint(goodlines1[j][0], goodlines1[j][1], goodlines1[j][2], goodlines1[j][3])) * 180 / PI;
            float deta =
                    atan(kPoint((goodlines1[j][0] + goodlines1[j][2]) / 2, (goodlines1[j][1] + goodlines1[j][3]) / 2,
                                (goodlines1[0][0] + goodlines1[0][2]) / 2, (goodlines1[0][1] + goodlines1[0][3]) / 2)) *
                    180 / PI;
            float ddeta;
            float dddeta;
            if (abs(deta1) > 80 && abs(deta2) > 80) {
                ddeta = abs(deta1) - abs(deta2);
                dddeta = abs(deta) - abs(deta1);
            } else {
                ddeta = deta1 - deta2;
                dddeta = deta - deta2;
            }
            double d1 = std::sqrt((goodlines1[0][0] - goodlines1[j][0]) * (goodlines1[0][0] - goodlines1[j][0]) +
                                  (goodlines1[0][1] - goodlines1[j][1]) * (goodlines1[0][1] - goodlines1[j][1]));
            double d2 = std::sqrt((goodlines1[0][2] - goodlines1[j][0]) * (goodlines1[0][2] - goodlines1[j][0]) +
                                  (goodlines1[0][3] - goodlines1[j][1]) * (goodlines1[0][3] - goodlines1[j][1]));
            double d3 = std::sqrt((goodlines1[0][0] - goodlines1[j][2]) * (goodlines1[0][0] - goodlines1[j][2]) +
                                  (goodlines1[0][1] - goodlines1[j][3]) * (goodlines1[0][1] - goodlines1[j][3]));
            double d4 = std::sqrt((goodlines1[0][2] - goodlines1[j][2]) * (goodlines1[0][2] - goodlines1[j][2]) +
                                  (goodlines1[0][3] - goodlines1[j][3]) * (goodlines1[0][3] - goodlines1[j][3]));
            double detam = min(min(d1, d2), min(d3, d4));
            // abs(dddeta)<5
            if (abs(ddeta) < 3 && abs(dddeta) < 3 && detam < 15) {
                A.push_back(goodlines1[j]);
            } else {
                B.push_back(goodlines1[j]);
            }
        }
        k--;
        if (A.size() > 1) {
            nihe(A, temp);
            B.push_back(temp);
            return fun(B, k);
        } else {
            B.push_back(goodlines1[0]);
            return fun(B, k);
        }
    }

    float getdistancepl(float x0, float y0, float x1, float y1, float x2, float y2) {

        float cross = (x2 - x1) * (x0 - x1) + (y2 - y1) * (y0 - y1);
        if (cross <= 0)
            return std::sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
        float d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
        if (cross >= d2)
            return std::sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));

        float r = cross / d2;
        float px = x1 + (x2 - x1) * r;
        float py = y1 + (y2 - y1) * r;
        return std::sqrt((x0 - px) * (x0 - px) + (py - y0) * (py - y0));


    }

    void Feature:: GetCrossPoint(Mat img)
    {
        vector<Point2f> corners;
        Mat image;
        cvtColor(img, image, COLOR_BGR2GRAY);
        vector<Vec4f> lines;
        vector<Vec4f> goodlines1;
        vector<Vec4f> goodlines;
        cv::Ptr<LineSegmentDetector> detector = createLineSegmentDetector(LSD_REFINE_NONE);
        detector->detect(image, lines);
        for(int i = 0; i < lines.size(); ++i)
        {
            if (dist2Point(lines[i][0], lines[i][1], lines[i][2], lines[i][3]) > 15)
                goodlines1.push_back(lines[i]);
        }
        int x=goodlines1.size();
        goodlines=fun(goodlines1,x);
        vector<vector<Vec4f>> A;
        vector<Vec4f> temp;
        vector<double> detam;
        for(int i = 0; i <goodlines.size() ; i++)
        {
            for(int j = 0; j < goodlines.size()-1-i; j++)
            {
                temp.push_back(goodlines[i]);
                double d1=std::sqrt((goodlines[i][0]-goodlines[i+1+j][0])*(goodlines[i][0]-goodlines[i+1+j][0])+(goodlines[i][1]-goodlines[i+1+j][1])*(goodlines[i][1]-goodlines[i+1+j][1]));
                double d2=std::sqrt((goodlines[i][2]-goodlines[i+1+j][0])*(goodlines[i][2]-goodlines[i+1+j][0])+(goodlines[i][3]-goodlines[i+1+j][1])*(goodlines[i][3]-goodlines[i+1+j][1]));
                double d3=std::sqrt((goodlines[i][0]-goodlines[i+1+j][2])*(goodlines[i][0]-goodlines[i+1+j][2])+(goodlines[i][1]-goodlines[i+1+j][3])*(goodlines[i][1]-goodlines[i+1+j][3]));
                double d4=std::sqrt((goodlines[i][2]-goodlines[i+1+j][2])*(goodlines[i][2]-goodlines[i+1+j][2])+(goodlines[i][3]-goodlines[i+1+j][3])*(goodlines[i][3]-goodlines[i+1+j][3]));
                double deta=min(min(d1,d2),min(d3,d4));
                if(deta<10)
                    temp.push_back(goodlines[i+1+j]);
                A.push_back(temp);
                temp.clear();
            }
        }
        cout << "A.size() = " << A.size() << endl;
        vector<vector<Vec4f>> B;
        for(int n = 0; n < A.size(); n++)
        {
            if(A[n].size()<2)
                continue;
            float deta1=atan(kPoint(A[n][1][0],A[n][1][1],A[n][1][2],A[n][1][3]))*180/PI;
            float deta2=atan(kPoint(A[n][0][0],A[n][0][1],A[n][0][2],A[n][0][3]))*180/PI;
            float ddeta;
            if(abs(deta1)>80 && abs(deta2)>80)
            {
                ddeta=abs(deta1)-abs(deta2);
            }
            else
            {
                ddeta=abs(deta1-deta2);
            }
            if(ddeta>20)
                B.push_back(A[n]);
        }

        Point2f pt;
        for(int i = 0; i < B.size(); i++)
        {
            bool as;
            for(int j = 0; j < B[i].size(); j++)
                as=getCross(B[i][0][0], B[i][0][1],B[i][0][2], B[i][0][3],B[i][j][0], B[i][j][1],B[i][j][2], B[i][j][3], pt);
            if(as==true)
                corners.push_back(pt);
        }
        _corners=corners;
        cout<<"corners.size():"<<corners.size()<<endl;
            //cout<<crosspoint[i].x<<" "<<crosspoint[i].y<<endl;
        }

}
