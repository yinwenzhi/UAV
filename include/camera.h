//
// Created by lizechuan on 20-7-13.
//

#ifndef CAMERA_H
#define CAMERA_H

#include "common_include.h"
#include <opencv2/opencv.hpp>
#include "Config.h"
namespace myslam
{

    class Camera
    {
    public:
        typedef std::shared_ptr<Camera> Ptr;
        float   fx_, fy_, cx_, cy_;
        static float invfx;
        static float invfy;
        cv::Mat mDistCoef;


        // Number of KeyPoints.
        int N; ///< KeyPoints数量
        Mat K;
    public:
        Camera();
        Camera( float fx, float fy, float cx, float cy) :
                fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy )
        {}

        // coordinate transform: world, camera, pixel
        Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
        Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w );
        Vector2d camera2pixel( const Vector3d& p_c );
        Vector3d pixel2camera( const Vector2d& p_p, double depth=1 );
        Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth=1 );
        Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w );
    };

}
#endif //CAMERA_H
