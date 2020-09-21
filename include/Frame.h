//
// Created by lizechuan on 20-7-13.
//

#ifndef FRAME_H
#define FRAME_H
// forward declare
#include "common_include.h"
#include "camera.h"
namespace myslam
{
class MapPoint;
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long                  id_;         // id of this frame
    double                         time_stamp_; // when it is recorded
    SE3                            T_c_w_;      // transform from world to camera
    Camera::Ptr                    camera_;     // Pinhole RGBD Camera model
    Mat                            color_; // color and depth image
    vector< Point3f >              pts_3d_;
    map<int,Point3f>               pts_3d_ref_;
    // std::vector<cv::KeyPoint>      keypoints_;  // key points in image
    // std::vector<MapPoint*>         map_points_; // associated map points
    //bool                           is_key_frame_;  // whether a key-frame

public: // data members
    Frame();
    Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat() );
    ~Frame();

    static Frame::Ptr createFrame();


    // Get Camera Center
    Vector3d getCamCenter() const;

    void setPose( const SE3& T_c_w );

};

}
#endif //FRAME_H
