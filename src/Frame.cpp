//
// Created by lizechuan on 20-7-13.
//

#include "Frame.h"

namespace myslam
{
    Frame::Frame()
            : id_(-1), time_stamp_(-1), camera_(nullptr)
    {

    }

    Frame::Frame ( long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color )
            : id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color)
    {

    }

    Frame::~Frame()
    {

    }

    Frame::Ptr Frame::createFrame()
    {
        static long factory_id = 0;
        return Frame::Ptr( new Frame(factory_id++) );
    }



    void Frame::setPose ( const SE3& T_c_w )
    {
        T_c_w_ = T_c_w;
    }


    Vector3d Frame::getCamCenter() const
    {
        return T_c_w_.inverse().translation();
    }
}