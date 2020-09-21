//
// Created by lizechuan on 20-7-13.
//
#include "common_include.h"
#include "Frame.h"
#include "Feature.h"
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#ifndef POINTMATCH_H
#define POINTMATCH_H
namespace myslam
{
    typedef struct Point{
        KeyPoint p1;
        Mat p2;
        Point3f p3;
    }pointx;

    class PointMatch
            {
    public:
        typedef shared_ptr<PointMatch> Ptr;
        enum VOState {
            INITIALIZING=-1,
            OK=0,
            LOST
        };
        VOState     state_;
        cv::Ptr<cv::ORB> orb_;  // orb detector and computer
       // vector<cv::Point3f>     pts_3d_ref_;        // 3d points in reference frame
        vector<cv::KeyPoint>    keypoints_curr_;    // keypoints in current frame
        vector<cv::KeyPoint>    keypoints_ref_;    // keypoints in ref frame
        Mat                     descriptors_curr_;  // descriptor in current frame
        Mat                     descriptors_ref_;   // descriptor in reference frame
        vector<cv::DMatch>      feature_matches_;   // feature matches
        cv::FlannBasedMatcher   matcher_flann_;     // flann matcher
        Frame::Ptr  ref_;       // reference key-frame
        Frame::Ptr  curr_;      // current frame
        vector< Point3f >              pts_3d_;
        map<int,Point3f>               pts_3d_ref_map_;
        Camera::Ptr                    camera_;
        Eigen::Isometry3d             T_esti;
        Vec3f                          angle_;
        Vector3d                          translation_;
        
    public:
        bool addFrame( Frame::Ptr frame );      // add a new frame
        void computeDescriptors();
        void featureMatching();
        bool pose_estimation_3d2d ( std::vector< DMatch > matches,Mat& R, Mat& t );
        void bundleAdjustment (
                const vector< Point3f > points_3d,
                const vector< Point2f > points_2d,
                const Mat& K,
                Mat& R, Mat& t );

    };
}
#endif //POINTMATCH_H
