//
// Created by lizechuan on 20-7-13.
//
#include "PointMatch.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>
#include <boost/timer.hpp>
using namespace std;
using namespace cv;

#define preparation 0  // 是否运行筛选程序 1->运行；0->不运行

namespace myslam
{
    //将旋转矩阵转换为欧拉角
    Vec3f rotationMatrixToEulerAngles(Mat &R)
    {

        float Cy = sqrt(R.at<double>(2,1) * R.at<double>(2,1) +  R.at<double>(2,2) * R.at<double>(2,2) );


        bool singular = Cy < 1e-6; // If


        float x, y, z;
        if (!singular) {
            x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
            y = atan2(-R.at<double>(2,0), Cy);
            z = atan2(R.at<double>(1,0), R.at<double>(0,0));
        } else {
            x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            y = atan2(-R.at<double>(2,0), Cy);
            z = 0;
        }
        // x = x*(180/3.1415926);
        // y = y*(180/3.1415926);
        // z = z*(180/3.1415926);
        return Vec3f(z, y, x);
    }

    bool PointMatch::addFrame ( Frame::Ptr frame )//计算了每一帧的位姿
    {
        
        curr_ = frame;
        Feature f;
        f.GetCrossPoint(curr_->color_);//提取交点
        KeyPoint::convert(f._corners, keypoints_curr_);//类型转换
        cout<<"keypoints_curr_.size()"<<keypoints_curr_.size()<<endl;
        if(keypoints_curr_.size()<10)
        {
            cout<<"keypoints_curr_.size()"<< keypoints_curr_.size() << "小于10, continue "<<endl;
            return false;
        }
        boost::timer timer;

        // 当前帧提取描述子
        vector<DMatch> match;
        // cv::Ptr<DescriptorExtractor> descriptor = ORB::create();
        // descriptor->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
        
        cv:: Ptr<Feature2D> descriptor = BRISK::create();
        descriptor->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );

        cv::Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
        matcher->match ( descriptors_ref_, descriptors_curr_, match );
        cout<<"match.size()= "<<match.size()<<endl;

        if (match.size() < 5)
        {
            cout<<"match.size()= "<<match.size()<<endl;
            return false;
        }



        double min_dist=10000, max_dist=0;
        //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
        for ( int i = 0; i < descriptors_ref_.rows; i++ )
            // for ( int i = 0; i < descriptors_1.rows; i++ )
        {
            double dist = match[i].distance;
            if ( dist < min_dist ) min_dist = dist;
            if ( dist > max_dist ) max_dist = dist;
        }

        //printf ( "-- Max dist : %f \n", max_dist );
        //printf ( "-- Min dist : %f \n", min_dist );

        feature_matches_.clear();

        #ifdef preparation
        //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
        for ( int i = 0; i < descriptors_ref_.rows; i++ )
        {
            if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
            {
                feature_matches_.push_back ( match[i] );
            }
        }
        #else
        for ( cv::DMatch& m : match )
        {
            if ( m.distance < max<float> ( min_dist*2, 30.0 ) 
            && ( abs(keypoints_ref_[m.queryIdx].pt.x - keypoints_curr_[m.trainIdx].pt.x ) < 400 ) 
            && ( abs(keypoints_ref_[m.queryIdx].pt.y - keypoints_curr_[m.trainIdx].pt.y ) < 200 ))
            {
                feature_matches_.push_back(m);
            }
        }
        #endif
        
        cout<<"feature_matches_.size()= "<<feature_matches_.size()<<endl;
        // featureMatching();

        //CV_RANSAC
        vector<Point2f> srcPoints(feature_matches_.size()),dstPoints(feature_matches_.size());
        //保存从关键点中提取到的匹配点对的坐标
        if (feature_matches_.size() == 0)
        {

        } else {
            for(int i=0;i<feature_matches_.size();i++)
            {
                srcPoints[i]=keypoints_ref_[feature_matches_[i].queryIdx].pt;
                dstPoints[i]=keypoints_curr_[feature_matches_[i].trainIdx].pt;
            }
            //保存计算的单应性矩阵
            Mat homography;
            //保存点对是否保留的标志
            vector<unsigned char> inliersMask1(srcPoints.size());
            //匹配点对进行RANSAC过滤
            homography = findHomography(srcPoints,dstPoints,CV_RANSAC,5,inliersMask1);
            //RANSAC过滤后的点对匹配信息
            vector<DMatch> matches_ransac;
            //手动的保留RANSAC过滤后的匹配点对
            for(int i=0;i<inliersMask1.size();i++)
            {
                if(inliersMask1[i])
                {
                    matches_ransac.push_back(feature_matches_[i]);
                    //cout<<"第"<<i<<"对匹配："<<endl;
                    //cout<<"queryIdx:"<<matches[i].queryIdx<<"\ttrainIdx:"<<matches[i].trainIdx<<endl;
                    //cout<<"imgIdx:"<<matches[i].imgIdx<<"\tdistance:"<<matches[i].distance<<endl;
                }
            }
            feature_matches_ = matches_ransac;
        }

        //计算当前帧位姿
        Mat R,t;
        if( !pose_estimation_3d2d(feature_matches_,R, t ) ){
            return false;
        }

        //筛选匹配到的当前帧的特征点和描述子，并将当前帧设为参考帧
        Mat desp_tem;
        // vector<cv::KeyPoint> kp_temp;
        vector< Point3f >  temp_3d=pts_3d_;
        // pts_3d_.clear();
        // for ( size_t i=0; i<keypoints_curr_.size(); i++ )
        // {
        //     for (size_t m = 0; m < feature_matches_.size(); m++)
        //     {
        //         if(i==feature_matches_[m].trainIdx)
        //         {
        //             cv::Point3f pp1;
        //             pp1=temp_3d[feature_matches_[m].queryIdx];
        //             // cout<<"pp1.p3= "<<pp1.p3<<endl;
        //             // pts_3d_.push_back(pp1);
        //             // desp_tem.push_back(descriptors_curr_.row(i).clone());
        //             kp_temp.push_back(keypoints_curr_[i]);
        //         }
        //         else
        //         {
        //             continue;
        //         }
        //     }
        // }
        
        descriptors_curr_.release();
        //keypoints_curr_.clear();
        //descriptors_ref_=desp_tem;
        // ref_ = curr_;
        // desp_tem.release();

        cout<<"match cost time: "<<timer.elapsed()<<endl;
        return true;
    }

    // void PointMatch::computeDescriptors()
    // {
    //     cv::Ptr<DescriptorExtractor> descriptor = ORB::create();
    //     cv:: Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //     descriptor->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
    //     cout<<123<<endl;
    //     boost::timer timer;
    //     cout<<123<<endl;
    //     orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
    //     cout<<"descriptor computation cost time: "<<timer.elapsed()<<endl;
    // }

    void PointMatch::featureMatching()
    {
        boost::timer timer;
        vector<cv::DMatch> matches;
        matcher_flann_.match( descriptors_ref_, descriptors_curr_, matches );
        // select the best matches
        cout<<123<<endl;
        float min_dis = std::min_element (
                matches.begin(), matches.end(),
                [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
                {
                    return m1.distance < m2.distance;
                } )->distance;

        feature_matches_.clear();
        for ( cv::DMatch& m : matches )
        {
            if ( m.distance < max<float> ( min_dis*2, 30.0 ) )
            {
                feature_matches_.push_back(m);
            }
        }
        cout<<"good matches: "<<feature_matches_.size()<<endl;
        cout<<"match cost time: "<<timer.elapsed()<<endl;
    }

    bool PointMatch:: pose_estimation_3d2d (std::vector< DMatch > matches,Mat& R, Mat& t )
    {
        cout<<"ref_->pts_3d_ref_.size:"<<pts_3d_.size()<<endl;
        //Mat K = f_cur.camera_->K;
        Mat K = ( Mat_<double> ( 3,3 ) << camera_->fx_, 0, camera_->cx_, 0, camera_->fy_, camera_->fy_, 0, 0, 1 );
        vector<Point3f> pts_3d;
        vector<Point2f> pts_2d;
        for ( DMatch m:matches )
        {
            Point3f p1 = pts_3d_[m.queryIdx];
            pts_3d.push_back (p1);
            pts_2d.push_back ( keypoints_curr_[m.trainIdx].pt );
           // cout<<"m.trainIdx= "<<m.trainIdx<<endl;
            //cout<<"m.queryIdx= "<<m.queryIdx<<endl;
            //cout<<"3d= "<<p1<<endl;
            //cout<<"2d= "<<keypoints_curr_[m.trainIdx].pt<<endl;
        }

        cout<<"3d-2d pairs: "<<pts_3d.size() <<endl;
        if(pts_3d.size()<4){
            cout << "3d-2d pairs < 4,return " << endl;  
            return false;
        }
        // solvePnP ( pts_3d, pts_2d, K, Mat(), R, t, false, SOLVEPNP_EPNP ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
        solvePnP ( pts_3d, pts_2d, K, Mat(), R, t, false, CV_ITERATIVE ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
        cv::Rodrigues ( R, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵

        // cout<<"R="<<endl<<R<<endl;
        angle_= rotationMatrixToEulerAngles(R);
        cout << "before BA R: " << std::endl << R << std::endl;
        cout << "EulerAngles_: "<<angle_*52.7<<endl;
        cout<<"before BA t="<<endl<<t<<endl;

        bundleAdjustment ( pts_3d, pts_2d, K, R, t );

        angle_= rotationMatrixToEulerAngles(R);
        cout<<endl<<"after optimization:"<<endl;
        cout << "after BA R: " << std::endl << R << std::endl;
        cout << "EulerAngles_: "<<angle_*52.7<<endl;
        cout<<"after BA t="<<endl<<t<<endl;

        // return 
        return true;

    }
    void  PointMatch:: bundleAdjustment (
            const vector< Point3f > points_3d,
            const vector< Point2f > points_2d,
            const Mat& K,
            Mat& R, Mat& t )
    {
        // 初始化g2o
        typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
        
        // Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
        // g2o接口更改
        std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverCSparse<Block::PoseMatrixType>());

        // Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
        // g2o接口更改
        std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器

        // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
        // g2o接口更改
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm ( solver );

        // vertex
        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
        Eigen::Matrix3d R_mat;
        R_mat <<
              R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
                R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
                R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
        pose->setId ( 0 );
        pose->setEstimate ( g2o::SE3Quat (
                R_mat,
                Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
        ) );
        optimizer.addVertex ( pose );

        int index = 1;
        for ( const Point3f p:points_3d )   // landmarks
        {
            g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
            point->setId ( index++ );
            point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
            point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
            optimizer.addVertex ( point );
        }

        // parameter: camera intrinsics
        g2o::CameraParameters* camera = new g2o::CameraParameters (
                K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
        );
        camera->setId ( 0 );
        optimizer.addParameter ( camera );

        // edges
        index = 1;
        for ( const Point2f p:points_2d )
        {
            g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
            edge->setId ( index );
            edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
            edge->setVertex ( 1, pose );
            edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
            edge->setParameterId ( 0,0 );
            edge->setInformation ( Eigen::Matrix2d::Identity() );
            optimizer.addEdge ( edge );
            index++;
        }

        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        optimizer.setVerbose ( false );
        optimizer.initializeOptimization();
        optimizer.optimize ( 100 );
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
        cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;
        T_esti=Eigen::Isometry3d ( pose->estimate() ).matrix();
        Eigen::Matrix4d tt=T_esti.matrix();
        Eigen::Matrix3d rotation=tt.block(0,0,3,3);
        Eigen::Vector3d trans=tt.block(0,3,3,0);
        Mat rotation1;
        cv::eigen2cv(rotation,rotation1);
        // angle_= rotationMatrixToEulerAngles(rotation1);
        trans = (-1)*(rotation.inverse()*trans); // ** 坐标系变换  
        translation_=trans;
        // cout<<"angle: "<<angle_<<endl;
        // cout<<"translation_ : "<<translation_<<endl;
        // cout<<endl<<"after optimization:"<<endl;
        // cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;

        cv::eigen2cv(rotation,R);
        cv::eigen2cv(trans,t);
    }
}