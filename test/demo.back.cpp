//
// Created by lizechuan on 20-7-13.
//

#include <iostream>
#include <PointMatch.h>
#include "common_include.h"
#include "Feature.h"
#include <boost/timer.hpp>
#include "Config.h"
#include "camera.h"
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include "CameraDevice.h"
using namespace std;

int main( int argc, char** argv)
{

    // if ( argc != 2 )
    // {
    //     cout<<"usage: run_vo parameter_file"<<endl;
    //     return 1;
    // }
//usb camera
    Monitor::CameraDevice camera_usb;
    if(camera_usb.ConnectDevice() == -1){
        std::cout << "camera Device connect failed.please check camera device" <<std::endl;
        return 0;
    }

    myslam::Config::setParameterFile ( argv[1] );

    //创建一个导航系统
    myslam::PointMatch::Ptr vo ( new myslam::PointMatch );
    string frameName;
    //图像序列
    ifstream listFramesFile("/home/emast/WorkSpace/programe/UAV-project/dataset/img.txt");
    //基准帧图片
    string str_img1=myslam::Config::get<string> ( "dataset_first" );
    Mat img1 = imread(str_img1, 1);
    //构建相机对象，同时从配置文件中读取相机内参（在构造函数中实现）
    myslam::Camera::Ptr camera ( new myslam::Camera );
    //提取基准帧特征点即直线交点
    myslam::Feature f;
    f.GetCrossPoint(img1);
    vector<Point2f> corners1=f._corners;

    int idxnum1=corners1.size();
    cout<<"num of first img's corners is :"<<idxnum1<<endl;
    //类型转换
    vector<KeyPoint> keypoints1(idxnum1);
    KeyPoint::convert(corners1, keypoints1);
    //计算描述子
    Mat descriptors_1;
    // Ptr<DescriptorExtractor> descriptor = ORB::create();
    // descriptor->compute ( img1, keypoints1, descriptors_1 );

    cv:: Ptr<Feature2D> descriptor = BRISK::create();
    descriptor->compute ( img1, keypoints1, descriptors_1 );


    cout<<"descriptors_1 :"<<descriptors_1.size()<<endl;
    //创建基准帧
    myslam::Frame::Ptr pF_ini = myslam::Frame::createFrame();

    pF_ini->color_ = img1;//读取图片
    pF_ini->time_stamp_ = 0;//时间戳
    pF_ini->id_=0;
//  读入基准帧的3D点坐标（夹板坐标系下）
    vector<KeyPoint> keypoints;//和3d点对应的特征点
    Mat descriptors;//和3d点对应的描述子
    //从文档中读取3D坐标
    string str_3d = myslam::Config::get<string> ( "dataset_3d" );
    cout<<"dataset: "<<str_3d<<endl;
    double num,x,y,z;
    for ( size_t i=0; i<keypoints1.size(); i++ )
    {
        ifstream myfile(str_3d);
        if (!myfile.is_open()) {
            cout << "can not open this file" << endl;
        }
        while(myfile>>num>>x>>y>>z)
        {
            if(i==num)
            {
                Point3f p_world;
                p_world.x=x;
                p_world.y=y;
                p_world.z=z;
                pF_ini->pts_3d_ref_.push_back(p_world);
                keypoints.push_back(keypoints1[i]);
                descriptors.push_back(descriptors_1.row(i).clone());
            }
            else
            {
                continue;
            }
        }
    }
    vo->keypoints_ref_ = keypoints;
   // cout<<"keypoints :"<<keypoints.size()<<endl;
   // cout<<"3d point :"<<pF_ini->pts_3d_ref_.size()<<endl;
   // cout<<"descriptors :"<<descriptors.size()<<endl;
    vo->pts_3d_=pF_ini->pts_3d_ref_;
    vo->descriptors_ref_=descriptors;
    vo->ref_=pF_ini;
    vo->camera_=camera;
    int k=0;



    for(int n = 0; n < keypoints1.size(); n++)
    {
        Scalar cc =   Scalar::all(-1);
        // circle(img1, keypoints1[n].pt, 1, Scalar(255, 0, 255), 2, 8, 0);
        circle(img1, keypoints1[n].pt, 1, cc, 2, 8, 0); //随机颜色
        // circle(img1,corners1[n],1,Scalar(255,0,255),2,8,0);
        cv::putText(img1,std::to_string(n), keypoints1[n].pt, cv::FONT_HERSHEY_SIMPLEX,0.60, CV_RGB(0, 0, 0),1,8);
        // circle(edges, vo->keypoints_curr_[n].pt, 1, Scalar(255, 0, 255), 2, 8, 0);
    }
    imshow("ref fram",img1);

    cout<<"******"<<"导航开始"<<"*******"<<endl;
    namedWindow("img_goodmatch",0);
    cv::resizeWindow("img_goodmatch",640,480);
    // namedWindow("edges2",1);
    // namedWindow("result",1);
    // namedWindow("img2,diyici",1);
    // namedWindow("当前帧",0);
    // cv::resizeWindow("当前帧",640,480);
    cv::Mat edges; //定义一个Mat变量，用于存储每一帧的图像
   // while ( getline(listFramesFile, frameName) ) {
   while (1){
        cout<<"*************"<<endl;
        camera_usb.GetImage(edges);
        // imshow("当前帧",edges);
        // imwrite("test.png",edges);
        std::cout << "write image" << std::endl;
        // waitKey(0);
        // frameName ="/home/emast/WorkSpace/programe/UAV-project/dataset/picture/" + frameName;
        Mat img2 = edges.clone();
        // std::cout << "&edges:" << &edges<< std::endl;
        // std::cout << "&img2:" << &img2<< std::endl;
        cout<<"当前为第"<<" "<<k+1<<" "<<"帧"<<endl;
        if ( img2.data==nullptr )
            break;
    //    namedWindow("result",0);
    //    imshow("result", edges);
    //    waitKey(0);
        char c1 = cvWaitKey(33);
        if(c1 == 27) {
            break;
        }
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        //pFrame->camera_ = camera;
        pFrame->color_ = img2;
        //pFrame->depth_ = depth;
        pFrame->id_ =k++;
        pFrame->time_stamp_ =0;
        boost::timer timer;

        // imshow("img2,diyici",img2);
        // waitKey(0);
        // imshow("edges2",edges);
        if(!vo->addFrame ( pFrame ))
        {
            cout<<"skip current frame"<<endl;
            continue;
        }
        // imshow("edges3",edges);
        cout<<"VO costs time: "<<timer.elapsed()<<endl;
        Eigen::Matrix4d T=vo->T_esti.matrix();
        Vec3f angle=vo->angle_;
        Vector3d trans=vo->translation_;
        int a1=20,b1=300;
        for(int m =0;m<3;m++)
        {
                double a=angle[m];
                double b=trans[m];
                cv::Point2f num1 (a1+m*150,200);
                cv::Point2f num2 (a1+m*150,230);
                cv::putText(img2,std::to_string(a*180/3.1415926), num1, cv::FONT_HERSHEY_SIMPLEX,0.60, CV_RGB(0, 0, 0),1,8);
                cv::putText(img2,std::to_string(b), num2, cv::FONT_HERSHEY_SIMPLEX,0.60, CV_RGB(0, 0, 0),1,8);
                // cv::putText(edges,std::to_string(a*180/3.1415926), num1, cv::FONT_HERSHEY_SIMPLEX,0.60, CV_RGB(0, 0, 0),1,8);
                // cv::putText(edges,std::to_string(b), num2, cv::FONT_HERSHEY_SIMPLEX,0.60, CV_RGB(0, 0, 0),1,8);

        }

        for(int n = 0; n < vo->keypoints_curr_.size(); n++)
        {
            circle(img2, vo->keypoints_curr_[n].pt, 1, Scalar(255, 0, 255), 2, 8, 0);
            // circle(edges, vo->keypoints_curr_[n].pt, 1, Scalar(255, 0, 255), 2, 8, 0);
        }
        // imshow("当前帧",img2);
        cv::Mat img_goodmatch;
        drawMatches ( 
            vo->ref_->color_, 
            keypoints,   // 因为匹配使用描述子是给定3D点的描述子，所以关键点也是3D点
            img2, 
            vo->keypoints_curr_, 
            vo->feature_matches_, 
            img_goodmatch ,
            Scalar(255,0,0) );
        imshow("img_goodmatch",img_goodmatch);
        // waitKey(0);
        
        // waitKey(0);
        char c = cvWaitKey(33);
        if(c == 27) {
            break;
        }else if (c==112)
        {
            waitKey(0);
        }
        // waitKey(0);
    }
    cout<<"******"<<"导航结束"<<"*******"<<endl;
}