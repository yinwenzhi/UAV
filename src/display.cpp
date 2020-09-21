
#include "display.h"

#define pi 3.1415926

using namespace cv;



void display(cv::Mat img, Eigen::Vector3d trans,int k, long dt){
            //*******符号显示
	    double x; //蝌蚪x坐标
        double y; //蝌蚪y坐标
        double r_1=5; //蝌蚪小尾巴长度
        double e_x1;  //因为视频中x代表飞机的y方向，y方向的速度误差
        double e_y1;  //因为视频中y代表飞机的z方向，z方向的速度误差
        double u;  //实际的飞机x位置
        double v;  //实际的飞机y位置
        double r_2=20;
        double v_x;  //W中心的x坐标
        double v_y;  //W中心的y坐标
        double z=20; //W宽度的1/4
        double e_x;  //因为视频中x代表飞机的y方向，y方向的位置误差
        double e_y;  //因为视频中y代表飞机的z方向，z方向的位置误差
        double l_1;
        double l_2=40; //位置误差偏差线的长度
        double e_x1_max=0.03*0.2;//最大速度误差，期望速度×20%
        double e_y1_max=0.03*0.2;
        double e_x_max=20/350;//最大位置误差
        double e_y_max=20/350;

        uint8_t lineWidth = 10;  // 画线宽度


        Eigen::Vector3d pos=trans;      //position
        Eigen::Vector3d prepos;   //save position
        Eigen::Vector3d depos;    //desired position .denpends on pos.
        Eigen::Vector3d predepos; //save desired position
        Eigen::Vector3d errp;       //position error
        Eigen::Vector3d deve;       //desired velocity
        Eigen::Vector3d ve;         //velocity
        Eigen::Vector3d errv;       //velocity error

        x=img.cols/2;  //根据视频的大小确定位置
        y=3*img.rows/4;

        v_x=img.cols/2;
        v_y=img.rows/4;

        l_1=img.cols/3;

        //desired position
        depos(1)=0;
        depos(2)=pos(0)*tan(3.5/180*pi);

        //position error
        errp=depos-pos;


        if(k==0)
        {
            errv(0)=0;
            errv(1)=0;
            errv(2)=0;
        }
        else
        {
            //desired veloc1ity
            deve(0)=0;
            deve(1)=0;
            deve(2)=(depos(2)-predepos(2))/dt;

            //velocity
            ve=(pos-prepos)/dt;

            //velocity error
            errv=deve-ve;
        }

        e_x1=errv(1)*(img.cols/8)/e_x1_max;   // 速度矢量y误差（期望-实际）
        if(e_x1>=img.cols/8)   //超过界限值保持在界限值不变
            e_x1=img.cols/8;
        if(e_x1<=-img.cols/8)   //超过界限值保持在界限值不变
            e_x1=-img.cols/8;

        e_y1=-errv(2)*(img.cols/10)/e_y1_max;  // 速度矢量z误差（期望-实际）（带一个负号的）由于图像坐标向下为正
        if(e_y1>=img.cols/10)
            e_y1=img.cols/10;
        if(e_y1<=-img.cols/10)
            e_y1=-img.cols/10;

        u=x-e_x1;//实际速度矢量的y坐标
        v=y-e_y1;//实际速度矢量的z坐标

        e_x=errp(1)*(3*img.cols/7)/e_x_max;//位置矢量
        if(e_x>=3*img.cols/7)//(期望-实际)
            e_x=3*img.cols/7;
        if(e_x<=-3*img.cols/7)//(期望-实际)
            e_x=-3*img.cols/7;

        e_y=-errp(2)*(1*img.cols/7)/e_y_max;//（期望-实际）带一个负号由于图像坐标向下为正
        if(e_y>=1*img.cols/7)
            e_y=1*img.cols/7;
        if(e_y<=-1*img.cols/7)
            e_y=-1*img.cols/7;

        int move = 200;
        cv::circle(img, Point(x, y-move), r_1, Scalar(0, 0, 255), lineWidth);//蝌蚪的圆
        cv::line(img , Point(x, y-r_1-move) ,Point(x,y-3*r_1-move), Scalar(0, 0, 255),lineWidth);//蝌蚪的直线

        cv::line(img , Point(x-5*r_1, y-move) ,Point(x-l_1,y-move), Scalar(255, 0, 0),lineWidth);//虚拟下滑道左线 蓝色长线
        cv::line(img , Point(x-l_1,y-move) ,Point(x-l_1,y+2*r_1-move), Scalar(255, 0, 0),lineWidth);
        cv::line(img , Point(x+5*r_1, y-move) ,Point(x+l_1,y-move), Scalar(255, 0, 0),lineWidth);//虚拟下滑道右线
        cv::line(img , Point(x+l_1,y-move) ,Point(x+l_1,y+2*r_1-move), Scalar(255, 0, 0),lineWidth);
        
        cv::circle(img,Point(u, v-move),r_1, Scalar(0, 255, 0),lineWidth);//实际速度矢量的圆
        cv::line(img  , Point(u-r_1,v-move),Point(u-3*r_1,v-move),Scalar(0, 255, 0),lineWidth);//实际速度矢量左横
        cv::line(img , Point(u+r_1,v-move),Point(u+3*r_1,v-move),Scalar(0, 255, 0),lineWidth);//实际速度矢量右横
        cv::line(img , Point(u,v-r_1-move),Point(u,v-3*r_1-move),Scalar(0, 255,0),lineWidth);//实际速度矢量上横

        cv::line(img , Point(v_x,v_y),Point(v_x+z,v_y+z),Scalar(255, 255, 0),lineWidth) ;//W
        cv::line(img  , Point(v_x+z,v_y+z),Point(v_x+2*z,v_y),Scalar(255, 255, 0),lineWidth) ;
        cv::line(img  , Point(v_x,v_y),Point(v_x-z,v_y+z),Scalar(255, 255, 0),lineWidth );
        cv::line(img  , Point(v_x-z,v_y+z),Point(v_x-2*z,v_y),Scalar(255, 255, 0),lineWidth) ;
        
        // cv::line(img  , Point(v_x+e_x,v_y-l_2/2),Point(v_x+e_x,v_y+l_2/2),Scalar(255,0,255),lineWidth);//x方向的位置误差  竖直方向
        // cv::line(img , Point(v_x-l_2/2,v_y+e_y),Point(v_x+l_2/2,v_y+e_y),Scalar(255,0,255),lineWidth);//y方向的位置误差 水平方向
        cv::line(img  , Point(630,600),Point(630,660),Scalar(255,0,255),lineWidth);//x方向的位置误差  竖直方向
        cv::line(img , Point(580,500),Point(670,500),Scalar(255,0,255),lineWidth);//y方向的位置误差 水平方向

        return;
}

