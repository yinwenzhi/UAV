#include "CameraDevice.h"
#include <iostream>

using namespace cv;

Monitor::CameraDevice::CameraDevice(){
    CameraSdkInit(1);
    std::cout<< "相机SDK初始化成功"<< std::endl;
}

// offsetx, offsety, width, height: 偏移宽高都选取为16的倍数兼容性最好（不同的相机对这个的要求不同，有的只需要2的倍数，有的可能需要16的倍数）
// offsetx, offsety, width, height: offset width and height are all chosen to be 16 times the best compatibility (different cameras have different requirements for this, some need only a multiple of 2, some may require a multiple of 16)
int SetCameraResolution(CameraHandle hCamera ,int offsetx, int offsety, int width, int height)
{
    tSdkImageResolution sRoiResolution = { 0 };

    // 设置成0xff表示自定义分辨率，设置成0到N表示选择预设分辨率
    // Set to 0xff for custom resolution, set to 0 to N for select preset resolution
    sRoiResolution.iIndex = 0xff;

    // iWidthFOV表示相机的视场宽度，iWidth表示相机实际输出宽度
    // 大部分情况下iWidthFOV=iWidth。有些特殊的分辨率模式如BIN2X2：iWidthFOV=2*iWidth，表示视场是实际输出宽度的2倍
    // iWidthFOV represents the camera's field of view width, iWidth represents the camera's actual output width
    // In most cases iWidthFOV=iWidth. Some special resolution modes such as BIN2X2:iWidthFOV=2*iWidth indicate that the field of view is twice the actual output width
    sRoiResolution.iWidth = width;
    sRoiResolution.iWidthFOV = width;

    // 高度，参考上面宽度的说明
    // height, refer to the description of the width above
    sRoiResolution.iHeight = height;
    sRoiResolution.iHeightFOV = height;

    // 视场偏移
    // Field of view offset
    sRoiResolution.iHOffsetFOV = offsetx;
    sRoiResolution.iVOffsetFOV = offsety;

    // ISP软件缩放宽高，都为0则表示不缩放
    // ISP software zoom width and height, all 0 means not zoom
    sRoiResolution.iWidthZoomSw = 0;
    sRoiResolution.iHeightZoomSw = 0;

    // BIN SKIP 模式设置（需要相机硬件支持）
    // BIN SKIP mode setting (requires camera hardware support)
    sRoiResolution.uBinAverageMode = 0;
    sRoiResolution.uBinSumMode = 0;
    sRoiResolution.uResampleMask = 0;
    sRoiResolution.uSkipMode = 0;

    return CameraSetImageResolution(hCamera, &sRoiResolution);
}

int Monitor::CameraDevice::ConnectDevice(){
    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
	printf("state = %d\n", iStatus);

	printf("count = %d\n", iCameraCounts);
    //没有连接设备
    if(iCameraCounts==0){
        std::cout<< "没有连接设备,请检查相机是否连接"<< std::endl;
        return -1;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

    //初始化失败
	printf("state = %d\n", iStatus);
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        std::cout<< "相机初始化失败"<< std::endl;
        return -1;
    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);

    //
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);

    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */




    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }
    SetCameraResolution(hCamera, 904, 520, 1280, 1024);

    CameraGetImageResolution(hCamera,&pImageResolution);
    std::cout << "iwidth: " << pImageResolution.iWidth
        << " iHeight: " << pImageResolution.iHeight << std::endl;
    
    return 0;
}



Monitor::CameraDevice::~CameraDevice(){
    CameraUnInit(hCamera);
    //注意，现反初始化后再free
    free(g_pRgbBuffer);
}

cv::Mat  Monitor::CameraDevice::GetImage(cv::Mat &image){
    if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
    {
        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
        
        cv::Mat matImage(
                cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight), 
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer
                );
        // std::cout << "showimage in get Image"<< std::endl;
        // imshow("Opencv Demo", matImage);
        image = matImage.clone();
        // waitKey(5);

        //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
        //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
        CameraReleaseImageBuffer(hCamera,pbyBuffer);
        return image;
    }

}


