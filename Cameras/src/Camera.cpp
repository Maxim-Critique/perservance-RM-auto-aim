#include "Camera.h"
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

atomic<int> CameraMode;//1 是外触发
atomic<double> GALAXY_EXPOSURE_TIME;//曝光时间
Mat ExternalImage;//外触发时用的图像

CameraDevice::CameraDevice()
{
    status = GX_STATUS_SUCCESS;
    CameraMode = 0;
    //    GX_DEV_HANDLE hDevice = nullptr;
    //    uint32_t nDeviceNum = 0;
    // 工业相机尺寸//1920*1080//640*360

#ifdef SHORT
    src.create(480, 640, CV_8UC3);
    // src.create(960, 1280, CV_8UC3); // 工业相机尺寸
#else
    src.create(480, 1280, CV_8UC3);
#endif
    //    src.create(1080, 1920, CV_8UC3);

    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam.openMode = GX_OPEN_INDEX;
    stOpenParam.pszContent = "1";
    nFrameNum = 0;
}

CameraDevice::~CameraDevice()
{
    status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);

    //释放图像缓冲区buffer
    free(stFrameData.pImgBuf);

    status = GXCloseDevice(hDevice);
    status = GXCloseLib();
}

void CameraDevice::CameraModeSwitch()
{
    
    
    if(Curr_EXPOSURE_TIME != GALAXY_EXPOSURE_TIME)
    {
        status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, GALAXY_EXPOSURE_TIME);
        Curr_EXPOSURE_TIME = 0;
        double nVal=0;
        if(GXGetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME,&nVal)==0)
            cout<<"nVal:"<<nVal<<endl;
            Curr_EXPOSURE_TIME = nVal;
        cout<<"DIFF_Curr_EXPOSURE_TIME:"<<Curr_EXPOSURE_TIME<<endl;
    }

    if(CameraCurrMode !=  CameraMode)
    {
        cout<<"Here !!!"<<endl;
         status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);
         if (status!=0)
            return;
            // sleep(1);
            usleep(10000);
        if(CameraMode)
        {
//                设置触发模式为 ON
            GXSetEnum(hDevice,GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
            //设置触发激活方式为上升沿
            status = GXSetEnum(hDevice,GX_ENUM_TRIGGER_ACTIVATION,GX_TRIGGER_ACTIVATION_RISINGEDGE);
            status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);
            status = GXSetEnum(hDevice, GX_ENUM_LINE_SELECTOR, GX_ENUM_LINE_SELECTOR_LINE2);
//                注册图像处理回调函数
            GXRegisterCaptureCallback(hDevice, NULL, OnFrameCallbackFun);
        }else {
            //注销采集回调
            status = GXUnregisterCaptureCallback(hDevice);
            //设置触发模式为 OFF
            status = GXSetEnum(hDevice,GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
        }
        int64_t nVal=0;
        GX_STATUS status = GXGetEnum(hDevice,GX_ENUM_TRIGGER_MODE,&nVal);
        if (status == 0)
            CameraCurrMode = nVal;
        if (CameraCurrMode ==  CameraMode)
            cout<<"------ Success!!! CameraMode: "<<CameraCurrMode<<" ------"<<endl;
        //发送开始采集命令
            status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
        
    }
}
static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)//外触发 回调函数
{
if (pFrame->status == 0)
{
//对图像进行某些操作
        //图像获取成功
        char* m_rgb_image=nullptr; //增加的内容
        m_rgb_image=new char[pFrame->nWidth*pFrame->nHeight*3];
    //    cout<<"ID"<<pFrame->nFrameID<<endl;
    //    extern int asd;
    //    std::cout << asd <<std::endl;
        void* pChar = const_cast<void*>(pFrame->pImgBuf);
        DxRaw8toRGB24(pChar,m_rgb_image,pFrame->nWidth, pFrame->nHeight,RAW2RGB_NEIGHBOUR3,DX_PIXEL_COLOR_FILTER(BAYERBG),false);
        ExternalImage.create(480,640,CV_8UC3);
//         cvtRaw2Mat(pFrame->pImgBuf,img_);
//         if (format == V4L2_PIX_FMT_MJPEG){
//             cv::Mat src(pFrame->nHeight, pFrame->nWidth, CV_8UC3, (void*) pFrame->pImgBuf);
//             img_ = cv::imdecode(src, 1);
//         }
        memcpy(ExternalImage.data,m_rgb_image,pFrame->nWidth*pFrame->nHeight*3);
//                         imshow("asdsda",img_);
//                         waitKey(1);
//         else if(format == V4L2_PIX_FMT_YUYV){
//             cv::Mat yuyv(capture_height, capture_width, CV_8UC2, (void*) data);
//             cv::cvtColor(yuyv, image, COLOR_YUV2BGR_YUYV);
//         }
//        src.copyTo(img);
//        imshow("Asd",img_);
//        waitKey(1);
//                img = src;
//        nFrameNum++;///////////

        //对图像进行处理...
        delete []m_rgb_image;
        //                        }
    }
    return;
}

int CameraDevice::init()
{
    // 初始化库
    status = GXInitLib();

    if (status != GX_STATUS_SUCCESS)
    {
        return 0;
    }
    status = GXUpdateDeviceList(&nDeviceNum, 1000);

    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0))
    {
        return 0;
    }
    status = GXOpenDevice(&stOpenParam, &hDevice);
    std::cout << "GxStatus: " << status << std::endl;
    if (status == GX_STATUS_SUCCESS)
    {
        int64_t nPayLoadSize = 0;
        //获取图像buffer大小，下面动态申请内存
        status = GXGetInt(hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);
        cout<<"status1:"<<status<<endl;
        if (status == GX_STATUS_SUCCESS && nPayLoadSize > 0)
        {
            cout<<"status2:"<<status<<endl;
            //定义GXGetImage的传入参数
            // GX_FLOAT_RANGE shutterRange;

            // double dExposureValue = 200.0;

            // status = GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME, &shutterRange);

            //根据获取的图像buffer大小m_nPayLoadSize申请buffer
            stFrameData.pImgBuf = malloc((size_t)nPayLoadSize);

            // 设置曝光值
            // status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_DELAY, dExposureValue);
            // status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, shutterRange.dMin);
            // status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, shutterRange.dMax);
            status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, GALAXY_EXPOSURE_TIME);
            // status = GXSetFloat(hDevice, GX_ENUM_EXPOSURE_AUTO, GALAXY_EXPOSURE_TIME);
            if(status==0)
                Curr_EXPOSURE_TIME = GALAXY_EXPOSURE_TIME;
                cout<<"GALAXY_EXPOSURE_TIME111:"<<GALAXY_EXPOSURE_TIME<<endl;

            //设置采集模式连续采集
            //                        status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
            status = GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
            //                        status = GXSetInt(hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 1);
            // status = GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);

            //设置ROI
            //    int64_t nWidth   = 640;
            //    int64_t nHeight  = 480;
            //    int64_t nOffsetX = 0;
            //    int64_t nOffsetY = 0;
            //    status = GXSetInt(hDevice, GX_INT_WIDTH, nWidth);
            //    status = GXSetInt(hDevice, GX_INT_HEIGHT, nHeight);
            //    status = GXSetInt(hDevice, GX_INT_OFFSET_X, nOffsetX);
            //    status = GXSetInt(hDevice, GX_INT_OFFSET_Y, nOffsetY);

            // //my white
            // //选择白平衡通道
            // status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_BLUE);
            // // //获取白平衡调节范围        
            // GX_FLOAT_RANGE ratioRange;
            // cout << "whitebalance_min:"<< ratioRange.dMin << endl;
            // cout << "whitebalance_max:"<< ratioRange.dMax << endl;
            // // //获取白平衡调节范围    
            // status = GXGetFloatRange(hDevice, GX_FLOAT_BALANCE_RATIO, &ratioRange);
            // // //设置最小白平衡系数
            // ratioRange.dMin = 1.7;
            // status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, ratioRange.dMin);
            // // //设置最大白平衡系数
            // ratioRange.dMax = 1.75;
            // status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, ratioRange.dMax);

            // status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
            // // cout << "while:"<< ratioRange.dMin << endl;
            // // //获取白平衡调节范围    
            // status = GXGetFloatRange(hDevice, GX_FLOAT_BALANCE_RATIO, &ratioRange);
            // // //设置最小白平衡系数
            // ratioRange.dMin = 2.3;
            // status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, ratioRange.dMin);
            // // //设置最大白平衡系数
            // ratioRange.dMax = 1.5;
            // status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, ratioRange.dMax);

            // status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
            // // cout << "while:"<< ratioRange.dMin << endl;
            // // //获取白平衡调节范围    
            // status = GXGetFloatRange(hDevice, GX_FLOAT_BALANCE_RATIO, &ratioRange);
            // // //设置最小白平衡系数
            // ratioRange.dMin = 2.0;
            // status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, ratioRange.dMin);
            // // //设置最大白平衡系数
            // ratioRange.dMax = 1.1;
            // status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, ratioRange.dMax);


//////////////////////////////////////// Add by Hiutung ///////////////////////////////////////////////////

            //自动白平衡设置--------------------------
            //设置自动白平衡光照环境,比如当前相机所处环境为荧光灯
            status = GXSetEnum(hDevice,GX_ENUM_AWB_LAMP_HOUSE,GX_AWB_LAMP_HOUSE_FLUORESCENCE);
            //设置连续自动白平衡
            status = GXSetEnum(hDevice,GX_ENUM_BALANCE_WHITE_AUTO,GX_BALANCE_WHITE_AUTO_CONTINUOUS);
            //环境光源预设设置-------------------------
            //设置环境光源预设为 Daylight-6500K
            status = GXSetEnum(hDevice,GX_ENUM_LIGHT_SOURCE_PRESET,GX_LIGHT_SOURCE_PRESET_DAYLIGHT_6500K);



            // // //选择黑电平通道类型
            // status = GXSetEnum(hDevice, GX_ENUM_BLACKLEVEL_SELECTOR,GX_BLACKLEVEL_SELECTOR_ALL);
            // // status=GXSetEnum(hDevice,GX_ENUM_BLACKLEVEL_SELECTOR,GX_BLACKLEVEL_SELECTOR_RED);
            // // //获取黑电平调节范围
            //  GX_FLOAT_RANGE blackLevelRange;
            //  cout << "Blacklever_min:"<< ratioRange.dMin << endl;
            //  cout << "Blacklever_max:"<< ratioRange.dMax << endl;
            //  status = GXGetFloatRange(hDevice, GX_FLOAT_BLACKLEVEL,&blackLevelRange);
            // // blackLevelRange.dMin=1.0;
            // // //设置最小黑电平值
            // // status = GXSetFloat(hDevice, GX_FLOAT_BLACKLEVEL,blackLevelRange.dMin);
            //  blackLevelRange.dMax=6.1;
            // // //设置最大黑电平值
            //  status = GXSetFloat(hDevice, GX_FLOAT_BLACKLEVEL,blackLevelRange.dMax);
    
           
            // status=GXSetEnum(hDevice,GX_ENUM_BLACKLEVEL_SELECTOR,GX_BLACKLEVEL_SELECTOR_GREEN);
            // status = GXGetFloatRange(hDevice, GX_FLOAT_BLACKLEVEL,&blackLevelRange);
            // //设置最小黑电平值
            // blackLevelRange.dMin=1.0;
            // status = GXSetFloat(hDevice, GX_FLOAT_BLACKLEVEL,blackLevelRange.dMin);
            // //设置最大黑电平值
            // blackLevelRange.dMax=2.1;
            // status = GXSetFloat(hDevice, GX_FLOAT_BLACKLEVEL,blackLevelRange.dMax);
    
            // status=GXSetEnum(hDevice,GX_ENUM_BLACKLEVEL_SELECTOR,GX_BLACKLEVEL_SELECTOR_BLUE);
            // status = GXGetFloatRange(hDevice, GX_FLOAT_BLACKLEVEL,&blackLevelRange);
            // //设置最小黑电平值
            //  blackLevelRange.dMin=1.0;
            // status = GXSetFloat(hDevice, GX_FLOAT_BLACKLEVEL,blackLevelRange.dMin);
            // //设置最大黑电平值
            //  blackLevelRange.dMax=2.1;
            // status = GXSetFloat(hDevice, GX_FLOAT_BLACKLEVEL,blackLevelRange.dMax);


            // //获取曝光调节范围
            // GX_FLOAT_RANGE shutterRange;
            // cout<<"shutterRange_min:"<<shutterRange.dMin<<endl;
            // cout<<"shutterRange_max:"<<shutterRange.dMax<<endl;
            // double dExposureValue = 2.0;
            // status = GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME,&shutterRange);
            // //设置最小曝光值
            // status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME,shutterRange.dMin);
            // //设置最大曝光值
            // status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME,shutterRange.dMax);
            //设置连续自动曝光
            // status = GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO,GX_EXPOSURE_AUTO_CONTINUOUS);
            // 设置曝光延迟为 2us
            // status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_DELAY,dExposureValue);
            // 设置曝光时间模式为极小曝光模式
            // status = GXSetEnum(hDevice, GX_ENUM_EXPOSURE_TIME_MODE,GX_EXPOSURE_TIME_MODE_ULTRASHORT);


            //假 设 当 前 相 机 支 持 GX_GAIN_SELECTOR_ALL
            //选 择 增 益 通 道 类 型
            status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
            // status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_RED);
            // status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_GREEN);
            // status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_BLUE);
            //获 取 增 益 调 节 范 围
            GX_FLOAT_RANGE gainRange;
            status = GXGetFloatRange(hDevice, GX_FLOAT_GAIN, &gainRange);
            //设 置 最 小 增 益 值
            cout << "gainRangeMIN: " << gainRange.dMin <<endl;
            //status = GXSetFloat(hDevice, GX_FLOAT_GAIN, gainRange.dMin);
            //设 置 最 大 增 益 值
            cout << "gainRangeMAX: " << gainRange.dMax <<endl;
            //gainRange.dMax = 26;
            status = GXSetFloat(hDevice, GX_FLOAT_GAIN, gainRange.dMax);
            //设 置 连 续 自 动 增 益
            //status = GXSetEnum(hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);

           
///////////////////////////////////////////////////////////////////////////////////////////////////////////////



            if(CameraMode)
            {
                //设置触发模式为 ON
                status =GXSetEnum(hDevice,GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
                //设置触发激活方式为上升沿
                status = GXSetEnum(hDevice,GX_ENUM_TRIGGER_ACTIVATION,GX_TRIGGER_ACTIVATION_RISINGEDGE);
                status = GXSetEnum(hDevice, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);  
                status = GXSetEnum(hDevice, GX_ENUM_LINE_SELECTOR, GX_ENUM_LINE_SELECTOR_LINE2);
                //注册图像处理回调函数
                status = GXRegisterCaptureCallback(hDevice, NULL, OnFrameCallbackFun);
            }
            else {
                //设置触发模式为 OFF   
                status =GXSetEnum(hDevice,GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
                //注销采集回调
                status = GXUnregisterCaptureCallback(hDevice);
            }
            cout<<"------ Success!!! CameraMode: "<<CameraCurrMode<<" ------"<<endl;

            int64_t nVal=0;
            status = GXGetEnum(hDevice,GX_ENUM_TRIGGER_MODE,&nVal);
            if (status == 0)
                CameraCurrMode = nVal;
            //发送开始采集命令
            status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
            return 1;
        }
    }
    return 0;
}

void CameraDevice::getImage(Mat &img)
{
    GXFlushQueue(hDevice);
    GXGetImage(hDevice, &stFrameData, 100);
    // usleep(1);

    if (stFrameData.nStatus == GX_FRAME_STATUS_SUCCESS)
    {
        //图像获取成功
        char *m_rgb_image = nullptr; //增加的内容
        m_rgb_image = new char[stFrameData.nWidth * stFrameData.nHeight * 3];
        DxRaw8toRGB24(stFrameData.pImgBuf, m_rgb_image, stFrameData.nWidth, stFrameData.nHeight, RAW2RGB_NEIGHBOUR3, DX_PIXEL_COLOR_FILTER(BAYERBG), false);

        memcpy(src.data, m_rgb_image, stFrameData.nWidth * stFrameData.nHeight * 3);

        src.copyTo(img);
        //        img = src;
        nFrameNum++;

        //对图像进行处理...
        delete[] m_rgb_image;
        //                        }
    }
}

uint64_t CameraDevice::getFrameNumber()
{
    return nFrameNum;
}
