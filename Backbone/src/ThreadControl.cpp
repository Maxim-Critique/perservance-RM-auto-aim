#include "ThreadControl.h"
static std::atomic<bool> end_thread_flag(false);
std::atomic<unsigned int> produce_index(0);     // 图像生成序号，用于线程之间逻辑 sem_t
std::atomic<unsigned int> consumption_index(0);// 图像消耗序号
struct timespec start_time, end_time;
Camera::HikCamera camera;
void signal_handler(int signal) {
    try {
        if (signal == SIGINT) {
            end_thread_flag = true;
            std::cout << "\nExit signal received." << std::endl;
            // 确保相机关闭
            camera.close();
            
            // 关闭所有 OpenCV 窗口
            cv::destroyAllWindows();
        }
    } catch (...) {
        std::cerr << "Error during shutdown!" << std::endl;
    }
    end_thread_flag = true;
}
int keyboard_key;
#define BUFFER_SIZE 1
#define END_THREAD if (end_thread_flag.load()) return;

static ImageData data1[BUFFER_SIZE];
static ImageData IMG_data[BUFFER_SIZE];


// 保存图片地质，用移动硬
string Writepath = "/media/zhbit/Hiutung/camera_calibration/";
int counting = 0;
string Photo_name;
double m_next_shoot_time,elapsed_time_ms;               // 下次允许射击时间

ThreadControl::ThreadControl()
{
    readYAML(Yaml);
    cout << "THREAD TASK ON !!!" << endl;
    std::signal(SIGINT, signal_handler);  // 注册信号处理
}


// 图像生成线程
void ThreadControl::ImageProduce()
{
    if(video_write_enable){
        const auto now = std::chrono::system_clock::now();
        const std::string video_path = "../videos/" + std::to_string(now.time_since_epoch().count()) + ".avi";
        std::filesystem::create_directory("../videos"); 
        // 初始化视频写入器
        video_writer = std::make_unique<cv::VideoWriter>(video_path, 
            cv::VideoWriter::fourcc('M','J','P','G'), 80.0, cv::Size(1440, 1080));
    }
#ifdef CAMERA
    cout << " ------ SHORT CAMERA PRODUCE TASK ON !!! ------ " << endl;
    
    camera.open();
    camera.setGain(1);//设置为自动手动要改一下
    camera.setGamma(0);
    
#endif  
#ifdef CAMERA1
    CameraDevice galaxy;
    cout << " ------ SHORT CAMERA PRODUCE TASK ON !!! ------ " << endl;
    
    galaxy.init();
#endif  
#ifdef VIDEO
    cout << " ------ SHORT VEDIO PRODUCE TASK ON !!! ------ " << endl;
    VideoCapture capture;
    capture.open(VIDEO_ADD);
    if (!capture.isOpened())
    {
        cout << "can not load the video" << endl;
        exit(EXIT_FAILURE);
    }
#endif
//---->            <----//

    while (!end_thread_flag.load()) {   
#ifdef CAMERA
        data1[produce_index % BUFFER_SIZE].img = camera.getImage();
        data1[produce_index % BUFFER_SIZE].timestamp = camera.getLastTimestamp();
        
        while(produce_index - consumption_index >= BUFFER_SIZE ){END_THREAD;}            
        ++produce_index;    
        if(video_write_enable){  
            if (video_writer && video_writer->isOpened()) {
                *video_writer << data1[produce_index % BUFFER_SIZE].img;
            }
        }
        if (end_thread_flag.load()) break;  
#endif            
#ifdef CAMERA1

        galaxy.CameraModeSwitch();
        if(!CameraMode)
        {
            galaxy.getImage(data1[produce_index % BUFFER_SIZE].img);
            while(produce_index - consumption_index >= BUFFER_SIZE && CameraMode ==0)
                END_THREAD;            
            ++produce_index;        
        } 
#endif
#ifdef VIDEO
        clock_gettime(CLOCK_MONOTONIC, &start_time);
        while (produce_index - consumption_index >= BUFFER_SIZE)
            END_THREAD; 
        capture.read(IMG_data[produce_index % BUFFER_SIZE].img);
        ++produce_index;
#endif
    }
    if(video_write_enable){  
        if (video_writer && video_writer->isOpened()) {
            video_writer->release();
            std::cout << "Video file saved successfully." << std::endl;
        }
    }
    camera.close(); 
}

// 图像处理线程
void ThreadControl::ImageProcess()
{ 

    vector<double> time_add_vec;
    cout << " ------ IMAGE PROCESS TASK ON !!! ------" << endl;
    Mat image;
    double fps;
    u_int64_t timestamp;
    Auto_aim auto_aim;
    Buff_Task buff_task;
    while (!end_thread_flag.load()) 
    {
     
#ifdef CAMERA
    // 等待图像生成后进行处理
        while(produce_index - consumption_index == 0){
        }
        data1[consumption_index % BUFFER_SIZE].img.copyTo(image);
        timestamp = data1[consumption_index % BUFFER_SIZE].timestamp;
        ++consumption_index;
        //cout<<image.cols  <<image.rows<<endl;
#endif
#ifdef CAMERA1
    // 等待图像生成后进行处理
        while(produce_index - consumption_index == 0){
        }
        data1[consumption_index % BUFFER_SIZE].img.copyTo(image);
        ++consumption_index;
        //cout<<image.cols  <<image.rows<<endl;
#endif
#ifdef VIDEO
        while (produce_index - consumption_index == 0)
        {
        }
        IMG_data[consumption_index % BUFFER_SIZE].img.copyTo(image);
        ++consumption_index;
#endif
        if (image.empty())
        {
            continue;  // 跳过本轮循环
        }


//////////////////////////////////////////////////////////////////////////////
#ifdef CapturingPhoto     
        /***********MAIN**************/                                    ///
        switch (keyboard_key)                                              ///
        {                                                                  ///
        case 'm' :                                                         ///
            Photo_name = Writepath+to_string(counting)+".jpg";             ///
            imwrite(Photo_name,image);                                     ///
            cout << Photo_name << endl;                                    ///
            counting++;                                                    ///
        break;                                                             ///
        }                                                                  ///
#endif                                                                     ///
//////////////////////////////////////////////////////////////////////////////
        {
            std::unique_lock<std::mutex> lock(ds_mutex);
            if(!ds.Buff_or_Armor){
                lock.unlock();
                camera.setExposureTime(exposure_time_aim);
                auto_aim.IAmFuture(image,timestamp);
            }else{
                lock.unlock();
                camera.setExposureTime(exposure_time_buff);                                                                                                                                                                                             
                buff_task.SeizeGlory(image,timestamp);
            }
        }
        
        if(show){
            ShowYawPitchRoll(image);
            imshow("Linux_ubuntu",image);
            keyboard_key = waitKey(WAITKEY_VALUE);
        }
        
        if (keyboard_key == 27)
            {
                end_thread_flag = true;
                camera.close();
            }
        switch (keyboard_key)
        {
        case 'p':
            camera.close();
        }
    }
}

void ThreadControl::GetSTM32()
{
    cout << " ------ STM32 DATA RECEVICE TASK ON !!! ------" << endl;
    
    while (1)
    { 
        
        serial_.read_data();
        //  for (size_t i = 0; i < 200; i++)
        //  {
        //      for (size_t j = 0; j < 2000; j++);
        //  }
        END_THREAD;
    }
}

YAML::Node ThreadControl::readYAML(const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);
        exposure_time_aim = config["CAMERA"]["exposure_time_aim"].as<int>();
        exposure_time_buff = config["CAMERA"]["exposure_time_buff"].as<int>();
        // video_write_enable = config["CAMERA"]["video_write_enable"].as<double>();
        show = config["CAMERA"]["show"].as<int>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("读取 YAML 文件失败: " + std::string(e.what()));
    }
    return YAML::LoadFile(filename);
}



//在画面中展示Yaw,Pitch
void ShowYawPitchRoll(Mat &image) {
    std::lock_guard<std::mutex> lock(ds_mutex);
    // 弧度显示
    putText(image, "Gyro data (radian)", Point(1200, 460), FONT_HERSHEY_COMPLEX, 0.6, Scalar(115, 170, 125), 1);
    putText(image, "Y: " + to_string(ds.camera_yaw_angle), Point(1200, 500), FONT_HERSHEY_COMPLEX, 0.6, Scalar(115, 170, 125), 1);
    putText(image, "P: " + to_string(ds.camera_pit_angle), Point(1200, 530), FONT_HERSHEY_COMPLEX, 0.6, Scalar(115, 170, 125), 1);
    putText(image, "R: " + to_string(ds.camera_roll_angle), Point(1200, 560), FONT_HERSHEY_COMPLEX, 0.6, Scalar(115, 170, 125), 1);
    
    // 角度显示
    putText(image, "Gyro data (degree)", Point(1200, 620), FONT_HERSHEY_COMPLEX, 0.6, Scalar(200, 130, 15), 1);
    putText(image, "Y: " + to_string(ds.camera_yaw_angle / PI * 180), Point(1200, 660), FONT_HERSHEY_COMPLEX, 0.6, Scalar(200, 130, 15), 1);
    putText(image, "P: " + to_string(ds.camera_pit_angle / PI * 180), Point(1200, 690), FONT_HERSHEY_COMPLEX, 0.6, Scalar(200, 130, 15), 1);
    putText(image, "R: " + to_string(ds.camera_roll_angle / PI * 180), Point(1200, 720), FONT_HERSHEY_COMPLEX, 0.6, Scalar(200, 130, 15), 1);
}