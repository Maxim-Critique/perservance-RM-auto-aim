#include "ThreadControl.h"

static std::atomic<bool> end_thread_flag(false);
std::atomic<unsigned int> produce_index(0);
std::atomic<unsigned int> consumption_index(0);
struct timespec start_time, end_time;
Camera::HikCamera camera;
std::string YamlPath = "/home/killer_machine/WRITE/text.yaml";
void signal_handler(int signal) {
    try {
        if (signal == SIGINT) {
            end_thread_flag = true;
            std::cout << "\nExit signal received." << std::endl;
            camera.close();
            cv::destroyAllWindows();
        }
    } catch (...) {
        std::cerr << "Error during shutdown!" << std::endl;
    }
    end_thread_flag = true;
}

int keyboard_key;

// 保存图片地址
string Writepath = "/media/zhbit/Hiutung/camera_calibration/";
int counting = 0;
string Photo_name;
double m_next_shoot_time, elapsed_time_ms;

ThreadControl::ThreadControl()
{
    readYAML(YamlPath);
    cout << "THREAD TASK ON !!!" << endl;
    std::signal(SIGINT, signal_handler);
}

// 图像生成线程 - 简化版本
void ThreadControl::ImageProduce()
{
    if(video_write_enable){
        const auto now = std::chrono::system_clock::now();
        const std::string video_path = "../videos/" + std::to_string(now.time_since_epoch().count()) + ".avi";
        std::filesystem::create_directory("../videos"); 
        video_writer = std::make_unique<cv::VideoWriter>(video_path, 
            cv::VideoWriter::fourcc('M','J','P','G'), 80.0, cv::Size(1440, 1080));
    }
    
#ifdef CAMERA
    cout << " ------ SHORT CAMERA PRODUCE TASK ON !!! ------ " << endl;
    camera.open();
    camera.setGain(1);
    camera.setGamma(0);
    camera.setExposureTime(exposure_time_aim);
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

    while (!end_thread_flag.load()) {   
#ifdef CAMERA
        cv::Mat new_frame = camera.getImage();
        uint64_t timestamp = camera.getLastTimestamp();
        
        std::unique_lock<std::mutex> lock(frame_mutex);
        shared_frame = new_frame;
        frame_updated = true;
        lock.unlock();
     
        if(video_write_enable){  
            if (video_writer && video_writer->isOpened()) {
                *video_writer << new_frame;
            }
        }
        
#endif            
        
#ifdef CAMERA1
        galaxy.CameraModeSwitch();
        if(!CameraMode) {
            cv::Mat new_frame;
            galaxy.getImage(new_frame);
            
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                shared_frame = new_frame;
                frame_updated = true;
            }
            
            ++produce_index;
        } 
#endif
        
#ifdef VIDEO
        cv::Mat new_frame;
        capture.read(new_frame);
        
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            shared_frame = new_frame;
            frame_updated = true;
        }
        
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

// 图像处理线程 - 简化版本
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
        bool has_new_frame = false;
        cv::Mat frame_to_process;

        std::unique_lock<std::mutex> lock_img(frame_mutex);
        if (frame_updated && !shared_frame.empty()) {
            frame_to_process = shared_frame.clone();
            frame_updated = false;
            has_new_frame = true;
            ++consumption_index;
        }
        lock_img.unlock();
        
        if (!has_new_frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        
        if (frame_to_process.empty()) {
            continue;
        }

//////////////////////////////////////////////////////////////////////////////
#ifdef CapturingPhoto     
        /***********MAIN**************/
        switch (keyboard_key)
        {
        case 'm' :
            Photo_name = Writepath+to_string(counting)+".jpg";
            imwrite(Photo_name, frame_to_process);
            cout << Photo_name << endl;
            counting++;
        break;
        }
#endif
//////////////////////////////////////////////////////////////////////////////
        
        std::unique_lock<std::mutex> lock(ds_mutex);
        if(!ds.Buff_or_Armor){
            lock.unlock();
            auto_aim.IAmFuture(frame_to_process, timestamp);
        } else {
            lock.unlock();                                                                                                                                                                                
            buff_task.SeizeGlory(frame_to_process, timestamp);
        }
        
        if(show){
            ShowYawPitchRoll(frame_to_process);
            imshow("Linux_ubuntu", frame_to_process);
            cv::waitKey(1);
        }
    }
}

void ThreadControl::GetSTM32()
{
    cout << " ------ STM32 DATA RECEVICE TASK ON !!! ------" << endl;
    
    while (!end_thread_flag.load())
    { 
        serial_.read_data();
        if (end_thread_flag.load()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

YAML::Node ThreadControl::readYAML(const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);
        exposure_time_aim = config["CAMERA"]["exposure_time_aim"].as<int>();
        exposure_time_buff = config["CAMERA"]["exposure_time_buff"].as<int>();
        show = config["CAMERA"]["show"].as<int>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("读取 YAML 文件失败: " + std::string(e.what()));
    }
    return YAML::LoadFile(filename);
}

//在画面中展示Yaw,Pitch
void ShowYawPitchRoll(Mat &image) {
    std::lock_guard<std::mutex> lock(ds_mutex);
    putText(image, "Gyro data (radian)", Point(1200, 460), FONT_HERSHEY_COMPLEX, 0.6, Scalar(115, 170, 125), 1);
    putText(image, "Y: " + to_string(ds.camera_yaw_angle), Point(1200, 500), FONT_HERSHEY_COMPLEX, 0.6, Scalar(115, 170, 125), 1);
    putText(image, "P: " + to_string(ds.camera_pit_angle), Point(1200, 530), FONT_HERSHEY_COMPLEX, 0.6, Scalar(115, 170, 125), 1);
    putText(image, "R: " + to_string(ds.camera_roll_angle), Point(1200, 560), FONT_HERSHEY_COMPLEX, 0.6, Scalar(115, 170, 125), 1);
    
    putText(image, "Gyro data (degree)", Point(1200, 620), FONT_HERSHEY_COMPLEX, 0.6, Scalar(200, 130, 15), 1);
    putText(image, "Y: " + to_string(ds.camera_yaw_angle / PI * 180), Point(1200, 660), FONT_HERSHEY_COMPLEX, 0.6, Scalar(200, 130, 15), 1);
    putText(image, "P: " + to_string(ds.camera_pit_angle / PI * 180), Point(1200, 690), FONT_HERSHEY_COMPLEX, 0.6, Scalar(200, 130, 15), 1);
    putText(image, "R: " + to_string(ds.camera_roll_angle / PI * 180), Point(1200, 720), FONT_HERSHEY_COMPLEX, 0.6, Scalar(200, 130, 15), 1);
}