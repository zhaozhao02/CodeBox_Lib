#include <string>
#include <memory>
#include <signal.h>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"
#include "camera/srv/save_image.hpp"  // 自定义服务的头文件


//ros2 run <package_name> <node_executable> --ros-args -p device:="/dev/video1" -p width:=1280 -p height:=720 -p framerate:=60 -p bitrate:=4000 -p IP:="192.168.0.10" -p PORT:=6000

//ros2 service call /save_image camera/srv/SaveImage "{file_path: '/home/khadas/saved_image.jpg'}"


cv::Mat current_frame;  // 保存图片的矩阵

// 捕捉 Ctrl+C 信号的处理函数
void signal_handler([[maybe_unused]] int signal) {
    
    rclcpp::shutdown();
}

// 定义自定义节点类，继承自 rclcpp::Node
class CameraNode : public rclcpp::Node {
public:
    

    CameraNode() : Node("usb_cam_forward"), frame_count_(0) {
        //frame_count_是帧数
        // 声明参数
        this->declare_parameter<std::string>("device", "/dev/video6");
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);
        this->declare_parameter<int>("framerate", 30);
        this->declare_parameter<int>("bitrate", 100000); // kbps
        this->declare_parameter<bool>("show_video", false);
        this->declare_parameter<std::string>("enc", "nvv4l2h264enc");
        this->declare_parameter<std::string>("IP", "192.168.1.123");
        this->declare_parameter<int>("PORT", 5000);

        // 获取参数
        this->get_parameter("device", device_);
        this->get_parameter("width", width_);
        this->get_parameter("height", height_);
        this->get_parameter("framerate", framerate_);
        this->get_parameter("bitrate", bitrate_);
        this->get_parameter("show_video", show_video_);
        this->get_parameter("enc", enc_);
        this->get_parameter("IP", IP_);
        this->get_parameter("PORT", PORT_);

        // 创建保存图片服务
        save_image_service_ = this->create_service<camera::srv::SaveImage>("save_image", std::bind(&CameraNode::save_image_callback, this, std::placeholders::_1, std::placeholders::_2));


        //使用 GStreamer 管道创建 OpenCV 视频捕获对象
       /* std::string gst_pipeline = "v4l2src device=" + device_ + " ! "
                                + "image/jpeg, width=" + std::to_string(width_) + ", height=" + std::to_string(height_) + ", framerate=" + std::to_string(framerate_) + "/1 ! "
                                + " tee name=t "

                                + "! queue max-size-buffers=1 leaky=1 ! "
                                + "jpegdec ! videoconvert ! videorate ! video/x-raw,framerate=" + std::to_string(framerate_) + "/1 ! "
                                + "amlvenc bitrate=" + std::to_string(bitrate_) + " ! "
                                + "rtph264pay ! udpsink host=192.168.137.1 port=5000 sync=false async=false "

                                + "t. ! queue max-size-buffers=1 leaky=1 ! jpegdec ! videoconvert ! videorate ! video/x-raw,framerate=" + std::to_string(framerate_) + "/1 ! appsink ";
*/
        // std::string gst_pipeline = "v4l2src device=" + device_ + " ! "
        //     + "video/x-raw, width=" + std::to_string(width_) + ", height=" + std::to_string(height_) + ", framerate=" + std::to_string(framerate_) + "/1 ! "
        //     + "tee name=t ! "
        //     + "queue max-size-buffers=1 leaky=1 ! "
        //     + "queue ! videoconvert ! "+enc_ + " tune=zerolatency"+ " preset=ultrafast" +" bitrate=" + std::to_string(bitrate_) + " ! rtph264pay ! udpsink host="+IP_ +" port="+std::to_string(PORT_)+" sync=false async=false "
        //     //+ "t. ! queue max-size-buffers=1 leaky=1 ! jpegdec ! videoconvert ! videorate ! video/x-raw,framerate=" + std::to_string(framerate_) + "/1 ! appsink ";
        //     + "t. ! queue ! videoconvert ! appsink"; //软件enc
        // std::string gst_pipeline = "v4l2src device=" + device_ + " ! "
        //     + "video/x-raw,format=YUY2,width=" + std::to_string(width_) + ",height=" + std::to_string(height_) + ",framerate=" + std::to_string(framerate_) + "/1 ! "
        //     + "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
        //     + "nvv4l2h264enc bitrate=" + std::to_string(bitrate_) + " preset-level=2 insert-sps-pps=true maxperf-enable=1 ! "
        //     + "rtph264pay ! udpsink host=" + IP_ + " port=" + std::to_string(PORT_) + " sync=false async=false "
        //     + "t. ! queue ! videoconvert ! appsink emit-signals=true drop=true max-buffers=1";  // 使用 appsink 获取图像流
        std::string gst_pipeline = IP_;

        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline: %s", gst_pipeline.c_str());

        cap_.open(gst_pipeline, cv::CAP_GSTREAMER);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Can't Open Camera ...");
            rclcpp::shutdown();
            return;
        }

        // 设置显示窗口的大小
        display_width_ = 1280;
        display_height_ = 720;

        // 创建定时器，每隔33ms（约30 FPS）执行一次
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&CameraNode::capture_frame, this));

        // 初始化开始时间
        start_time_ = std::chrono::high_resolution_clock::now();
        // 创建存储图像的目录
        std::filesystem::create_directory("image_capture");
    }

    ~CameraNode() {
        // 确保在析构时释放资源
        cap_.release();
        cv::destroyAllWindows();

        RCLCPP_INFO(this->get_logger(), "CameraNode shutdown");
    }
private:
    // 成员变量
    cv::VideoCapture cap_;
    std::string device_,enc_,IP_;
    int width_, height_, framerate_, bitrate_;
    bool show_video_;
    int PORT_;
    int display_width_, display_height_;
    int frame_count_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 保存图片服务
    rclcpp::Service<camera::srv::SaveImage>::SharedPtr save_image_service_;


    // 定时器回调函数
    void capture_frame() {
        cv::Mat image, resized_image;

        cap_ >> image;
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Can't Rev. Video Frames ...");
            rclcpp::shutdown();
            return;
        }

        // 更新 current_frame
        current_frame = image.clone();  // 使用 clone() 确保保存的是图像的副本

        if (show_video_) {
            // 调整图像的显示尺寸
            cv::resize(image, resized_image, cv::Size(display_width_, display_height_));
            // 显示捕获并调整后的图像
            cv::imshow("Captured Video", resized_image);
            if (cv::waitKey(1) == 27) {  // 按下 ESC 键退出
                rclcpp::shutdown();
            }
        }

        // 计算并显示 FPS
        frame_count_++;
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time_;

        if (elapsed.count() >= 1.0) {  // 更新一次
            double fps = frame_count_ / elapsed.count();
            RCLCPP_INFO_STREAM(this->get_logger(), "FPS: " << fps);
            frame_count_ = 0;
            start_time_ = end_time;
        }
    }


    // 保存图片的逻辑
    bool save_image(const std::string &filename) {
        RCLCPP_INFO(this->get_logger(), "waiting...");
        if (!current_frame.empty()) {
            cv::imwrite(filename, current_frame);
            RCLCPP_INFO(this->get_logger(), "Image saved as %s", filename.c_str());
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "No image available to save.");
            return false;
        }
    }

    // 保存图片的回调函数
    void save_image_callback(const std::shared_ptr<camera::srv::SaveImage::Request> request,
                             std::shared_ptr<camera::srv::SaveImage::Response> response) {
        return;
        // 获取当前时间并格式化为文件名
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm *now_tm = std::localtime(&now_time);
        // 格式化文件名为 image_capture/captured_image_YYYYMMDD_HHMMSS.jpg
        std::ostringstream filename;
        filename << "image_capture/captured_image_"
                << std::put_time(now_tm, "%Y%m%d_%H%M%S") // 格式化为 YYYYMMDD_HHMMSS
                << ".jpg";
        // 如果请求中指定了路径，使用请求的路径，否则使用生成的文件名
        std::string final_filename = request->file_path.empty() ? filename.str() : request->file_path;

        if (save_image(final_filename)) {
            response->success = true;
            response->message = "Image saved as " + final_filename;
        } else {
            response->success = false;
            response->message = "No image available to save.";
        }
    }


};

int main(int argc, char **argv) {
    // 初始化 ROS 2 节点
    rclcpp::init(argc, argv);

    // 捕捉 Ctrl+C 信号
    signal(SIGINT, signal_handler);

    // 创建 CameraNode 并通过 rclcpp::spin 运行
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
