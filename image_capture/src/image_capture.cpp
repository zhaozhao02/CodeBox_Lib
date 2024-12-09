#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>

using CaptureImage = std_srvs::srv::Trigger;

class ImageCaptureNode : public rclcpp::Node
{
public:
    ImageCaptureNode()
        : Node("image_capture_node")//, cap_(4) // 初始化 VideoCapture，默认打开 /dev/video0
    {
        // 创建服务
        service_ = this->create_service<CaptureImage>(
            "capture_image", 
            std::bind(&ImageCaptureNode::capture_image_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        
        // 创建存储图像的目录
        std::filesystem::create_directory("image_capture");
    }

private:
    rclcpp::Service<CaptureImage>::SharedPtr service_;
    cv::VideoCapture cap_; // VideoCapture 对象

    void capture_image_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request_header; // 标记为未使用
        (void)request;        // 标记为未使用

        // 打开摄像头，如果未在构造函数中打开
        if (!cap_.isOpened()) {
            cap_.open(6); // 0是/dev/video0，确保打开第一个摄像头
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
            response->success = false;
            response->message = "Could not open camera.";
            return;
        }

        cv::Mat frame;
        cap_ >> frame; // 捕获一帧图像

        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture image.");
            response->success = false;
            response->message = "Image capture failed.";
            return;
        }

        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm *now_tm = std::localtime(&now_time);

        // 格式化文件名
        std::ostringstream filename;
       filename << "image_capture/captured_image_"
                 << std::put_time(now_tm, "%Y%m%d_%H%M%S") // 格式化为 YYYYMMDD_HHMMSS
                 << ".jpg";

        // 保存图像
        if (cv::imwrite(filename.str(), frame)) {
            RCLCPP_INFO(this->get_logger(), "Image captured and saved as 'captured_image.jpg'.");
            response->success = true;
            response->message = "Image captured successfully.";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save image.");
            response->success = false;
            response->message = "Image saving failed.";
        }

        // 释放摄像头
        cap_.release();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageCaptureNode>());
    rclcpp::shutdown();
    return 0;
}
