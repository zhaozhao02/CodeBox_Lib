#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <mutex>


#include "fdcl_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "geometry_msgs/msg/pose.hpp"



int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("usb_cam");

    // 创建图像传输发布器
    image_transport::ImageTransport it(node);
    auto image_pub = it.advertise("camera/image", 1);

    // 设置 GStreamer 管道
    node->declare_parameter<std::string>("device", "/dev/video0");
    node->declare_parameter<int>("width", 1920);
    node->declare_parameter<int>("height", 1080);
    node->declare_parameter<int>("framerate", 30);

    std::string device;
    int width, height, framerate;
    node->get_parameter("device", device);
    node->get_parameter("width", width);
    node->get_parameter("height", height);
    node->get_parameter("framerate", framerate);

    std::string gst_pipeline = "v4l2src device=" + device + " ! "
                             + "image/jpeg, width=" + std::to_string(width) + ", height=" + std::to_string(height) + ", framerate=" + std::to_string(framerate) + "/1 ! "
                             + "jpegdec ! videoconvert ! video/x-raw,format=GRAY8 ! appsink";
    // std::string gst_pipeline = "v4l2src device=" + device + " ! "
    //                         + "image/jpeg, width=" + std::to_string(width) + ", height=" + std::to_string(height) + ", framerate=" + std::to_string(framerate) + "/1 ! "
    //                         + "jpegdec ! nvvidconv ! video/x-raw(memory:NVMM), format=NV12 ! "
    //                         + "nvv4l2h264enc preset-level=2 insert-sps-pps=true maxperf-enable=1 ! "
    //                         + "appsink";
    RCLCPP_INFO(node->get_logger(), "GStreamer pipeline: %s", gst_pipeline.c_str());

    // 使用 GStreamer 管道创建 OpenCV 视频捕获对象
    cv::VideoCapture cap(gst_pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Can't Open Camera ...");
        return -1;
    }

    cv::Mat image;
    int frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    // 主线程
    while (rclcpp::ok()) {

        cap >> image;

        if (image.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Can't Rev. Video Frames ...");
            break;
        }

        // 计算并显示 FPS
        frame_count++;
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;

        if (elapsed.count() >= 3.0) { // 每秒更新一次
            double fps = frame_count / elapsed.count();
            //RCLCPP_INFO_STREAM(node->get_logger(), "FPS: " << fps );
            frame_count = 0;
            start_time = end_time;
        }

        // 将图像转换为 ROS 2 消息并发布
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image).toImageMsg();
        image_pub.publish(msg);
    }

    // 释放视频捕获对象并关闭所有OpenCV窗口
    cap.release();

    rclcpp::shutdown();
    return 0;
}