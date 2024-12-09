#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <mutex>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <map>
#include <string>

#include "fdcl_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/highgui/highgui.hpp>

#include "aruco_msgs/msg/aruco_pose.hpp"


// 不同码的标定参数
struct CalibrationData {
    std::string calibration_file;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    double marker_length_m;
};

// 全局变量
cv::Mat g_image;                        //图像本体 OpenCV图像格式
std::mutex g_image_mutex;               
bool g_recv_run_flag = false;           //接收到图像的标志位
unsigned int g_recv_cnt = 0;            //接收到的图像数量计数
std::map<int, CalibrationData> g_id_calibration_data;


// 读取配置文件
void read_config_file(const std::string file, int & dictionary_id, std::string & dp){

    YAML::Node config = YAML::LoadFile(file.c_str());

    // 解析YAML文件中的数据
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
        std::string key = it->first.as<std::string>();

        if (key == "Dictionary" || key == "Detector_Params") {
            // 检测参数
            if ( key == "Detector_Params" ){
                dp = it->second.as<std::string>();
            }

            // 字典类型
            if ( key == "Dictionary" ){
                dictionary_id = it->second.as<int>();
            }

        }  else {
            // 处理ID配置项
            int id = std::stoi(key); // 将key转换为int
            YAML::Node id_node = it->second;
            CalibrationData data;
            data.calibration_file = id_node["calibration_file"].as<std::string>();
            data.marker_length_m = id_node["marker_length_m"].as<double>();

            // 读取校准文件中的相机矩阵和畸变系数
            cv::FileStorage fs(data.calibration_file.c_str(), cv::FileStorage::READ);
            if (!fs.isOpened()) {
                std::cerr << "Failed to open calibration file: " << data.calibration_file << std::endl;
                continue;
            }
            fs["camera_matrix"] >> data.camera_matrix;
            fs["distortion_coefficients"] >> data.dist_coeffs;

            g_id_calibration_data[id] = data;
        }
    }

    return ;
}


// 绘制摄像头坐标系
void drawCameraCoordinateAxes(cv::Mat &image, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs) {
    float axisLength = 0.15f;

    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(0, 0, 0));
    axisPoints.push_back(cv::Point3f(axisLength, 0, 0));
    axisPoints.push_back(cv::Point3f(0, axisLength, 0));
    axisPoints.push_back(cv::Point3f(0, 0, axisLength));

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 10); // X轴 (红色)
    cv::line(image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 10); // Y轴 (绿色)
    cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 10); // Z轴 (蓝色)
}

// 接受图像数据
void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
        std::lock_guard<std::mutex> lock(g_image_mutex);//来获得一个互斥锁 g_image_mutex，确保对共享资源 g_image 的访问是线程安全的。这样可以防止多个线程同时访问或修改 g_image，避免竞争条件和数据不一致的问题。
        g_image = cv_bridge::toCvShare(msg, "bgr8")->image;//将 ROS 图像消息 (msg) 转换为 OpenCV 图像格式 bgr8为图像格式
        g_recv_run_flag = true;
        g_recv_cnt ++;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("aruco_detector_mul");

    image_transport::ImageTransport it(node);
    image_transport::Subscriber image_sub = it.subscribe("camera/image", 1, imageCallback);//订阅话题 指明回调函数


    auto publisher_aruco_pose = node->create_publisher<aruco_msgs::msg::ArucoPose>("/aruco_pose", 10);


    // 读取参数
    node->declare_parameter<int>("detect_hz", 5);
    node->declare_parameter<bool>("show_pic", false);
    node->declare_parameter<std::string>("config_file", "");


    bool show_pic_f = node->get_parameter("show_pic").as_bool();
    int hz = node->get_parameter("detect_hz").as_int();
    if ( hz <= 0 ){
        hz = 5;
    }

    // 读取配置文件
    std::string config_file = node->get_parameter("config_file").as_string();
    RCLCPP_INFO_STREAM(node->get_logger(), " Config_File : " << config_file);

    int dictionary_id = 16;
    std::string dp_file ;
    read_config_file(config_file, dictionary_id, dp_file);      


    // 打印解析后的数据
    // for (const auto& pair : g_id_calibration_data) {

    //     RCLCPP_INFO_STREAM(node->get_logger(), "ID: " << pair.first
    //               << ", Calibration File: " << pair.second.calibration_file
    //               << ", marker_length_m: " << pair.second.marker_length_m
    //               << "Camera Matrix: " << pair.second.camera_matrix 
    //               << "Distortion Coefficients: " << pair.second.dist_coeffs);
    // }


    // RCLCPP_INFO_STREAM(node->get_logger(), "DP FILE : " << dp_file);


    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));       //初始化字典 适用的规则是16 DICT_ARUCO_ORIGINAL
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();//创建 DetectorParameters 对象，用于设置 ArUco 标记检测的各种参数（如阈值、检测精度等）
    bool readOk = readDetectorParameters(dp_file, detectorParams);
    if(!readOk) {
        RCLCPP_ERROR(node->get_logger(), "Invalid detector parameters file");
        return 0;
    }


    cv::Mat image_copy;                 //被复制的一份图像
    int frame_count = 0;
    unsigned int curr_cnt = 0;


    auto start_time = std::chrono::high_resolution_clock::now();
    rclcpp::Rate loop(hz);
    while (rclcpp::ok()) 
    {
        {
            if ( !g_recv_run_flag ){
                loop.sleep();
                rclcpp::spin_some(node);
                continue;
            }

            if ( curr_cnt == g_recv_cnt ){      
                loop.sleep();
                rclcpp::spin_some(node);
                continue;
            }else{
                curr_cnt = g_recv_cnt;
            }
            

            std::lock_guard<std::mutex> lock(g_image_mutex);        //图像为空的处理
            if (g_image.empty()) {
                RCLCPP_ERROR(node->get_logger(), "Video Frame Error ...");
                break;
            }

            std::vector<int> ids;                       //ids中存储了所有的被识别到的ArUco码
            std::vector<std::vector<cv::Point2f>> corners;          //对应的坐标
            cv::aruco::detectMarkers(g_image, dictionary, corners, ids, detectorParams);//进行ArUco检测 

            if ( show_pic_f ){
                 g_image.copyTo(image_copy);            //图像数据拷贝到新的位置 用于显示
            }

            // 开始判断每个码位姿情况
            if (!ids.empty()) {
                if ( show_pic_f ){
                    cv::aruco::drawDetectedMarkers(image_copy, corners, ids,cv::Scalar(0,0,255));//绘制检测到的ArUco标记
                }

                // 对检测到的码分别进行位姿估计
                for (size_t i = 0; i < ids.size(); ++i) {
                    int id_ = ids[i];
                    std::vector<std::vector<cv::Point2f>> corners_;
                    corners_.push_back(corners[i]);

                    if ( g_id_calibration_data.find(id_) == g_id_calibration_data.end() ){//判断是否为特殊标记 121 123 等在config文件中提到的 不是的话就continue
                        continue;                           //这个end意味着find返回的是指向map末尾后的迭代器，意味着没有找到id_这个元素
                    }

                    std::vector<cv::Vec3d> rvecs, tvecs;
                    cv::aruco::estimatePoseSingleMarkers(corners_, g_id_calibration_data[id_].marker_length_m, 
                                                                  g_id_calibration_data[id_].camera_matrix, 
                                                                  g_id_calibration_data[id_].dist_coeffs, 
                                                                  rvecs, 
                                                                  tvecs);

                    if ( show_pic_f ){
                        cv::aruco::drawAxis(image_copy,                         //绘制坐标系
                                             g_id_calibration_data[id_].camera_matrix, 
                                             g_id_calibration_data[id_].dist_coeffs, 
                                             rvecs, 
                                             tvecs, 0.1);
                    }

                    // 变换位姿
                    cv::Mat rotation_matrix;
                    cv::Rodrigues(rvecs[0], rotation_matrix);

                    cv::Vec3d rotation_angles;
                    rotation_angles[0] = std::atan2(rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
                    rotation_angles[1] = std::atan2(-rotation_matrix.at<double>(2, 0), std::sqrt(rotation_matrix.at<double>(2, 1) * rotation_matrix.at<double>(2, 1) + rotation_matrix.at<double>(2, 2) * rotation_matrix.at<double>(2, 2)));
                    rotation_angles[2] = std::atan2(rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0));

                    double x = tvecs[0][0];
                    double y = tvecs[0][1];
                    double z = tvecs[0][2];

                    double roll = rotation_angles[0];
                    double pitch = rotation_angles[1];
                    double yaw = rotation_angles[2];

                    tf2::Quaternion quaternion;
                    quaternion.setRPY(roll, pitch, yaw);

                    aruco_msgs::msg::ArucoPose msg;
                    
                    msg.mark_id = id_;
                    msg.px = x;
                    msg.py = y;
                    msg.pz = z;
                    msg.ox = quaternion.x();
                    msg.oy = quaternion.y();
                    msg.oz = quaternion.z();
                    msg.ow = quaternion.w();

                    publisher_aruco_pose->publish(msg);//发布位姿数据 话题为:aruco_pose

                }
            }
        }

        if ( show_pic_f ){
            
            int original_width = image_copy.cols;
            int original_height = image_copy.rows;
            int new_width = original_width / 2;
            int new_height = original_height / 2;
            cv::Mat resized_image;
            cv::resize(image_copy, resized_image, cv::Size(new_width, new_height));//缩小图片尺寸 用于显示

            imshow("Pose estimation", resized_image);   //窗口显示图像 
            char key = (char)cv::waitKey(1);            //Esc键退出  
            if (key == 27) {
                break;
            }
        }
        
        frame_count++;
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;

        if (elapsed.count() >= 3.0) {
            double fps = frame_count / elapsed.count();
            RCLCPP_INFO_STREAM(node->get_logger(), "FPS: " << fps );
            frame_count = 0;
            start_time = end_time;
        }

        loop.sleep();
        rclcpp::spin_some(node);
    }

    if ( show_pic_f ){
        cv::destroyAllWindows();//销毁cv相关的窗口并释放资源
    }
    
    rclcpp::shutdown();
    return 0;
}
