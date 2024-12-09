#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <mutex>
#include <cv_bridge/cv_bridge.h>

#include "fdcl_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/highgui/highgui.hpp>

#define IMAGE_SHOW (0)

// 全局变量
cv::Mat g_image;
std::mutex g_image_mutex;
bool g_recv_run_flag = false;
unsigned int g_recv_cnt = 0;

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
        std::lock_guard<std::mutex> lock(g_image_mutex);
        g_image = cv_bridge::toCvShare(msg, "bgr8")->image;
        g_recv_run_flag = true;
        g_recv_cnt ++;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("aruco_detector");

    image_transport::ImageTransport it(node);
    image_transport::Subscriber image_sub = it.subscribe("camera/image", 1, imageCallback);

    node->declare_parameter<std::string>("calibration_file", "");
    node->declare_parameter<int>("detect_hz", 5);
    node->declare_parameter<bool>("show_pic", false);

    cv::CommandLineParser parser(argc, argv, fdcl::keys);

    const char* about = "Pose estimation of ArUco marker images";
    auto success = parse_inputs(parser, about);
    if (!success) {
        return 1;
    }

    int dictionary_id = parser.get<int>("d");
    float marker_length_m = parser.get<float>("l");

    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    auto publisher_pose = node->create_publisher<geometry_msgs::msg::Pose>("/tag_pose", 10);

    cv::Mat image_copy;
    cv::Mat camera_matrix, dist_coeffs;

    std::ostringstream vector_to_marker;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<std::string>("dp"), detectorParams);
        if(!readOk) {
            RCLCPP_ERROR(node->get_logger(), "Invalid detector parameters file");
            return 0;
        }
    }

    std::string calibration_file = node->get_parameter("calibration_file").as_string();
    cv::FileStorage fs(calibration_file.c_str(), cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

    int frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    unsigned int curr_cnt = 0;


    bool show_pic_f = node->get_parameter("show_pic").as_bool();


    int hz = node->get_parameter("detect_hz").as_int();
    if ( hz <= 0 ){
        hz = 5;
    }

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
            

            std::lock_guard<std::mutex> lock(g_image_mutex);
            if (g_image.empty()) {
                RCLCPP_ERROR(node->get_logger(), "Video Frame Error ...");
                break;
            }

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(g_image, dictionary, corners, ids, detectorParams);

            if ( show_pic_f ){
                 g_image.copyTo(image_copy);
            }

            if (!ids.empty()) {
                if ( show_pic_f ){
                     cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
                }
               
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m, camera_matrix, dist_coeffs, rvecs, tvecs);

                for (size_t i = 0; i < ids.size(); ++i) {
                    int index = ids[i];
                    if (index != 123) {
                        continue;
                    }
                    
                    if ( show_pic_f ){
                         cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1);
                    }

                    cv::Mat rotation_matrix;
                    cv::Rodrigues(rvecs[i], rotation_matrix);

                    cv::Vec3d rotation_angles;
                    rotation_angles[0] = std::atan2(rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
                    rotation_angles[1] = std::atan2(-rotation_matrix.at<double>(2, 0), std::sqrt(rotation_matrix.at<double>(2, 1) * rotation_matrix.at<double>(2, 1) + rotation_matrix.at<double>(2, 2) * rotation_matrix.at<double>(2, 2)));
                    rotation_angles[2] = std::atan2(rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0));

                    double x = tvecs[i][0];
                    double y = tvecs[i][1];
                    double z = tvecs[i][2];

                    double roll = rotation_angles[0];
                    double pitch = rotation_angles[1];
                    double yaw = rotation_angles[2];

                    tf2::Quaternion quaternion;
                    quaternion.setRPY(roll, pitch, yaw);

                    geometry_msgs::msg::TransformStamped transform_stamped;
                    transform_stamped.header.stamp = node->now();
                    transform_stamped.header.frame_id = "marker_frame";
                    transform_stamped.child_frame_id = "camera_frame";

                    transform_stamped.transform.translation.x = x;
                    transform_stamped.transform.translation.y = y;
                    transform_stamped.transform.translation.z = z;
                    transform_stamped.transform.rotation.x = quaternion.x();
                    transform_stamped.transform.rotation.y = quaternion.y();
                    transform_stamped.transform.rotation.z = quaternion.z();
                    transform_stamped.transform.rotation.w = quaternion.w();

                    tf_broadcaster->sendTransform(transform_stamped);

                    auto message = geometry_msgs::msg::Pose();
                    message.position.x = x;
                    message.position.y = y;
                    message.position.z = z;
                    message.orientation.x = quaternion.x();
                    message.orientation.y = quaternion.y();
                    message.orientation.z = quaternion.z();
                    message.orientation.w = quaternion.w();

                    publisher_pose->publish(message);

                    rotation_angles[0] = rotation_angles[0] * 180.0 / CV_PI;
                    rotation_angles[1] = rotation_angles[1] * 180.0 / CV_PI;
                    rotation_angles[2] = rotation_angles[2] * 180.0 / CV_PI;
                }
            }
        }

        if ( show_pic_f ){
            drawCameraCoordinateAxes(image_copy, camera_matrix, dist_coeffs);

            int original_width = image_copy.cols;
            int original_height = image_copy.rows;
            int new_width = original_width / 2;
            int new_height = original_height / 2;
            cv::Mat resized_image;
            cv::resize(image_copy, resized_image, cv::Size(new_width, new_height));

            imshow("Pose estimation", resized_image);
            char key = (char)cv::waitKey(1);
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
        cv::destroyAllWindows();
    }
    
    rclcpp::shutdown();
    return 0;
}
