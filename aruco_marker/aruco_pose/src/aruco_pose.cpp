#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <mutex>


#include "fdcl_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose.hpp"





#define IMAGE_SHOW (1)

// 绘制摄像头坐标系
void drawCameraCoordinateAxes(cv::Mat &image, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs) {
    // 定义摄像头坐标系的轴长度
    float axisLength = 0.15f;

    // 定义摄像头坐标系的轴的终点
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(0, 0, 0));
    axisPoints.push_back(cv::Point3f(axisLength, 0, 0));
    axisPoints.push_back(cv::Point3f(0, axisLength, 0));
    axisPoints.push_back(cv::Point3f(0, 0, axisLength));

    // 假设摄像头坐标系的旋转向量和平移向量
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

    // 将摄像头坐标系的轴投影到图像平面
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    // 绘制摄像头坐标系的轴
    cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 10); // X轴 (红色)
    cv::line(image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 10); // Y轴 (绿色)
    cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 10); // Z轴 (蓝色)
}

int main(int argc, char ** argv) {

    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("aruco_pose");

    // 参数设置及解析
    node->declare_parameter<std::string>("calibration_file", "");
    cv::CommandLineParser parser(argc, argv, fdcl::keys);

    const char* about = "Pose estimation of ArUco marker images";
    auto success = parse_inputs(parser, about);
    if (!success) {
        return 1;
    }

    int dictionary_id = parser.get<int>("d");
    float marker_length_m = parser.get<float>("l");

    // 创建TransformBroadcaster
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    auto publisher_pose = node->create_publisher<geometry_msgs::msg::Pose>("/tag_pose", 10);

    // 设置 GStreamer 管道
    /*std::string gst_pipeline = "v4l2src device=/dev/video2 ! "
                               "video/x-h264, width=1280, height=720, framerate=30/1 ! "
                               "h264parse ! avdec_h264 ! videoconvert ! appsink";*/
							   

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

    // std::string gst_pipeline = "v4l2src device=" + device + " ! "
    //                          + "image/jpeg, width=" + std::to_string(width) + ", height=" + std::to_string(height) + ", framerate=" + std::to_string(framerate) + "/1 ! "
    //                          + "jpegdec ! videoconvert ! appsink";

    std::string gst_pipeline = "v4l2src device=" + device + " ! "
                             + "image/jpeg, width=" + std::to_string(width) + ", height=" + std::to_string(height) + ", framerate=" + std::to_string(framerate) + "/1 ! "
                             + "jpegdec ! videoconvert ! video/x-raw,format=GRAY8 ! appsink";

    RCLCPP_INFO(node->get_logger(), "GStreamer pipeline: %s", gst_pipeline.c_str());


    // 使用 GStreamer 管道创建 OpenCV 视频捕获对象
    cv::VideoCapture cap(gst_pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Can't Open Camera ...");
        return -1;
    }

     // 设置二维码检测参数
    cv::Mat image, image_copy;
    cv::Mat cap_image;
    cv::Mat camera_matrix, dist_coeffs;

    std::ostringstream vector_to_marker;

    // 创建字典
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<std::string>("dp"), detectorParams);
        if(!readOk) {
            RCLCPP_ERROR(node->get_logger(), "Invalid detector parameters file");
            // cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }

    std::string calibration_file = node->get_parameter("calibration_file").as_string();
    cv::FileStorage fs(calibration_file.c_str(), cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

    int frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    // 主线程
    while (rclcpp::ok()) {

        cap >> image;

        if (image.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Can't Rev. Video Frames ...");
            break;
        }
        

#if 1
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams);

#if IMAGE_SHOW
        //cv::undistort(image, image_copy, camera_matrix, dist_coeffs);
        image.copyTo(image_copy);
#endif

        if (!ids.empty()) {
#if IMAGE_SHOW
            cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
#endif

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m, camera_matrix, dist_coeffs, rvecs, tvecs);

           
            //RCLCPP_INFO_STREAM(node->get_logger(), "Translation: " << tvecs[0] << "\tRotation: " << rvecs[0]);

            for (size_t i = 0; i < ids.size(); ++i) {
                    
                int index = ids[i];

                // 只识别特定的码
                if ( index != 123 ){
                    break;
                }   
                
#if IMAGE_SHOW
                cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1);
#endif

                // 将旋转向量转换为旋转矩阵
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvecs[i], rotation_matrix);

                // 从旋转矩阵中提取旋转角度
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


                // 使用 roll, pitch, yaw 转换为四元数
                tf2::Quaternion quaternion;
                quaternion.setRPY(roll, pitch, yaw);

                // 发布坐标变换
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


                // 发布相对位姿关系
                auto message = geometry_msgs::msg::Pose();
                message.position.x = x;
                message.position.y = y;
                message.position.z = z;
                message.orientation.x = quaternion.x();
                message.orientation.y = quaternion.y();
                message.orientation.z = quaternion.z();
                message.orientation.w = quaternion.w();
    
                publisher_pose->publish(message);

                // 将旋转角度转换为度 RPY
                rotation_angles[0] = rotation_angles[0] * 180.0 / CV_PI;
                rotation_angles[1] = rotation_angles[1] * 180.0 / CV_PI;
                rotation_angles[2] = rotation_angles[2] * 180.0 / CV_PI;

                //RCLCPP_INFO_STREAM(node->get_logger(), "Translation: " << tvecs[i] << "\tRotation (degrees): " << rotation_angles);
            }
        }



#if IMAGE_SHOW

        // 绘制摄像头坐标系
        drawCameraCoordinateAxes(image_copy, camera_matrix, dist_coeffs);

        // 显示帧
        int original_width = image_copy.cols;
        int original_height = image_copy.rows;

        // 计算新的尺寸，长宽为原来的一半
        int new_width = original_width / 2;
        int new_height = original_height / 2;

        // 调整图像尺寸
        cv::Mat resized_image;
        cv::resize(image_copy, resized_image, cv::Size(new_width, new_height));

        imshow("Pose estimation", resized_image);
        char key = (char)cv::waitKey(1);
        if (key == 27) {
            break;
        }
#endif

#endif
        // 计算并显示 FPS
        frame_count++;
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;

        if (elapsed.count() >= 1.0) { // 每秒更新一次
            double fps = frame_count / elapsed.count();
            RCLCPP_INFO_STREAM(node->get_logger(), "FPS: " << fps );
            frame_count = 0;
            start_time = end_time;
        }

        usleep(1000);

    }

    // 释放视频捕获对象并关闭所有OpenCV窗口
    cap.release();

#if IMAGE_SHOW
    cv::destroyAllWindows();
#endif

    rclcpp::shutdown();
    return 0;
}
