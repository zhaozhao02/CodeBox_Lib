#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cstdlib>
#include <chrono>

#include "fdcl_common.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("aruco_pose");

    node->declare_parameter<std::string>("calibration_file", "");

    cv::CommandLineParser parser(argc, argv, fdcl::keys);

    const char* about = "Pose estimation of ArUco marker images";

    auto success = parse_inputs(parser, about);
    if (!success) {
        return 1;
    }

    cv::VideoCapture in_video;
    success = parse_video_in(in_video, parser);
    if (!success) {
        return 1;
    }

    // 设置输出格式 
    in_video.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    // 读取当前设置的格式
    int fourcc = static_cast<int>(in_video.get(cv::CAP_PROP_FOURCC));
    char fourcc_chars[] = {
        static_cast<char>(fourcc & 0xFF),
        static_cast<char>((fourcc >> 8) & 0xFF),
        static_cast<char>((fourcc >> 16) & 0xFF),
        static_cast<char>((fourcc >> 24) & 0xFF),
        '\0'
    };
    RCLCPP_INFO(node->get_logger(), "Current FOURCC: %s", fourcc_chars);

    int dictionary_id = parser.get<int>("d");
    float marker_length_m = parser.get<float>("l");
    int wait_time = 10;

    if (marker_length_m <= 0) {
        // std::cerr << "Marker length must be a positive value in meter\n";
        RCLCPP_ERROR(node->get_logger(), "Marker length must be a positive value in meter\n");
        return 1;
    }

    cv::Mat image, image_copy;
    cv::Mat camera_matrix, dist_coeffs;

    std::ostringstream vector_to_marker;

    // Create the dictionary from the same dictionary the marker was generated.
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));


    std::string calibration_file = node->get_parameter("calibration_file").as_string();

    cv::FileStorage fs(calibration_file.c_str(), cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;


    while (rclcpp::ok())
    {

        auto start = std::chrono::high_resolution_clock::now();

        if ( !in_video.grab() ){
            break;
        }

        in_video.retrieve(image);
        // image.copyTo(image_copy);

        // std::vector<int> ids;
        // std::vector<std::vector<cv::Point2f> > corners;
        // cv::aruco::detectMarkers(image, dictionary, corners, ids);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        double fps = 1.0 / elapsed.count();
        RCLCPP_INFO_STREAM(node->get_logger(), "FPS : " << fps);

        // // if at least one marker detected
        // if (ids.size() > 0)
        // {
        //     cv::aruco::drawDetectedMarkers(image_copy, corners, ids);

        //     std::vector<cv::Vec3d> rvecs, tvecs;
        //     cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
        //             camera_matrix, dist_coeffs, rvecs, tvecs);
                    
        //     // std::cout << "Translation: " << tvecs[0]
        //     //     << "\tRotation: " << rvecs[0] << "\n";
        //     RCLCPP_INFO_STREAM(node->get_logger(), "Translation: " << tvecs[0] << "\tRotation: " << rvecs[0]);
            
        //     // Draw axis for each marker
        //     for(int i=0; i < ids.size(); i++)
        //     {
        //         cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs,
        //                 rvecs[i], tvecs[i], 0.1);

        //         // This section is going to print the data for the first the 
        //         // detected marker. If you have more than a single marker, it is 
        //         // recommended to change the below section so that either you
        //         // only print the data for a specific marker, or you print the
        //         // data for each marker separately.
        //         drawText(image_copy, "x", tvecs[0](0), cv::Point(10, 30));
        //         drawText(image_copy, "y", tvecs[0](1), cv::Point(10, 50));
        //         drawText(image_copy, "z", tvecs[0](2), cv::Point(10, 70));
        //         drawText(image_copy, "fps", fps, cv::Point(10, 90));
        //     }
        // }

        //  // 获取原始图像的尺寸
        // int original_width = image_copy.cols;
        // int original_height = image_copy.rows;

        // // 计算新的尺寸，长宽为原来的一半
        // int new_width = original_width / 2;
        // int new_height = original_height / 2;

        // // 调整图像尺寸
        // cv::Mat resized_image;
        // cv::resize(image_copy, resized_image, cv::Size(new_width, new_height));

        // imshow("Pose estimation", resized_image);
        // char key = (char)cv::waitKey(wait_time);
        // if (key == 27) {
        //     break;
        // }
    }

    in_video.release();

    return 0;
}
