#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cstdlib>
#include <chrono>
//#include <Eigen/Eigen>
//#include <Eigen/Dense>
#include <thread>
#include <chrono>
#include "std_msgs/msg/float32.hpp"
#include "fdcl_common.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/status_text.hpp"
#include "aruco_msgs/msg/aruco_pose.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "camera/srv/save_image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


#define NORM_FLY_HEIHT   (norm_fly_height)  //起飞高度设定为2m
#define OVER_TIME_S      (over_time_s)      //二维码超时 时间
#define FALL_DOWN_VEL    (Fall_speed)       //下降速度

#define SWITCH_FLYMODE_CHANEL  (5)          // 飞行模式切换通道   
#define SWITCH_OFFBOARD_CHANEL (9)          // 板外切换通道      左侧C按钮  


#define MAX_VEL_ABS     (max_vel_abs)       //单轴最大速度 宏 1m/s
#define MODEL_TAKE_OFF  (0)
#define MODEL_TRACKING  (1)
#define MODEL_LANDING   (2)
#define MODEL_UNLOCK    (3)

class tracking_node : public rclcpp::Node
{
    public:
        tracking_node(std::string name);
        ~tracking_node();

        // MAV里程计消息接收回调函数
        void mavpose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        // 订阅TagPose
        void tagpose_callback(const aruco_msgs::msg::ArucoPose::SharedPtr msg);

        // 订阅无人机通道
        void rc_callback(const mavros_msgs::msg::RCIn::SharedPtr msg);

        // 订阅egp_planer
        void way_point_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        // 向飞行器发送控制指令
        void uav_speed_control(const double vx, const  double cy, const double cz, const double yaw_rate);
        void uav_speed_control_without_yaw(const double vx, const  double vy, const double vz);

        // 控制器线程
        void controller_task(void);

        // 等待线程结束
        void wait_for_all_tasks_finished(void);

        // 二维码是否收到
        bool recv_new_qrcode_info(void);

        // 速度限制
        double limit_vel(double x);

        // 采点函数 
        void addWaypoint(const tf2::Vector3& position, double yaw, bool camera);

        // 获取目标点函数
        bool getNextWaypoint(tf2::Vector3& position, double& yaw, bool& camera);

        // 输出所有点
        void printAllWaypoints() const;

        // 向qgc发送调试信息
        void messages_send(uint8_t severity, const std::string& text);

    private:
        
        // 订阅器
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mavpose_sub_;
        rclcpp::Subscription<aruco_msgs::msg::ArucoPose>::SharedPtr tagpose_sub_;
        rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr way_point_sub_;  // 订阅ego_planer发出的点信息
        // 发布器
        rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr uav_target_pub_; 
        rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr status_text_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo_pwm_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr way_point_publisher_;
   
        //客户端
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
        rclcpp::Client<camera::srv::SaveImage>::SharedPtr camera_client_;

        // 无人机里程计位姿
        tf2::Vector3 mav_p;              //三轴位置
        tf2::Quaternion mav_q;           //姿态四元数
        double mav_yaw=0.0;
        double mav_pitch=0.0;
        double mav_roll=0.0;
        // tag码在世界坐标系中的位置
        tf2::Vector3 tag_position_w;     
        // tag码在无人机坐标系中的位置
        tf2::Vector3 tag_position_u;
        // 降落点的坐标
        tf2::Vector3 land_position;
        // 定点队列 存储 Vector3 和 double
        std::queue<std::tuple<tf2::Vector3, double, bool>> waypoint_queue;
        // 保证里程计信息必须先获得
        bool gotodom_flag = false;
        // 控制线程句柄
        std::thread controller_handler_;    
        // 目标Yaw角度
        double target_yaw = 0;  
        // 目标点坐标
        tf2::Vector3 target_position;   
        // 目标点拍照标志变量
        bool camera; 
        // 线程信号
        bool stop_requested_ = false;
        // 二维码信号计数器
        bool qrcode_cnt_start_ = false;
        unsigned int qrcode_cnt_ = 0;
        // MODEL飞行模式  0-空闲  1-跟踪 2-降落
        int fly_model_ = MODEL_UNLOCK;
        //标定起飞时的坐标
        float take_off_x=0.0;
        float take_off_y=0.0;
        float take_off_z=0.0;
        //起飞状态 解锁电机用
        bool take_off_state=false;    
        //相机状态 开关摄像头用
        bool usb_cam_state=false;
        bool usb_cam_running = false;
        //循迹状态 
        bool tracking_state=false;                          
        // 控制器PID参数
        double P=0.0;
        double I=0.0; 
        double D=0.0; 
        double Sum_err=0.0;  
        double Fall_speed=0.0;   
        double yaw_p=0.0;
        double camera_time=0.0;
        double over_time_s=0.0;
        double max_vel_abs=0.0;
        double norm_fly_height=0.0;
        double set_point_x=0.0;
        double set_point_y=0.0;
        double tracking_mode=0.0;
        double tracking_radius=0.0;
        double tracking_angle=10.0;
};

// 添加一个点到队列的函数
void tracking_node::addWaypoint(const tf2::Vector3& position, double yaw, bool camera) {
    waypoint_queue.push(std::make_tuple(position, yaw, camera));
}

// 获取并移除队列中的下一个点
bool tracking_node::getNextWaypoint(tf2::Vector3& position, double& yaw, bool& camera) {
    if (!waypoint_queue.empty()) {
        std::tie(position, yaw, camera) = waypoint_queue.front(); // 获取队首点
        waypoint_queue.pop(); // 从队列中移除该点
        return true;
    }
    return false;
}

// 输出队列中所有点的函数
void tracking_node::printAllWaypoints() const {
    // 创建队列的副本来遍历，以免修改原始队列
    std::queue<std::tuple<tf2::Vector3, double, bool>> temp_queue = waypoint_queue;
    RCLCPP_INFO(this->get_logger(), "Printing all waypoints in the queue:");
    while (!temp_queue.empty()) {
        tf2::Vector3 position;
        double yaw;
        bool camera;
        // 获取队首点并打印
        std::tie(position, yaw, camera) = temp_queue.front();
        RCLCPP_INFO_STREAM(this->get_logger(), "Position: (" << position.x() << ", " << position.y() << ", " << position.z() << "), Yaw: " << yaw << ", camera:" << camera);
        // 移除队首点
        temp_queue.pop();
    }
}

// 向qgc发送信息
void tracking_node::messages_send(uint8_t severity, const std::string& text)
{   
    mavros_msgs::msg::StatusText message_qgc;
    message_qgc.severity = severity; // severity 可以是 INFO, WARNING, ERROR 等
    message_qgc.text = text; // 自定义文本消息
    status_text_publisher_->publish(message_qgc);
}

// 构造函数
tracking_node::tracking_node(std::string name) : Node(name)
{
    // 订阅无人机里程计信息	
    #define USE_MAVROS_GPS
    #ifdef  USE_MAVROS_GPS
    mavpose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(          //订阅飞控的位姿和速度信息
        //"/mavros/local_position/odom",
        "/mavros/odometry/out",
        rclcpp::SensorDataQoS(),
        std::bind(&tracking_node::mavpose_callback, this, std::placeholders::_1)
    );
    #else
    mavpose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/out/odom",
        10,
        std::bind(&tracking_node::mavpose_callback, this, std::placeholders::_1)
    );
    #endif


    // 订阅二维码识别情况
    tagpose_sub_ = this->create_subscription<aruco_msgs::msg::ArucoPose>(
        "/aruco_pose",
        100,
        std::bind(&tracking_node::tagpose_callback, this, std::placeholders::_1)
    );

    // 订阅无人机通道状态
    rc_sub_ = this->create_subscription<mavros_msgs::msg::RCIn>(
        "/mavros/rc/in",
        100,
        std::bind(&tracking_node::rc_callback, this, std::placeholders::_1)
    );

    // 订阅ego_planer发布的目标点信息
    way_point_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ego_output", 
        10,
        std::bind(&tracking_node::way_point_callback, this, std::placeholders::_1)
    );

    // 锁定的服务
    arming_client_=this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    // 控制相机拍照的服务
    camera_client_ = this->create_client<camera::srv::SaveImage>("save_image");
    // 发布无人机控制指令
    uav_target_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
    // QGC的log发布
    status_text_publisher_ = this->create_publisher<mavros_msgs::msg::StatusText>("/mavros/statustext/send", 10);
    // 舵机pwm发布
    servo_pwm_publisher_ = this->create_publisher<std_msgs::msg::Float32>("pwm_duty", 10);
    // 采集到的点发布
    way_point_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);


    // 读取外部参数
    this->declare_parameter<double>("P", 1.5);
    this->declare_parameter<double>("I", 0.0);
    this->declare_parameter<double>("D", 4.5);
    this->declare_parameter<double>("Sum_err", 0.05);
    this->declare_parameter<double>("Fall_speed", 0.25);
    this->declare_parameter<double>("yaw_p", 1.0);   
    this->declare_parameter<double>("camera_time", 5.0);   
    this->declare_parameter<double>("over_time_s", 5.0);   
    this->declare_parameter<double>("max_vel_abs", 0.5);  
    this->declare_parameter<double>("norm_fly_height", 1.5);  
    this->declare_parameter<double>("set_point_x", 3.0);  
    this->declare_parameter<double>("set_point_y", 0.0);
    this->declare_parameter<double>("tracking_mode", 0.0);
    this->declare_parameter<double>("tracking_radius", 0.10);
    this->declare_parameter<double>("tracking_angle", 10.0);

    P = this->get_parameter("P").get_value<double>();
    I = this->get_parameter("I").get_value<double>();
    D = this->get_parameter("D").get_value<double>();
    Sum_err = this->get_parameter("Sum_err").get_value<double>();
    Fall_speed = this->get_parameter("Fall_speed").get_value<double>();
    yaw_p = this->get_parameter("yaw_p").get_value<double>();
    camera_time = this->get_parameter("camera_time").get_value<double>();
    over_time_s = this->get_parameter("over_time_s").get_value<double>();
    max_vel_abs = this->get_parameter("max_vel_abs").get_value<double>();
    norm_fly_height = this->get_parameter("norm_fly_height").get_value<double>();
    set_point_x = this->get_parameter("set_point_x").get_value<double>();
    set_point_y = this->get_parameter("set_point_y").get_value<double>();
    tracking_mode = this->get_parameter("tracking_mode").get_value<double>();
    tracking_radius = this->get_parameter("tracking_radius").get_value<double>();
    tracking_angle = this->get_parameter("tracking_angle").get_value<double>();

    RCLCPP_INFO_STREAM(this->get_logger(), 
    "PID Param P: " << P << " I: " << I << " D: " << D 
    << " Sum_err: " << Sum_err 
    << " Fall_speed: " << Fall_speed 
    << " yaw_p: " << yaw_p 
    << " camera_time: " << camera_time 
    << " over_time_s: " << over_time_s 
    << " max_vel_abs: " << max_vel_abs 
    << " norm_fly_height: " << norm_fly_height
    << " set_point_x: " << set_point_x 
    << " set_point_y: " << set_point_y
    << " tracking_mode: " << tracking_mode
    << " tracking_radius: " << tracking_radius
    << " tracking_angle: " << tracking_angle
    );
    
    // 控制器句柄
    controller_handler_ =  std::thread(&tracking_node::controller_task, this);

}

// 析构函数
tracking_node::~tracking_node()
{
    wait_for_all_tasks_finished();
}

// 等待线程结束
void tracking_node::wait_for_all_tasks_finished(void)
{
     // 线程停止
    stop_requested_ = true;

    // 等待线程结束
    if (controller_handler_.joinable())
    {
        controller_handler_.join();
    }
}

// 速度限制
double tracking_node::limit_vel(double x)
{
    if ( x > std::abs(MAX_VEL_ABS) ){
        x = std::abs(MAX_VEL_ABS);
    }
    if ( x < -1.0 * std::abs(MAX_VEL_ABS) ){
        x = -1.0 * std::abs(MAX_VEL_ABS);
    }
    return x;
}


// 向飞行器发送控制指令
void tracking_node::uav_speed_control(const double vx, const  double vy, const double vz, const double yaw_rate)
{
    mavros_msgs::msg::PositionTarget msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";           //参考坐标系为map
    msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;       //忽略位置、加速度信息 让PX4飞控自己处理
    msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX +
                    mavros_msgs::msg::PositionTarget::IGNORE_PY +
                    mavros_msgs::msg::PositionTarget::IGNORE_PZ +
                    mavros_msgs::msg::PositionTarget::IGNORE_AFX +
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY +
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ +
                    mavros_msgs::msg::PositionTarget::FORCE +                       //使用力控制
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW;       
                    //mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

    msg.velocity.x = vx;                //改变飞机的速度
    msg.velocity.y = vy;
    msg.velocity.z = vz;
    msg.yaw_rate = yaw_rate;                      //改变飞机的偏航角
    
    uav_target_pub_->publish(msg);      //发布消息

}

// 向飞行器发送控制指令忽略Yaw
void tracking_node::uav_speed_control_without_yaw(const double vx, const  double vy, const double vz)
{
    mavros_msgs::msg::PositionTarget msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";    
    msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX +
                    mavros_msgs::msg::PositionTarget::IGNORE_PY +
                    mavros_msgs::msg::PositionTarget::IGNORE_PZ +
                    mavros_msgs::msg::PositionTarget::IGNORE_AFX +
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY +
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ +
                    mavros_msgs::msg::PositionTarget::FORCE +
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW +
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

    msg.velocity.x = vx;
    msg.velocity.y = vy;
    msg.velocity.z = vz;

    uav_target_pub_->publish(msg);

}

// 控制器线程
void tracking_node::controller_task(void)
{
    double loop_rate_hz = 10;
    double loop_rete_ts = 1.0 / loop_rate_hz;
    double over_cnt = 0;
    auto request_arming = std::make_shared<mavros_msgs::srv::CommandBool::Request>();   // 锁定电机的信息
    auto request_camera = std::make_shared<camera::srv::SaveImage::Request>();          // 控制前置相机拍照的信息
    double elapsed_time = 0.0;                                                          // 累加时间，单位为秒
    auto message = std_msgs::msg::Float32();                                            // 控制pwm和io的话题
    rclcpp::Rate rate(10.0); // 10 Hz
    rclcpp::Rate rate_1hz(1.0); // 1 Hz
    // 检查ODOM是否收到
    RCLCPP_INFO(this->get_logger(), "Waiting for ODOM ...");
    while( rclcpp::ok() && !stop_requested_ && !gotodom_flag ){
        rate.sleep();
    }
    //等待arming服务器
    RCLCPP_INFO(this->get_logger(),"Waiting for arming service ...");
    while(!arming_client_->wait_for_service(std::chrono::seconds(1))){

    }
    // 等待服务可用
    RCLCPP_INFO(this->get_logger(), "Waiting for the camera service to be available...");
    while (!camera_client_->wait_for_service(std::chrono::seconds(1))) {
        messages_send(mavros_msgs::msg::StatusText::WARNING,"Waiting for camera...");
    }

    // 控制循环
    while( rclcpp::ok() && !stop_requested_ ){

     
        // 判断ODOM数据是否有 [ 后续再实现吧 ]
        // ...

        // 未解锁状态
        if ( fly_model_ == MODEL_UNLOCK ){
            RCLCPP_INFO_STREAM(this->get_logger(), "MODEL_UNLOCK " );
            rate_1hz.sleep();
            uav_speed_control_without_yaw(0, 0, 0);
            continue;
        }

        // 空闲模式 (起飞不需要二维码识别)
        // 起飞后五秒钟高度预估
        if ( fly_model_ == MODEL_TAKE_OFF ){

            if(take_off_state==false)
            {
                request_arming->value=true;//true;            //解锁
                auto result_feture = arming_client_->async_send_request(request_arming,
                    [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future){
                        auto response =future.get();
                        if(response->success){
                            RCLCPP_INFO_STREAM(this->get_logger(), "Armed successful, motors start!" );
                            take_off_state=true;
                            messages_send(mavros_msgs::msg::StatusText::INFO,"Armed!");
                        }
                        else{
                            RCLCPP_INFO_STREAM(this->get_logger(), "Armed failed!" );
                        }
                    }
                );             
                take_off_x=mav_p.x();
                take_off_y=mav_p.y();
                take_off_z=mav_p.z();
            }
            double output_vx = 0;
            double output_vy = 0;
            double output_vz = 0;
            double delta_x = take_off_x - mav_p.x();         //mav_p是当前位置 此处使用了X轴
            double delta_y = take_off_y - mav_p.y();         //mav_p是当前位置 此处使用了Y轴
            double delta_h = NORM_FLY_HEIHT - mav_p.z() + take_off_z;     //mav_p是当前位置 相对高度 

            if(delta_h<=0.1)
            {
                fly_model_=MODEL_TRACKING;
                RCLCPP_INFO_STREAM(this->get_logger(), "Takeoff finished! turn to Tracking mode " );
                auto msg = geometry_msgs::msg::PoseStamped();
                // 填充 header 部分
                msg.header.stamp = this->get_clock()->now();  // 当前时间戳
                msg.header.frame_id = "odom";  // 设置坐标系为 "odom"
                // 填充 pose 部分（只设置位置，姿态为空）
                msg.pose.position.x = set_point_x;  // 设置位置的 x 坐标
                msg.pose.position.y = set_point_y;  // 设置位置的 y 坐标
                msg.pose.position.z = 1.0;  // 设置位置的 z 坐标
                // 发布消息
                way_point_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Send Pose: x=%.2f, y=%.2f", msg.pose.position.x, msg.pose.position.y);
            }

            output_vx = delta_x * 1.0;                       //高度低于目标值 上升 P控制
            output_vy = delta_y * 1.0;
            output_vz = delta_h * 0.5;

            // 限位
            output_vx = limit_vel(output_vx);
            output_vy = limit_vel(output_vy);
            output_vz = limit_vel(output_vz);

            RCLCPP_INFO_STREAM(this->get_logger(), "MODEL_TAKE_OFF - Speed : " << output_vx << "," << output_vy << "," << output_vz);

            // 发送给无人机控制指令
            uav_speed_control_without_yaw(output_vx,  output_vy, output_vz);        //控制三轴速度
            rate.sleep();
            continue;
        }
       
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if(fly_model_==MODEL_LANDING)               //降落模式或跟踪模式
        {
            auto node_names = this->get_node_graph_interface()->get_node_names();
            usb_cam_running = false;
            // 检查 usb_cam 是否在运行
            for (const auto & name : node_names) {
                if (name == "/usb_cam") {  // 指定根命名空间的 usb_cam 
                    usb_cam_running = true;
                    RCLCPP_INFO(this->get_logger(), "usb_cam is running.");
                    break;
                }
            }
            if (!usb_cam_running) {
                RCLCPP_WARN(this->get_logger(), "usb_cam is not running.");
            }
            if (take_off_state==true && fly_model_ == MODEL_LANDING && !usb_cam_running && !usb_cam_state){     // 已经起飞 切换到降落模式 且摄像头没打开 
                usb_cam_state=true;
                // message_qgc.severity = mavros_msgs::msg::StatusText::INFO; 
                // message_qgc.text = "close cam_forward!";
                // status_text_publisher_->publish(message_qgc);
                // const std::string script_path_kill = "/home/khadas/roaf3d_ws_loop/kill_usb_cam_forward.sh";          // 关闭前置摄像头
                // std::system(script_path_kill.c_str());  
                messages_send(mavros_msgs::msg::StatusText::INFO,"open cam_downward!");
                qrcode_cnt_start_=false;
                const std::string script_path = "/home/khadas/roaf3d_ws_loop/start_aruco_pose.sh";                   // 开启下视摄像头
                std::thread([this, script_path]() {
                    int result = std::system(script_path.c_str());
                    if (result == 0)
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "Camera_downword open successfully.");
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "Camera_downword open failed.");
                    }
                }).detach(); // 使用 detach() 使线程在后台运行
                rate.sleep();
                uav_speed_control_without_yaw(0,  0, 0 );
                continue;
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (!qrcode_cnt_start_&&fly_model_==MODEL_LANDING){    //降落模式    
            // 从来没有接受到过二维码数据 就原地不动
            rate.sleep();
            uav_speed_control_without_yaw(0,  0, 0 );
            continue;
        }
        // 判断是否有新的二维码数据
        // 三种情况:1.有新的数据 0:2.没有新的数据但是未超时 1 :3.没有新的数据但是已经超时了 2.
        int status_info_rev = 0;
        if ( !recv_new_qrcode_info() ){
            // 尚未接收到新的二维码信息
            over_cnt ++;    
            double curr_over_time_s = over_cnt * loop_rete_ts;
            if ( curr_over_time_s >= OVER_TIME_S ){
                // 已经超时了
                status_info_rev = 2;
            }else{
                status_info_rev = 1;
            }
        }else{
            // 收到了新的数据
            over_cnt = 0;
            status_info_rev = 0;
        }

        // 缓慢降落 如果未识别到二维码是无法降落的 
        if ( fly_model_ == MODEL_LANDING && qrcode_cnt_start_ ){
            double output_vx = 0;
            double output_vy = 0;
            double output_vz = 0;
            double yaw_rate = 0;
            double delta_x = 0;          
            double delta_y = 0;
            double delta_h = mav_p.z()-land_position.z();
            //double delta_h = TEST_HIGHT_ERRO - (-tag_position_u(2));
            if( status_info_rev==0 || status_info_rev==1 )//有二维码以及没有超时
            {
                // 计算水平上的偏差
                delta_x = land_position.x() - mav_p.x();          
                delta_y = land_position.y() - mav_p.y();        
                // PID控制器
                static double delta_x_sum=0.0;
                static double delta_y_sum=0.0;
                static double delta_x_last=0.0;
                static double delta_y_last=0.0;
                // PID 计算
                output_vx = P * delta_x + I * delta_x_sum + D * (delta_x - delta_x_last);
                output_vy = P * delta_y + I * delta_y_sum + D * (delta_y - delta_y_last);
                // 更新积分和限制
                delta_x_sum += delta_x;
                if(delta_x_sum>Sum_err) delta_x_sum=Sum_err;
                if(delta_x_sum<-Sum_err) delta_x_sum=-Sum_err;
                delta_y_sum += delta_y;
                if(delta_y_sum>Sum_err) delta_y_sum=Sum_err;
                if(delta_y_sum<-Sum_err) delta_y_sum=-Sum_err;
                // 更新上一个误差
                delta_x_last = delta_x;
                delta_y_last = delta_y;

                if ( delta_h > 0.1 ) 
                {
                    output_vz = -std::abs(FALL_DOWN_VEL);    //高度不低于目标点才下降
                }
                if(delta_h <= 0.1)      //高度足够低 
                {
                    output_vz=-0.35;             //给个向下的速度 遇到地面就锁定了
                    if(take_off_state==true)
                    {
                        request_arming->value=false;            //锁定电机
                        auto result_feture_ = arming_client_->async_send_request(request_arming,
                            [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future_arming){
                                auto response_arming =future_arming.get();
                                if(response_arming->success){
                                    RCLCPP_INFO_STREAM(this->get_logger(), "Disarm successful, motors stop!" );
                                    take_off_state=false;               //释放起飞状态
                                    qrcode_cnt_start_=false;            //摄像头接受状态关闭
                                    const std::string script_path = "/home/khadas/roaf3d_ws_loop/kill_usb_cam_downward.sh"; // 关闭摄像头
                                    std::system(script_path.c_str());  
                                    usb_cam_state=false;     
                                    messages_send(mavros_msgs::msg::StatusText::INFO,"Disarm!");
                                    messages_send(mavros_msgs::msg::StatusText::DEBUG,"Land: "+std::to_string(land_position.x())+","+std::to_string(land_position.y())+","+std::to_string(land_position.z()));
                                    messages_send(mavros_msgs::msg::StatusText::DEBUG,"Posi: "+std::to_string(mav_p.x())+","+std::to_string(mav_p.y())+","+std::to_string(mav_p.z()));
                                }
                                else{
                                    RCLCPP_INFO_STREAM(this->get_logger(), "Disarm failed!" );
                                }
                            }
                        );
                    }
                    RCLCPP_INFO_STREAM(this->get_logger(), "Finish decline! okokokokokok" );
                }
                else
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "In decline! ----------------" );
                }
            }
            else    //二维码丢失，超时 
            {
                if(mav_p.z()-take_off_z<=1.5)
                {
                    output_vz=0.15;
                }
                else
                {
                    output_vz=0;
                }
                RCLCPP_INFO_STREAM(this->get_logger(), "Aruco over time! ----------------"<< over_time_s);
            }
            // 先旋转，角度误差小于±10°再开始平移和下落
            // 计算角度误差
            double yaw_error = target_yaw - mav_yaw;
            // 保证误差在 [-π, π] 范围内
            if (yaw_error > M_PI) yaw_error -= 2 * M_PI;
            if (yaw_error < -M_PI) yaw_error += 2 * M_PI;
            // 将误差从弧度转换为度数
            double yaw_error_degrees = yaw_error * (180.0 / M_PI);
            // 检查误差绝对值是否大于 10° (这个角度是自己定的
            if (fabs(yaw_error_degrees) > 10.0) {
                output_vx=0;
                output_vy=0;
                output_vz=0;
                RCLCPP_INFO_STREAM(this->get_logger(), "mav_yaw : "<< mav_yaw );
                RCLCPP_INFO_STREAM(this->get_logger(), "tar_yaw : "<< target_yaw );
                RCLCPP_INFO_STREAM(this->get_logger(), "err_yaw : "<< yaw_error_degrees );
                RCLCPP_INFO_STREAM(this->get_logger(), "yaw_err : "<< yaw_error );
                RCLCPP_INFO_STREAM(this->get_logger(), "yaw_err too much! " );
            } 
            yaw_rate = yaw_p * yaw_error;               //计算角速度
            if (yaw_rate > 1.0) yaw_rate = 1.0;         //角速度限幅
            if (yaw_rate < -1.0) yaw_rate = -1.0;
            //先平移，再下落 大码期间限定0.3m圆 小码0.2m圆
            if(delta_h>0.6 && sqrt( delta_x*delta_x + delta_y*delta_y ) > 0.3)
            {   
                output_vz=0;
                RCLCPP_INFO_STREAM(this->get_logger(), "distance too much! "<< sqrt( delta_x*delta_x + delta_y*delta_y ) );
            }
            if(delta_h<=0.6 && sqrt( delta_x*delta_x + delta_y*delta_y ) > 0.2)
            {   
                output_vz=0;
                RCLCPP_INFO_STREAM(this->get_logger(), "distance too much! "<< sqrt( delta_x*delta_x + delta_y*delta_y ) );
            }
            // 限位
            output_vx = limit_vel(output_vx);
            output_vy = limit_vel(output_vy);
            output_vz = limit_vel(output_vz);
            RCLCPP_INFO_STREAM(this->get_logger(), "MODEL_LANDING - Speed : " << output_vx << "," << output_vy << "," << output_vz);  
            RCLCPP_INFO_STREAM(this->get_logger(), "MODEL_LANDING - Local : " << mav_p.x() << "," << mav_p.y() << "," << mav_p.z());    
            RCLCPP_INFO_STREAM(this->get_logger(), "MODEL_LANDING - Land_ : " << land_position.x() << "," << land_position.y() << "," << land_position.z());  
            static int count_1s=0;
            if(count_1s<30)
            {
                count_1s++;
            }
            else
            {
                messages_send(mavros_msgs::msg::StatusText::INFO, "Land: "+std::to_string(land_position.x())+","+std::to_string(land_position.y())+","+std::to_string(land_position.z()));
                count_1s=0;
            }
            // 发送给无人机控制指令
            uav_speed_control(output_vx,  output_vy, output_vz, yaw_rate);

            rate.sleep();
            continue;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // 追踪部分
        if ( fly_model_ == MODEL_TRACKING ){

            // 获取点坐标
            // 判断是否到达 
            // 未到达 接着跑 
            // 已到达 判断是否为尾
            // 是 结束
            // 不是 切换到下个点
            // 计算速度      
            if( !tracking_state )       //还没开始跑点
            {
                printAllWaypoints();    //输出要跑的所有点
                if( getNextWaypoint(target_position,target_yaw,camera))        //获取第一个目标点 
                {
                    tracking_state=true;    // 成功获取 则切换跑点状态
                    RCLCPP_INFO(this->get_logger(), "Start Tracking!");   
                }
                else
                {
                    uav_speed_control_without_yaw(0,  0, 0 );
                    rate.sleep();
                    RCLCPP_INFO(this->get_logger(), "Tracking no way!");  
                    continue;               // 没有要跑的点 结束
                }
            }
            double distance = mav_p.distance(target_position);      // 计算当前点与目标点的距离
            if (distance <= tracking_radius) {
                RCLCPP_INFO(this->get_logger(), "Arrived at target point. -----------------------------------------------------------");     //到达了目标点
                static bool take_photo=false;
                // 判断是否为拍照点 是则悬停5s 否则切换下一个点
                if( camera && elapsed_time <= camera_time)
                {
                    message.data=1;
                    servo_pwm_publisher_->publish(message);
                    if(!take_photo&& elapsed_time>= camera_time*0.2)
                    {
                        // 发送请求并使用 lambda 回调处理响应
                        auto future = camera_client_->async_send_request(request_camera,
                        [this](rclcpp::Client<camera::srv::SaveImage>::SharedFuture future) {
                            auto response = future.get();
                            if (response->success) {
                                RCLCPP_INFO_STREAM(this->get_logger(), "Image captured successfully.");
                            } else {
                                RCLCPP_ERROR_STREAM(this->get_logger(), "Image capture failed: " + response->message);
                            }
                        });
                        take_photo=true;
                        messages_send(mavros_msgs::msg::StatusText::INFO, "Taking photos !");
                    }
                    elapsed_time+=loop_rete_ts;     //时间累加
                    RCLCPP_INFO(this->get_logger(), "Taking photo, please wait......");        //到达了目标点
                }
                else
                {
                    take_photo=false;
                    elapsed_time=0;                 //时间累加变量置零
                    message.data=0;
                    servo_pwm_publisher_->publish(message);
                    //切换点
                    if( getNextWaypoint(target_position,target_yaw,camera))   //获取成功 
                    {
                        distance = mav_p.distance(target_position);     //更新距离
                        messages_send(mavros_msgs::msg::StatusText::INFO, "Arrived at target point !");
                    }
                    else        //队列为空 
                    {
                        if(tracking_mode==0.0)      //定点模式
                        {
                            tracking_state=false;
                            fly_model_ = MODEL_LANDING;     //飞完巡点 切换为降落模式
                            RCLCPP_INFO_STREAM(this->get_logger(), "Tracking finished! turn to Landing mode " );
                        }

                        // 距离10 0点小于一定距离后 认为跑完了 清空队列 并发送0 0 点
                        tf2::Vector3 set_position(set_point_x, set_point_y, 1.0);
                        if(mav_p.distance(set_position)<=tracking_radius)
                        {
                            auto msg = geometry_msgs::msg::PoseStamped();
                            // 填充 header 部分
                            msg.header.stamp = this->get_clock()->now();  // 当前时间戳
                            msg.header.frame_id = "odom";  // 设置坐标系为 "odom"
                            // 填充 pose 部分（只设置位置，姿态为空）
                            msg.pose.position.x = 0;  // 设置位置的 x 坐标
                            msg.pose.position.y = 0;  // 设置位置的 y 坐标
                            msg.pose.position.z = 1.0;  // 设置位置的 z 坐标
                            // 发布消息
                            static bool send_flag=false;
                            if(!send_flag)
                            {
                                way_point_publisher_->publish(msg);
                                RCLCPP_INFO(this->get_logger(), "Send Pose: x=%.2f, y=%.2f", msg.pose.position.x, msg.pose.position.y);
                                send_flag=true;
                            }
                        }
                        RCLCPP_INFO(this->get_logger(), "Tracking finished! -----------------------------------------------------------"); 
                        messages_send(mavros_msgs::msg::StatusText::INFO, "Tracking finished!");
                    }
                }
            } else {
                RCLCPP_INFO_STREAM(this->get_logger(), "Distance to target: " << distance);
                //接着跑点W
            }
            double output_vx = 0;
            double output_vy = 0;
            double output_vz = 0;
            double yaw_rate = 0;
            double delta_x = target_position.x() - mav_p.x();
            double delta_y = target_position.y() - mav_p.y();
            double delta_z = target_position.z() - mav_p.z();
            // PID控制器
            static double delta_x_last=0.0;
            static double delta_y_last=0.0;
            static double delta_z_last=0.0;
            // PID 计算
            output_vx = P * delta_x  + D * (delta_x - delta_x_last);                            //这个地方可以尝试师兄的那个公式了
            output_vy = P * delta_y  + D * (delta_y - delta_y_last);
            output_vz = P * delta_z  + D * (delta_z - delta_z_last);
            // 更新上一个误差
            delta_x_last = delta_x;
            delta_y_last = delta_y;            
            delta_z_last = delta_z;
            if(tracking_mode==0.0)          //定点模式
            {
                // output_vx = limit_vel(output_vx);        //单轴限速
                // output_vy = limit_vel(output_vy);
                // output_vz = limit_vel(output_vz);
                double velocity_magnitude = sqrt(output_vx * output_vx + output_vy * output_vy + output_vz * output_vz);
                // 检查是否超出最大速度限制
                if (velocity_magnitude > MAX_VEL_ABS) {     
                    // 计算缩放因子
                    double scaling_factor = MAX_VEL_ABS / velocity_magnitude;           //矢量限速
                    // 按比例缩放速度矢量
                    output_vx *= scaling_factor;
                    output_vy *= scaling_factor;
                    output_vz *= scaling_factor;
                }
            }
            else if(tracking_mode==1.0)     //定速模式
            {
                // 限位
                if(waypoint_queue.size()>=1)    //还有目标点没跑完 就固定速度跑  if(waypoint_queue.size()>=1 && camera == 0)    没跑完 且当前点不为拍照点 就定速跑 
                {
                    // 计算速度矢量的模长
                    double velocity_magnitude = sqrt(output_vx * output_vx + output_vy * output_vy + output_vz * output_vz);
                    // 计算缩放因子
                    double scaling_factor = MAX_VEL_ABS / velocity_magnitude;           //可以根据前方还有多少点来动态的规划速度
                    // 按比例缩放速度矢量
                    output_vx *= scaling_factor;
                    output_vy *= scaling_factor;
                    output_vz *= scaling_factor;
                }
                else                            //最后一个点了 或者 当前点为拍照点 按照定点的方法跑
                {
                    output_vx = limit_vel(output_vx);
                    output_vy = limit_vel(output_vy);
                    output_vz = limit_vel(output_vz);
                }
            }
                
            // 先旋转，角度误差小于±18°再开始移动
            // 计算角度误差
            double yaw_error = target_yaw - mav_yaw;
            // 保证误差在 [-π, π] 范围内
            if (yaw_error > M_PI) yaw_error -= 2 * M_PI;
            if (yaw_error < -M_PI) yaw_error += 2 * M_PI;
            // 将误差从弧度转换为度数
            double yaw_error_degrees = yaw_error * (180.0 / M_PI);
            // 检查误差绝对值是否大于 10° (这个角度是自己定的
            if (fabs(yaw_error_degrees) > tracking_angle) {
                output_vx=0;
                output_vy=0;
                output_vz=0;
                RCLCPP_INFO_STREAM(this->get_logger(), "mav_yaw : "<< mav_yaw );
                RCLCPP_INFO_STREAM(this->get_logger(), "tar_yaw : "<< target_yaw );
                RCLCPP_INFO_STREAM(this->get_logger(), "err_yaw : "<< yaw_error_degrees );
                RCLCPP_INFO_STREAM(this->get_logger(), "yaw_err too much! " );
            } 
            yaw_rate = yaw_p * yaw_error;               //计算角速度
            if (yaw_rate > 1.0) yaw_rate = 1.0;         //角速度限幅
            if (yaw_rate < -1.0) yaw_rate = -1.0;
            
            RCLCPP_INFO_STREAM(this->get_logger(), "MODEL_TRACKING - Speed : " << output_vx << "," << output_vy << "," << output_vz);  
            RCLCPP_INFO_STREAM(this->get_logger(), "MODEL_TRACKING - Local : " << mav_p.x() << "," << mav_p.y() << "," << mav_p.z());   
            RCLCPP_INFO_STREAM(this->get_logger(), "MODEL_TRACKING - Targe : " << target_position.x() << "," << target_position.y() << "," << target_position.z());  
            uav_speed_control(output_vx,  output_vy, output_vz, yaw_rate);
        }
        rate.sleep();
    }

    // 在这里执行线程退出时候的操作
    uav_speed_control_without_yaw(0,  0, 0 );
}

// 判断二维码信息是否收到了
bool tracking_node::recv_new_qrcode_info(void)
{
    static unsigned int curr_cnt = 0;
    // 尚未启动二维码
    if ( !qrcode_cnt_start_ ){
        curr_cnt = 0;
    }
    // 二维码启动了
    bool res = ( qrcode_cnt_ != curr_cnt );
    if ( res ){
        curr_cnt = qrcode_cnt_;
    }
    return res;
}

void tracking_node::way_point_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
    // 从 PoseStamped 中提取 position 和 orientation
    tf2::Vector3 position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    double yaw = 0.0, roll, pitch; // 固定 yaw 为 0
    bool camera = false; // 固定 camera 为 false

    // 如果需要，可以从 orientation 中计算 yaw（通过四元数转欧拉角）
    tf2::Quaternion quat(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf2::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll); // yaw 就是转角

    // 调用 addWaypoint 写入队列
    addWaypoint(position, yaw, camera);

    // 打印接收到的数据
    RCLCPP_INFO(this->get_logger(),
        "Received Pose: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f. Queue size: %zu",
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, yaw, waypoint_queue.size());   
}

// 接受遥控器通道
void tracking_node::rc_callback(const mavros_msgs::msg::RCIn::SharedPtr msg)
{
    // 获取channels数据
    const std::vector<uint16_t>& channels = msg->channels;
    uint16_t A_key = channels[SWITCH_OFFBOARD_CHANEL - 1];
    uint16_t E_key = channels[SWITCH_FLYMODE_CHANEL - 1];
    uint16_t G_key = channels[10 - 1];                      //标记飞行点的按键
    uint16_t C_key = channels[12 - 1];                      //标记拍照点的按键
    uint16_t P_key = channels[11 - 1];                      //控制舵机PWM的
    static uint16_t E_key_state = 0;                        //置零 当切换为板外模式后可以产生跳变沿 切换模式用
    static uint16_t G_key_state = G_key;                    //防止启动的时候就采集一个点 因为一开始我也不晓得这个键的值是多少 
    static uint16_t P_key_state = P_key; 
    auto message = std_msgs::msg::Float32();                //控制pwm和io的话题
    // 飞行控制器未解锁
    if ( A_key!=1950 ){
        fly_model_ = MODEL_UNLOCK;
        E_key_state=0;                                      //UNLOCK后将这个置零 再切换为板外模式后可以获取一个跳变沿从而触发模式切换 不然fly_model_不会变 
    }else{
        if ( E_key == 1050 && E_key_state!=E_key ){         //两个条件 1. 键值 2. 跳变沿
            fly_model_ = MODEL_TRACKING;
            E_key_state=E_key;
            messages_send(mavros_msgs::msg::StatusText::INFO, "mode to Tracking!");
        }else if ( E_key == 1950 && E_key_state!=E_key){
            fly_model_ = MODEL_LANDING;
            messages_send(mavros_msgs::msg::StatusText::INFO, "mode to Landing!");
            E_key_state=E_key;
        }else if ( E_key == 1500 && E_key_state!=E_key){
            messages_send(mavros_msgs::msg::StatusText::INFO, "mode to Take_off!");
            fly_model_ = MODEL_TAKE_OFF;
            E_key_state=E_key;
        }
    }
    if(fly_model_== MODEL_UNLOCK)           // 未解锁的前提下 
    {
        if(G_key_state!=G_key)              // 按踩点键    
        {
            if(C_key>=1750)                 // 被标记为特殊点 则拍照等动作 1750对应着拨杆向右拨 并且留有一定的余量 满量程1950
            {
                addWaypoint(mav_p,mav_yaw,true);
                RCLCPP_INFO_STREAM(this->get_logger(), "Get a camera point!  " << "position: " << mav_p.x() << "," << mav_p.y() << "," << mav_p.z() << "," << "yaw: " <<mav_yaw << ", camera: " << 1 << " " << waypoint_queue.size());
                messages_send(mavros_msgs::msg::StatusText::INFO, "get a camera point! " + std::to_string(waypoint_queue.size()));
            }
            else if(C_key>=1250)            // 被标记为普通点 采集当前点坐标与朝向
            {
                addWaypoint(mav_p,mav_yaw,false);
                RCLCPP_INFO_STREAM(this->get_logger(), "Get a point!  " << "position: " << mav_p.x() << "," << mav_p.y() << "," << mav_p.z() << "," << "yaw: " <<mav_yaw << ", camera: " << 0 << " " << waypoint_queue.size());
                messages_send(mavros_msgs::msg::StatusText::INFO, "get a point! " + std::to_string(waypoint_queue.size()));
            }
            else                            // 不采点 发送自身信息
            {
                messages_send(mavros_msgs::msg::StatusText::DEBUG, "flaymode: "+std::to_string(fly_model_));
                messages_send(mavros_msgs::msg::StatusText::DEBUG, "take_off_state: "+std::to_string(take_off_state));
                messages_send(mavros_msgs::msg::StatusText::DEBUG, "cam_down_run: "+std::to_string(usb_cam_running));
                messages_send(mavros_msgs::msg::StatusText::DEBUG, "Posi: "+std::to_string(mav_p.x())+","+std::to_string(mav_p.y())+","+std::to_string(mav_p.z()));
                if(qrcode_cnt_start_)
                {
                    messages_send(mavros_msgs::msg::StatusText::DEBUG, "Land: "+std::to_string(land_position.x())+","+std::to_string(land_position.y())+","+std::to_string(land_position.z()));
                }
            }
            G_key_state=G_key;
        }
        if(C_key>=1750)                 // 开启前灯
        {
            message.data=1;
            servo_pwm_publisher_->publish(message);
        }
        else if(C_key<=1250)            // 关闭前灯
        {
            message.data=0;
            servo_pwm_publisher_->publish(message);
        }
    }
    else if(G_key_state!=G_key) //已解锁的情况下按下采点键 发送自身状态
    {
        messages_send(mavros_msgs::msg::StatusText::DEBUG, "flaymode: "+std::to_string(fly_model_));
        messages_send(mavros_msgs::msg::StatusText::DEBUG, "take_off_state: "+std::to_string(take_off_state));
        messages_send(mavros_msgs::msg::StatusText::DEBUG, "cam_down_run: "+std::to_string(usb_cam_running));
        messages_send(mavros_msgs::msg::StatusText::DEBUG, "Posi: "+std::to_string(mav_p.x())+","+std::to_string(mav_p.y())+","+std::to_string(mav_p.z()));
        if(qrcode_cnt_start_)
        {
            messages_send(mavros_msgs::msg::StatusText::DEBUG, "Land: "+std::to_string(land_position.x())+","+std::to_string(land_position.y())+","+std::to_string(land_position.z()));
        }
        G_key_state=G_key;
        if(C_key>=1750)                 // 开启前灯
        {
            message.data=1;
            servo_pwm_publisher_->publish(message);
        }
        else if(C_key<=1250)            // 关闭前灯
        {
            message.data=0;
            servo_pwm_publisher_->publish(message);
        }
    }

    if( P_key_state!=P_key )
    {
        if(P_key>1800)
        {
            message.data=8;
            auto msg = geometry_msgs::msg::PoseStamped();
            // 填充 header 部分
            msg.header.stamp = this->get_clock()->now();  // 当前时间戳
            msg.header.frame_id = "odom";  // 设置坐标系为 "odom"
            // 填充 pose 部分（只设置位置，姿态为空）
            msg.pose.position.x = set_point_x;  // 设置位置的 x 坐标
            msg.pose.position.y = set_point_y;  // 设置位置的 y 坐标
            msg.pose.position.z = 1.0;  // 设置位置的 z 坐标
            // 发布消息
            way_point_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Send Pose: x=%.2f, y=%.2f", msg.pose.position.x, msg.pose.position.y);
            P_key=1500;
        }
        else if(P_key<1200)
        {
            message.data=7;
            auto msg = geometry_msgs::msg::PoseStamped();
            // 填充 header 部分
            msg.header.stamp = this->get_clock()->now();  // 当前时间戳
            msg.header.frame_id = "odom";  // 设置坐标系为 "odom"
            // 填充 pose 部分（只设置位置，姿态为空）
            msg.pose.position.x = 0;  // 设置位置的 x 坐标
            msg.pose.position.y = 0;  // 设置位置的 y 坐标
            msg.pose.position.z = 1.0;  // 设置位置的 z 坐标
            // 发布消息
            way_point_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Send Pose: x=%.2f, y=%.2f", msg.pose.position.x, msg.pose.position.y);
            P_key=1500;
        }
        else
        {
            message.data=7.5;
        }
        P_key_state=P_key;
        servo_pwm_publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
    }
    // 判断第7个按键状态
    //RCLCPP_INFO_STREAM(this->get_logger(), " Channel [10]" << channels[9]);
}

// 二维码位姿回调函数
#define Big_Marker_ID (123)
#define Small_Marker_ID (136)

void tracking_node::tagpose_callback(const aruco_msgs::msg::ArucoPose::SharedPtr msg)
{
    if ( !gotodom_flag ){
        RCLCPP_WARN_STREAM(this->get_logger(), " UAV Odom Not Got ... ");
        return ;
    }

    // 二维码在相机坐标系中的位姿值
    double x = msg->px;
    double y = msg->py;
    double z = msg->pz;

    double ox = msg->ox;
    double oy = msg->oy;
    double oz = msg->oz;
    double ow = msg->ow;
    size_t mark_id = msg->mark_id;

    if( (z > 0.3 && mark_id == Big_Marker_ID) || (z <= 0.8 && mark_id == Small_Marker_ID) )               //高度大于0.6 以大码为准 高度小于0.6 以小码为准
    {
        // 接收到了新的二维码数据 
        qrcode_cnt_start_ = true;
        qrcode_cnt_ ++;
        // 二维码在相机坐标系中的位置
        tag_position_u.setValue(-y,-x,-z); 
        // 相机坐标系到机身坐标系的外参数
        tf2::Vector3 T_cam2mav(0.10595, 0, 0.05632);
        // 二维码在世界坐标系中位姿
        // mav_q 是无人机坐标系相对于map坐标系的值
        tf2::Matrix3x3 R(mav_q.normalized());
        tag_position_w = mav_p + R * (tag_position_u+T_cam2mav);
        // tag_position_w= mav_p + R * (tag_position_u);
        // 降落点在世界坐标系的位置
        //tf2::Vector3 T_tag2pos(0, -0.28413, -0.151112); // 默认值 大二维码的降落点        车上的大机巢
        tf2::Vector3 T_tag2pos(0, -0.18744, -0.0034772); // 默认值 大二维码的降落点          室内的小机巢
        if (mark_id == Big_Marker_ID) {
            //T_tag2pos.setValue(0, -0.28413, -0.151112); // 更新为大二维码的降落点         车上的大机巢
            T_tag2pos.setValue(0, -0.18744, -0.0034772); // 更新为大二维码的降落点          室内的小机巢
        } else if (mark_id == Small_Marker_ID) {
            T_tag2pos.setValue(0, -0.10595, -0.02); // 小二维码的降落点保持不变
        }
        tf2::Quaternion q_land(ox, oy, oz, ow);
        tf2::Matrix3x3 R_land(q_land);
        tf2::Vector3 T_(x,y,z);
        tf2::Vector3 T_res=R_land*T_tag2pos+T_;
        tf2::Vector3 T_tag(-T_res.getY(),-T_res.getX(),-T_res.getZ());
        land_position=mav_p + R * (T_tag +T_cam2mav);
        // 二维码在相机中的朝向
        tf2::Quaternion q(ox, oy, oz, ow);
        // 四元数 机身和相机姿态旋转
        tf2::Quaternion q_camera_to_uav(1, 0, 0, 0); 
        // 二维码在map坐标系中的四元数
        tf2::Quaternion mav_q_tf2(mav_q.x(), mav_q.y(), mav_q.z(), mav_q.w());
        tf2::Quaternion q_map = mav_q_tf2 * q_camera_to_uav * q;
        // 将四元数转换为旋转矩阵
        tf2::Matrix3x3 tf2_rotation(q_map);
        // 将旋转矩阵转换回欧拉角
        double global_roll, global_pitch, global_yaw;
        tf2_rotation.getRPY(global_roll, global_pitch, global_yaw);
        // 更新目标角度
        target_yaw = global_yaw;
        RCLCPP_INFO_STREAM(this->get_logger(), "mark_id :" << mark_id << " pos: "<< x << " " << y << " " << z << ", roll: " <<mav_roll << ", pitch: " << mav_pitch);
    }
}

// MAV里程计消息接收回调函数
void tracking_node::mavpose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    double ox = msg->pose.pose.orientation.x;
    double oy = msg->pose.pose.orientation.y;
    double oz = msg->pose.pose.orientation.z;
    double ow = msg->pose.pose.orientation.w;
    // 三轴线速度
    // double twist_x = msg->twist.twist.linear.x;     
    // double twist_y = msg->twist.twist.linear.y;
    // double twist_z = msg->twist.twist.linear.z;
    
    //RCLCPP_INFO_STREAM(this->get_logger(), "Local Odom : << " << x << ',' << y << ',' << z << " >> ");

    // 位置
    mav_p.setValue(x, y, z);
    // 四元数
    mav_q.setValue(ox, oy, oz, ow);  
    //偏航角
    mav_yaw = atan2(2.0 * (ow * oz + ox * oy), 1.0 - 2.0 * (oy * oy + oz * oz));
    // 俯仰角 (Pitch)
    mav_pitch = asin(2.0 * (ow * oy - oz * ox));
    // 滚转角 (Roll)
    mav_roll = atan2(2.0 * (ow * ox + oy * oz), 1.0 - 2.0 * (ox * ox + oy * oy));

    gotodom_flag = true;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<tracking_node>("tracking_node");

    rclcpp::spin(node);
    rclcpp::shutdown();

    // 等待线程结束
    node->wait_for_all_tasks_finished();

    return 0;
}
