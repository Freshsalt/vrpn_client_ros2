/*
 * vrpn_listener.cpp
 * VRPNListener 节点实现
 * 功能：连接 VRPN 服务，接收跟踪器 pose/twist/accel 数据并发布为 ROS2 话题
 * 主要职责：
 *  - 建立并维护与 VRPN 服务器的连接
 *  - 自动发现发送者（sender），为每个 sender 创建 Synchronizer
 *  - 将 VRPN 回调的数据转换为 ROS2 消息并发布
 */

#include "vrpn_listener/vrpn_listener.hpp"

VRPNListener::VRPNListener(std::string name) : Node(name)
{
    // ------------------------
    // 构造函数：初始化节点状态
    // 1) 加载参数
    // 2) 建立 VRPN 连接
    // 3) 创建主循环和发现 tracker 的定时器
    // ------------------------

    // 加载参数（server/port/frame_id/频率等）
    loadParams();

    // 初始化 VRPN 连接：将 server 与 port 组合为 host:port
    std::string host = _server + ":" + std::to_string(_port);
    _vrpn_connection = std::shared_ptr<vrpn_Connection>(vrpn_get_connection_by_name(host.c_str()));

    // 创建周期性定时器，用于执行 VRPN 主循环以及刷新发现的 tracker 列表
    _mainloop_timer = this->create_wall_timer(std::chrono::seconds(1) / _mainloop_frequency, std::bind(&VRPNListener::mainloop, this));
    _trackers_refresh_timer = this->create_wall_timer(std::chrono::seconds(1) / _refresh_trackers_frequency, std::bind(&VRPNListener::refresh_trackers, this));

    RCLCPP_INFO(this->get_logger(), "VRPN listener initialized");
}

// 回调：处理接收到的 pose 数据（VRPN tracker 回调）
// 参数：
// - userData: 指向 Synchronizer 的指针（通过 register_change_handler 注册时传入）
// - poseData: VRPN 提供的位姿数据，包含位置 pos[3]、四元数 quat[4]、消息时间 msg_time
// 将 VRPN 数据转换为 ROS2 的 geometry_msgs::msg::PoseStamped，并发布到对应的 topic
void VRPN_CALLBACK VRPNListener::handlePose(void *userData, const vrpn_TRACKERCB poseData)
{
    Synchronizer *synchronizer = static_cast<Synchronizer *>(userData);
    VRPNListener *listener = static_cast<VRPNListener *>(synchronizer->listener_ptr);

    RCLCPP_INFO(listener->get_logger(), "recv new pose data from sender [%s]", synchronizer->sender_name.c_str());

    // 填充 pose 消息
    geometry_msgs::msg::PoseStamped pose_msg;
    // 位置（米）
    pose_msg.pose.position.x = poseData.pos[0];
    pose_msg.pose.position.y = poseData.pos[1];
    pose_msg.pose.position.z = poseData.pos[2];
    
    // 四元数：原始四元数绕Z轴逆时针旋转90度
    // 创建绕Z轴逆时针90度的旋转四元数 (q_z_90)
    tf2::Quaternion q_original(poseData.quat[0], poseData.quat[1], poseData.quat[2], poseData.quat[3]);
    tf2::Quaternion q_z_90(0, 0, std::sin(M_PI / 4.0), std::cos(M_PI / 4.0));  // 绕Z轴90度旋转
    
    // 组合四元数：q_final = q_z_90 * q_original
    tf2::Quaternion q_rotated = q_z_90 * q_original;
    
    // 将旋转后的四元数赋值到消息（x,y,z,w）
    pose_msg.pose.orientation.x = q_rotated.x();
    pose_msg.pose.orientation.y = q_rotated.y();
    pose_msg.pose.orientation.z = q_rotated.z();
    pose_msg.pose.orientation.w = q_rotated.w();

    // 设置消息头（frame 与时间戳）
    pose_msg.header.frame_id = listener->_frame_id;
    // VRPN 的时间以 timeval 给出，这里转换为 ROS2 时间（秒 + 纳秒）
    pose_msg.header.stamp.sec = poseData.msg_time.tv_sec;
    pose_msg.header.stamp.nanosec = poseData.msg_time.tv_usec * 1000;

    // 发布 pose
    synchronizer->pose_publisher->publish(pose_msg);
}

// 回调：处理接收到的 twist（速度）数据
// 将线速度直接赋值到 twist.linear，
// 对于角速度这里通过速度对应的四元数转为 RPY（角度）并放入 twist.angular
void VRPN_CALLBACK VRPNListener::handleTwist(void *userData, const vrpn_TRACKERVELCB twistData)
{
    Synchronizer *synchronizer = static_cast<Synchronizer *>(userData);
    VRPNListener *listener = static_cast<VRPNListener *>(synchronizer->listener_ptr);

    RCLCPP_INFO(listener->get_logger(), "recv new twist data from sender [%s]", synchronizer->sender_name.c_str());

    // 填充 twist 消息
    geometry_msgs::msg::TwistStamped twist_msg;
    // 线速度（m/s）
    twist_msg.twist.linear.x = twistData.vel[0];
    twist_msg.twist.linear.y = twistData.vel[1];
    twist_msg.twist.linear.z = twistData.vel[2];
    // 角速度：由四元数转换为 RPY（弧度）作为 angular 分量
    tf2::Matrix3x3 rot_mat(
        tf2::Quaternion(
            twistData.vel_quat[0],
            twistData.vel_quat[1],
            twistData.vel_quat[2],
            twistData.vel_quat[3]));
    double roll, pitch, yaw;
    rot_mat.getRPY(roll, pitch, yaw);
    twist_msg.twist.angular.x = roll;
    twist_msg.twist.angular.y = pitch;
    twist_msg.twist.angular.z = yaw;

    // 发布 twist
    synchronizer->twist_publisher->publish(twist_msg);
}

// 回调：处理接收到的加速度（accel）数据
// 将线加速度放入 accel.linear，角加速度由四元数转换为 RPY 后放入 accel.angular
void VRPN_CALLBACK VRPNListener::handleAccel(void *userData, const vrpn_TRACKERACCCB accelData)
{
    Synchronizer *synchronizer = static_cast<Synchronizer *>(userData);
    VRPNListener *listener = static_cast<VRPNListener *>(synchronizer->listener_ptr);

    RCLCPP_INFO(listener->get_logger(), "recv new accel data from sender [%s]", synchronizer->sender_name.c_str());

    // 填充 accel 消息
    geometry_msgs::msg::AccelStamped accel_msg;
    // 线加速度（m/s^2）
    accel_msg.accel.linear.x = accelData.acc[0];
    accel_msg.accel.linear.y = accelData.acc[1];
    accel_msg.accel.linear.z = accelData.acc[2];
    // 角加速度：由四元数转换为 RPY（弧度）
    tf2::Matrix3x3 rot_mat(
        tf2::Quaternion(
            accelData.acc_quat[0],
            accelData.acc_quat[1],
            accelData.acc_quat[2],
            accelData.acc_quat[3]));
    double roll, pitch, yaw;
    rot_mat.getRPY(roll, pitch, yaw);
    accel_msg.accel.angular.x = roll;
    accel_msg.accel.angular.y = pitch;
    accel_msg.accel.angular.z = yaw;

    // 发布 accel
    synchronizer->accel_publisher->publish(accel_msg);
}

// 调用 VRPN 连接的 mainloop，检查连接状态
// 该函数在定时器中周期调用，用于处理来自 VRPN 的网络事件
void VRPNListener::vrpnConnectionMainloop()
{
    _vrpn_connection->mainloop();

    if (!_vrpn_connection->doing_okay())
    {
        RCLCPP_WARN(this->get_logger(), "VRPN connection is not 'doing okay'");
    }
}

// 定期刷新 VRPN 上的 sender 列表，发现新的 sender 则创建对应的 Synchronizer
void VRPNListener::refresh_trackers()
{
    for (int i = 0; _vrpn_connection->sender_name(i) != NULL; i++)
    {
        std::string sender_name = _vrpn_connection->sender_name(i);

        if (_synchronizers.count(sender_name) != 0)
        {
            // 已存在对应的 synchronizer，跳过
            continue;
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Found new sender: " << sender_name);
            // 为新 sender 创建同步器并注册回调
            createSynchronizer(sender_name);
        }
    }
}

// 为指定 sender 创建 Synchronizer：
// - 为该 sender 创建 vrpn_Tracker_Remote
// - 创建对应的 ROS2 发布者（pose/twist/accel）
// - 注册 VRPN 的回调函数（pose/twist/accel）到本对象
// - 创建 tracker 的定时器循环（执行 vrpn_Tracker_Remote::mainloop）
// - 将 synchronizer 保存到 _synchronizers 映射中
void VRPNListener::createSynchronizer(std::string sender_name)
{
    std::string &tracker_name = sender_name;
    std::string &synchronizer_name = sender_name;
    std::string rigid_name = sender_name;
    // 将 sender 名中的空格替换为下划线，便于作为 ROS topic 名称的一部分
    replaceSpace(rigid_name);

    // new synchronizer
    std::shared_ptr<Synchronizer> new_synchronizer = std::make_shared<Synchronizer>();
    new_synchronizer->sender_name = sender_name;
    new_synchronizer->listener_ptr = this;
    // 创建 VRPN tracker 对象，绑定到共享的 VRPN 连接
    new_synchronizer->vrpn_tracker = std::make_shared<vrpn_Tracker_Remote>(tracker_name.c_str(), _vrpn_connection.get());

    // 为该 tracker 创建 ROS topic 名称
    std::string pose_topic_name = "/vrpn/" + rigid_name + "/pose";
    std::string twist_topic_name = "/vrpn/" + rigid_name + "/twist";
    std::string accel_topic_name = "/vrpn/" + rigid_name + "/accel";
    // 创建发布者
    new_synchronizer->pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_name, 1);
    new_synchronizer->twist_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_name, 1);
    new_synchronizer->accel_publisher = this->create_publisher<geometry_msgs::msg::AccelStamped>(accel_topic_name, 1);

    // 注册 VRPN 回调处理函数，用户数据传入 new_synchronizer 的裸指针
    new_synchronizer->vrpn_tracker->register_change_handler(new_synchronizer.get(), &VRPNListener::handlePose);
    new_synchronizer->vrpn_tracker->register_change_handler(new_synchronizer.get(), &VRPNListener::handleTwist);
    new_synchronizer->vrpn_tracker->register_change_handler(new_synchronizer.get(), &VRPNListener::handleAccel);

    // 为该 tracker 创建独立的定时器，周期性调用其 mainloop
    this->create_wall_timer(
        std::chrono::seconds(1) / _tracker_mainloop_frequency,
        std::bind(&vrpn_Tracker_Remote::mainloop, new_synchronizer->vrpn_tracker));

    // 将新创建的 synchronizer 注册到 map 中
    _synchronizers.insert(std::make_pair(synchronizer_name, new_synchronizer));

    RCLCPP_INFO_STREAM(this->get_logger(), "New synchronizer created: " << synchronizer_name);
}

// 节点主循环：当前仅调用 VRPN 连接的 mainloop
void VRPNListener::mainloop()
{
    vrpnConnectionMainloop();
}

// 模板函数：声明并加载参数
// param_name: 参数名称
// default_value: 参数默认值（用于 declare_parameter）
// param: 输出参数（加载到此引用中）
template <class T>
void VRPNListener::loadParam(std::string param_name, T default_value, T &param)
{
    // 声明参数，设置默认值
    this->declare_parameter(param_name, default_value);

    // 尝试获取参数值到 param 中
    bool success = this->get_parameter(param_name, param);

    if (!success)
    {
        RCLCPP_INFO(this->get_logger(), "load param [%s] failed", param_name.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "load param [%s] success", param_name.c_str());
    }
}

// 加载所有节点相关参数（从参数服务器或默认值）
// 支持参数：server, port, frame_id, mainloop_frequency, refresh_trackers_frequency, tracker_mainloop_frequency
void VRPNListener::loadParams()
{
    loadParam(std::string("server"), std::string(""), _server);
    loadParam(std::string("port"), 3883, _port);
    loadParam(std::string("frame_id"), std::string("world"), _frame_id);
    loadParam(std::string("mainloop_frequency"), 100.0, _mainloop_frequency);
    loadParam(std::string("refresh_trackers_frequency"), 1.0, _refresh_trackers_frequency);
    loadParam(std::string("tracker_mainloop_frequency"), 100.0, _tracker_mainloop_frequency);
}

// Helper：将字符串中的空格替换为下划线，便于用于 topic 名称
void VRPNListener::replaceSpace(std::string &ori_str)
{
    for (int i = 0; i < int(ori_str.size()); i++)
    {
        if (ori_str[i] == ' ')
        {
            ori_str[i] = '_';
        }
    }
}

// 程序入口：初始化 ROS2，创建 VRPNListener 节点并运行
int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建 VRPNListener 节点实例
    std::string name = "vrpn_listener";
    auto vrpn_listener = std::make_shared<VRPNListener>(name);

    // 进入 ROS2 事件循环，直到节点被中止
    rclcpp::spin(vrpn_listener);

    // 退出 ROS2
    rclcpp::shutdown();

    return 0;
}
