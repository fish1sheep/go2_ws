/*
 *需求：节点启动时，实现三个功能：
 *  1.发布里程计消息
 *  2.广播里程计相关坐标变换
 *  3.发布关节状态信息
 *分析1：发布里程计信息
 *  1.先了解里程计消息的字段
 *  2.这些数据从哪获取？机器人已经发布相关话题了
 *  3.可以先订阅状态话题，解析转换成里程计消息，最后发布
 *分析2：广播里程计相关坐标变换
 *  1.需要发布机器人基坐标系与odom坐标系的相对关系
 *  2.这些相对关系与里程计数据类似
 *  3.最后发布即可
 *分析3：发布关节状态信息
 *  1.了解关节状态信息
 *  2.这些数据从哪获取？机器人已经发布相关话题了
 *  3.先订阅低层状态信息，解析转换成关节信息，最后发布
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "nav_msgs/msg/odometry.hpp"    // IWYU pragma: keep
#include "tf2/LinearMath/Quaternion.h"  // IWYU pragma: keep
#include "tf2_ros/transform_broadcaster.h"
#include "unitree_go/msg/imu_state.hpp"         // IWYU pragma: keep
#include "unitree_go/msg/low_state.hpp"         // IWYU pragma: keep
#include "unitree_go/msg/motor_state.hpp"       // IWYU pragma: keep
#include "unitree_go/msg/sport_mode_state.hpp"  // IWYU pragma: keep

using namespace std::placeholders;

class Driver : public rclcpp::Node
{
   public:
    Driver() : Node("driver")
    {
        // 初始化参数
        this->declare_parameter("publish_odom_tf", true);
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base");

        publish_odom_tf = this->get_parameter("publish_odom_tf").as_bool();
        odom_frame      = this->get_parameter("odom_frame").as_string();
        base_frame      = this->get_parameter("base_frame").as_string();
        // 发布里程计消息
        sport_mode_state_suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "lf/sportmodestate", 10, std::bind(&Driver::state_callback, this, _1));
        odom_pub_       = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 发布关节消息
        joint_names_ = {"FL_hip_joint",   "FL_thigh_joint", "FL_calf_joint",  "FR_hip_joint",
                        "FR_thigh_joint", "FR_calf_joint",  "RL_hip_joint",   "RL_thigh_joint",
                        "RL_calf_joint",  "RR_hip_joint",   "RR_thigh_joint", "RR_calf_joint"};
        joint_state_pub_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        low_state_suber_ = this->create_subscription<unitree_go::msg::LowState>(
            "lf/lowstate", 10, std::bind(&Driver::low_state_callback, this, _1));
    }

   private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr            odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                   tf_broadcaster_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr       low_state_suber_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_mode_state_suber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       joint_state_pub_;
    std::vector<std::string>                                         joint_names_;
    bool                                                             publish_odom_tf;
    std::string                                                      odom_frame, base_frame;

    void state_callback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        // 创建里程计消息
        nav_msgs::msg::Odometry odom_msg;

        // 设置时间戳
        // odom_msg.header.stamp.sec = data->stamp.sec;
        // odom_msg.header.stamp.nanosec = data->stamp.nanosec;
        odom_msg.header.stamp    = this->now();
        odom_msg.header.frame_id = odom_frame;
        odom_msg.child_frame_id  = base_frame;

        // 设置位置
        odom_msg.pose.pose.position.x = data->position[0];
        odom_msg.pose.pose.position.y = data->position[1];
        odom_msg.pose.pose.position.z = data->position[2];

        // 设置姿态
        odom_msg.pose.pose.orientation.w = data->imu_state.quaternion[0];
        odom_msg.pose.pose.orientation.x = data->imu_state.quaternion[1];
        odom_msg.pose.pose.orientation.y = data->imu_state.quaternion[2];
        odom_msg.pose.pose.orientation.z = data->imu_state.quaternion[3];

        // 设置线速度
        odom_msg.twist.twist.linear.x = data->velocity[0];
        odom_msg.twist.twist.linear.y = data->velocity[1];
        odom_msg.twist.twist.linear.z = data->velocity[2];

        // 设置角速度
        odom_msg.twist.twist.angular.z = data->yaw_speed;

        // 发布里程计消息
        odom_pub_->publish(odom_msg);

        // 根据参数选择是否发布坐标变换
        if (publish_odom_tf)
        {
            geometry_msgs::msg::TransformStamped transformStamped;

            // 设置时间戳
            // transformStamped.header.stamp.sec = data->stamp.sec;
            // transformStamped.header.stamp.nanosec = data->stamp.nanosec;
            transformStamped.header.stamp    = this->now();
            transformStamped.header.frame_id = odom_frame;
            transformStamped.child_frame_id  = base_frame;

            // 设置平移
            transformStamped.transform.translation.x = data->position[0];
            transformStamped.transform.translation.y = data->position[1];
            transformStamped.transform.translation.z = data->position[2];

            // 设置旋转
            transformStamped.transform.rotation = odom_msg.pose.pose.orientation;

            // 发布坐标变换
            tf_broadcaster_->sendTransform(transformStamped);
        }
    }
    void low_state_callback(unitree_go::msg::LowState::SharedPtr data)
    {
        // Populate the joint state message
        auto joint_state_msg         = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();
        joint_state_msg.name         = joint_names_;
        auto ms                      = data->motor_state;
        // 12个关节
        for (size_t i = 0; i < 12; i++)
        {
            joint_state_msg.position.push_back(ms[i].q);
            // RCLCPP_INFO(this->get_logger(),"角度: = %.6f",ms[i].q);
        }
        joint_state_pub_->publish(joint_state_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Driver>());
    rclcpp::shutdown();
    return 0;
}
