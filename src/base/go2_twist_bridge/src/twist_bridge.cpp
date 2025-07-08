/*
需求: 订阅twist消息，并将之转换为go2所需的request消息以控制机器狗运动。
实现:
  1. 创建一个Request的发布对象;
  2. 创建twist订阅对象;
  3. 在回调函数中实现消息的转换以及发布。
*/

#include "geometry_msgs/msg/twist.hpp"  // IWYU pragma: keep
#include "nlohmann/json.hpp"            // IWYU pragma: keep
#include "rclcpp/rclcpp.hpp"            // IWYU pragma: keep
#include "sport_model.hpp"
#include "unitree_api/msg/request.hpp"  // IWYU pragma: keep

using namespace std::placeholders;

class TwistBridge : public rclcpp::Node
{
   public:
    TwistBridge() : Node("my_node")
    {
        RCLCPP_INFO(this->get_logger(), "TwistBridge创建，可以将geometry_msgs/msg/twist消息转换为unitree_api/msg/request消息！");
        request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        twist_sub_   = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&TwistBridge::twist_cb, this, _1));
    }

   private:
    void twist_cb(const geometry_msgs::msg::Twist::SharedPtr twist)
    {
        unitree_api::msg::Request request;
        double                    x = twist->linear.x;
        double                    y = twist->linear.y;
        double                    z = twist->angular.z;
        // 默认api_id为平衡站立
        auto api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
        if (x != 0 || y != 0 || z != 0)
        {
            api_id = ROBOT_SPORT_API_ID_MOVE;
            // 设置参数 --- 组织一个字符串样式的速度指令
            nlohmann::json js;
            js["x"] = x;
            js["y"] = y;
            js["z"] = z;
            request.parameter = js.dump();
        }
        request.header.identity.api_id = api_id ;
        request_pub_->publish(request);
    }
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr    request_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistBridge>());
    rclcpp::shutdown();
    return 0;
}
