/*
 * 需求：编写第一个小程序，控制机器狗打招呼
 * 流程：
 *  1.创建发布对象；
 *  2.创建定时器；
 *  3.定时器中，调用发布对象发布速度指令
 */
#include "rclcpp/rclcpp.hpp"            // IWYU pragma: keep
#include "unitree_api/msg/request.hpp"  // IWYU pragma: keep

using namespace std::chrono_literals;

class HelloWorld : public rclcpp::Node
{
   public:
    HelloWorld() : Node("helloworld")
    {
        RCLCPP_INFO(this->get_logger(), "HelloWorld创建");
        pub_   = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&HelloWorld::on_timer, this));
    }

   private:
    void on_timer()
    {
        unitree_api::msg::Request request;
        request.header.identity.api_id = 1016;
        pub_->publish(request);
    }
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr                            timer_;
};

int main(int argc, char **argv)
{
    // 初始化ROS2客户端；
    rclcpp::init(argc, argv);
    // 创建节点
    auto node = std::make_shared<HelloWorld>();
    // 调用spin函数，并传入节点对象指针；
    rclcpp::spin(node);
    // 资源释放。
    rclcpp::shutdown();
    return 0;
}
