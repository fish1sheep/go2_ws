/*
    需求：控制机器人以圆周运动方式巡航。
*/
// 1.包含头文件；
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"  // IWYU pragma: keep
#include "sport_model.hpp"
#include "unitree_api/msg/request.hpp"  // IWYU pragma: keep

using namespace std::chrono_literals;

// 3.自定义节点类；
class Go2Ctrl : public rclcpp::Node
{
   public:
    Go2Ctrl() : Node("go2_ctrl_node")
    {
        this->declare_parameter<double>("vx", 0.0);
        this->declare_parameter<double>("vy", 0.0);
        this->declare_parameter<double>("vyaw", 0.0);
        this->declare_parameter<int64_t>("sport_api_id", ROBOT_SPORT_API_ID_BALANCESTAND);

        // 创建一个ros2 pubilsher
        pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        timer_    = this->create_wall_timer(0.1s,
                                            std::bind(&Go2Ctrl::cruise, this));
    }

   private:
    void cruise()
    {
        unitree_api::msg::Request req;  // 创建一个运动请求msg
        int64_t                   id = this->get_parameter("sport_api_id").as_int();
        req.header.identity.api_id   = ROBOT_SPORT_API_ID_MOVE;

        if (id == ROBOT_SPORT_API_ID_MOVE)
        {
            nlohmann::json js;
            js["x"]       = this->get_parameter("vx").as_double();
            js["y"]       = this->get_parameter("vy").as_double();
            js["z"]       = this->get_parameter("vyaw").as_double();
            req.parameter = js.dump();

            // 或
            // req.parameter = "{\"x\": 0.0, \"y\": 0.0, \"z\": 0.6}";
            // RCLCPP_INFO(this->get_logger(),"req.param = %s", req.parameter.c_str());
        }

        pub_->publish(req);  // 发布数据
    }
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr                            timer_;
};

int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端；
    rclcpp::init(argc, argv);
    // 4.调用spain函数，并传入节点对象指针；
    rclcpp::spin(std::make_shared<Go2Ctrl>());
    // 5.资源释放。
    rclcpp::shutdown();
    return 0;
}
