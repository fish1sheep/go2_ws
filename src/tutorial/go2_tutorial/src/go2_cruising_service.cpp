/*
    需求：巡航服务端，当客户端发送请求时，如果请求数据是1那么开始巡航，如果是0那么就结束巡航，
         并且无论是开始巡航还是结束巡航，都需要返回机器人当时的坐标。
    流程：
        1.包含头文件；
        2.初始化ROS2客户端；
        3.自定义节点类；
          3-1.创建参数服务客户端，连接到速度发布节点；
          3-2.创建订阅方，订阅机器人的里程计以获取机器人坐标。
          3-3.创建服务端，处理客户端请求，如果提交的是1,那么通过参数客户端设置有效的角速度、线速度数据，
              如果是0,那么通过参数客户端将速度指令置0。
        4.调用spin函数，并传入节点对象指针；
        5.资源释放。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep
#include "nav_msgs/msg/odometry.hpp" // IWYU pragma: keep
#include "go2_tutorial_inter/srv/cruising.hpp" // IWYU pragma: keep
#include "sport_model.hpp" // IWYU pragma: keep
#include "geometry_msgs/msg/point.hpp" // IWYU pragma: keep

using namespace std::chrono_literals;
using namespace std::placeholders;
// 3.自定义节点类；
class CruServer: public rclcpp::Node{
public:
    CruServer():Node("cru_server_node"){
        this->declare_parameter("vx",0.0);
        this->declare_parameter("vy",0.0);
        this->declare_parameter("vyaw",0.5);
        // 3-1.创建参数服务客户端，连接到速度发布节点；
        paramClient = std::make_shared<rclcpp::AsyncParametersClient>(this,"go2_ctrl_node");
        // 等待服务连接
        while (!paramClient->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
              return;
            }
            RCLCPP_INFO(this->get_logger(),"服务未连接");
        }
        //
        RCLCPP_INFO(this->get_logger(),"已经连接成功速度发送节点的参数服务，可以设置线速度和角速度了");
        // 3-2.创建订阅方，订阅机器人的里程计以获取机器人坐标。
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",10,std::bind(&CruServer::on_timer,this,_1));
        // 3-3.创建服务端，处理客户端请求，如果提交的是1,那么通过参数客户端设置有效的角速度、线速度数据，
        //     如果是0,那么通过参数客户端将速度指令置0。
        service = this->create_service<go2_tutorial_inter::srv::Cruising>("cruising",std::bind(&CruServer::cb,this,_1,_2));
    }
private:
    void cb(const go2_tutorial_inter::srv::Cruising::Request::SharedPtr req,
            const go2_tutorial_inter::srv::Cruising::Response::SharedPtr res)
    {
        int flag = req->flag;
        int64_t id = ROBOT_SPORT_API_ID_BALANCESTAND;
        // 判断提交的数据
        if(flag != 0){ // 开始
            RCLCPP_INFO(this->get_logger(),"开始巡航......");
            id = ROBOT_SPORT_API_ID_MOVE;
        } else { // 结束
            RCLCPP_INFO(this->get_logger(),"终止巡航......");
        }
        paramClient->set_parameters({
                this->get_parameter("vx"),
                this->get_parameter("vy"),
                this->get_parameter("vyaw"),
                rclcpp::Parameter("sport_api_id",id)
            });
        res->point = current_point;
    }

    void on_timer(const nav_msgs::msg::Odometry & odom){
        current_point = odom.pose.pose.position;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    // rclcpp::SyncParametersClient::SharedPtr paramClient;
    rclcpp::AsyncParametersClient::SharedPtr paramClient;
    rclcpp::Service<go2_tutorial_inter::srv::Cruising>::SharedPtr service;
    geometry_msgs::msg::Point current_point;
};
int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端；
    rclcpp::init(argc,argv);
    // 4.调用spain函数，并传入节点对象指针；
    rclcpp::spin(std::make_shared<CruServer>());
    // 5.资源释放。
    rclcpp::shutdown();
    return 0;
}
