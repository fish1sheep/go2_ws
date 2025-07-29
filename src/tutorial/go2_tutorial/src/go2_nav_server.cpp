
/*
    需求：简单的模拟导航功能，向机器人发送一个前进N米的请求，机器人就会以 0.1m/s的速度前进，
         当与目标点的距离小于0.05m时，机器人就会停止运动，返回机器人的停止坐标，并且在此过程中，
         会连续反馈机器人与目标点之间的剩余距离。
    流程：
        1.包含头文件；
        2.初始化ROS2客户端；
        3.自定义节点类；
          3-1.创建参数服务客户端，连接到速度发布节点；
          3-2.创建订阅方，订阅机器人的里程计以获取机器人坐标；
          3-3.创建动作服务端，解析客户端的相关并生成响应。
        4.调用spin函数，并传入节点对象指针；
        5.资源释放。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep
#include "nav_msgs/msg/odometry.hpp" // IWYU pragma: keep
#include "rclcpp_action/rclcpp_action.hpp" // IWYU pragma: keep
#include "go2_tutorial_inter/action/nav.hpp" // IWYU pragma: keep
#include "geometry_msgs/msg/point.hpp" // IWYU pragma: keep
#include "sport_model.hpp" // IWYU pragma: keep

using namespace std::chrono_literals;
using namespace std::placeholders;
// 3.自定义节点类；
class NavServer: public rclcpp::Node{
public:
    NavServer():Node("nav_server_node_cpp"){

        this->declare_parameter("vx",0.1);
        this->declare_parameter("error",0.2);

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
        // 3-2.创建订阅方，订阅机器人的里程计以获取机器人坐标；
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "odom",
                    10,
                    std::bind(&NavServer::on_timer,this,_1));
        // 3-3.创建动作服务端，解析客户端的相关并生成响应。
        nav_action_server_ = rclcpp_action::create_server<go2_tutorial_inter::action::Nav>(
                    this,
                    "nav",
                    std::bind(&NavServer::handle_goal,this,_1,_2),
                    std::bind(&NavServer::handle_cancel,this,_1),
                    std::bind(&NavServer::handle_accepted,this,_1)
        );
    }
private:
    rclcpp::AsyncParametersClient::SharedPtr paramClient;
    geometry_msgs::msg::Point current_point;
    geometry_msgs::msg::Point start_point; // 每次导航时，机器人起点坐标
    // double error; // 终点允许误差
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp_action::Server<go2_tutorial_inter::action::Nav>::SharedPtr nav_action_server_;

    void on_timer(const nav_msgs::msg::Odometry & odom){
        current_point = odom.pose.pose.position;
    }

    // 解析动作客户端发送的请求；
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & goal_uuid, std::shared_ptr<const go2_tutorial_inter::action::Nav::Goal> goal){
        (void)goal_uuid;
        float goal_distance = goal->goal;
        if (goal_distance > 0.0)
        {
          RCLCPP_INFO(this->get_logger(),"请求前进%.2f米", goal_distance);
          start_point = current_point;
        } else {
          RCLCPP_INFO(this->get_logger(),"只许进，不许退!");
          return rclcpp_action::GoalResponse::REJECT;
        }
        paramClient->set_parameters({
            this->get_parameter("vx"),
            rclcpp::Parameter("sport_api_id",ROBOT_SPORT_API_ID_MOVE)
            });

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 处理动作客户端发送的取消请求；
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<go2_tutorial_inter::action::Nav>> goal_handle){
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(),"任务取消中....!");
        balance_stand();
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void balance_stand(){
        paramClient->set_parameters({
            rclcpp::Parameter("vx",0.0),
            rclcpp::Parameter("sport_api_id",ROBOT_SPORT_API_ID_BALANCESTAND)
            });
    }
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<go2_tutorial_inter::action::Nav>> goal_handle){
        RCLCPP_INFO(this->get_logger(),"开始执行任务......");
        // 获取目标距离
        float goal = goal_handle->get_goal()->goal;
        // 连续反馈
        auto feedback = std::make_shared<go2_tutorial_inter::action::Nav::Feedback>();
        // 最终结果
        auto result = std::make_shared<go2_tutorial_inter::action::Nav::Result>();

        // 设置连续反馈
        rclcpp::Rate rate(1.0);
        while(rclcpp::ok()){
          // 检查任务是否被取消；
          if(goal_handle->is_canceling()){
            result->point = current_point;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "任务取消");
            // start_point = current_point;
            balance_stand();
            return;
          }   

          double distance = sqrt(pow(current_point.x - start_point.x,2) + pow(current_point.y - start_point.y,2));
          feedback->distance = goal - distance;
          goal_handle->publish_feedback(feedback);

          if (goal - distance <= this->get_parameter("error").as_double())
          {
            break;
          }

          rate.sleep();
        }
        // 设置最终结果
        if (rclcpp::ok()) {
          result->point = current_point;
          goal_handle->succeed(result);
          balance_stand();
          RCLCPP_INFO(this->get_logger(), "任务完成！");
        }
    }
    // 创建新线程处理请求；
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<go2_tutorial_inter::action::Nav>> goal_handle){
        std::thread{std::bind(&NavServer::execute,this,_1),goal_handle}.detach();
    }

};
int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端；
    rclcpp::init(argc,argv);
    // 4.调用spain函数，并传入节点对象指针；
    rclcpp::spin(std::make_shared<NavServer>());
    // 5.资源释放。
    rclcpp::shutdown();
    return 0;
}