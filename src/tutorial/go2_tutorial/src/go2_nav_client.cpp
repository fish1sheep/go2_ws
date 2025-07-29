/*
   需求：向导航动作服务端发送目标点数据，并处理服务端的响应数据。
   步骤：
       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
            3-1.创建动作客户端；
            3-2.发送请求数据，并处理服务端响应；
            3-3.处理目标响应；
            3-4.处理响应的连续反馈；
            3-5.处理最终响应。
       4.调用spin函数，并传入节点对象指针；
       5.释放资源。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep
#include "rclcpp_action/rclcpp_action.hpp" // IWYU pragma: keep
#include "go2_tutorial_inter/action/nav.hpp" // IWYU pragma: keep

using namespace std::chrono_literals;
using namespace std::placeholders;

// 3.定义节点类；
class NavClient: public rclcpp::Node{
public:
    NavClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    :Node("exe_nav_action_client",options){
        // 3-1.创建动作客户端；
        nav_client = rclcpp_action::create_client<go2_tutorial_inter::action::Nav>(this,"nav");
    }
    // 3-2.发送请求数据，并处理服务端响应；
    void send_goal(float x){
        // 连接动作服务端，如果超时（5s），那么直接退出。
        if (!nav_client->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(this->get_logger(),"服务连接失败!");
            return;
        }
        // 组织请求数据
        auto goal_msg = go2_tutorial_inter::action::Nav::Goal();
        goal_msg.goal = x;

        rclcpp_action::Client<go2_tutorial_inter::action::Nav>::SendGoalOptions options;
        options.goal_response_callback = std::bind(&NavClient::goal_response_callback, this, _1);
        options.feedback_callback = std::bind(&NavClient::feedback_callback, this, _1, _2);
        options.result_callback = std::bind(&NavClient::result_callback, this, _1);
        // 发送
        nav_client->async_send_goal(goal_msg,options);
        // 判断是否关闭终端
    }
    ~NavClient() {
        nav_client->async_cancel_all_goals();
    }
private:
    rclcpp_action::Client<go2_tutorial_inter::action::Nav>::SharedPtr nav_client;

    // 3-3.处理目标响应；
    void goal_response_callback(rclcpp_action::ClientGoalHandle<go2_tutorial_inter::action::Nav>::SharedPtr goal_handle){
        if(!goal_handle){
            RCLCPP_ERROR(this->get_logger(),"目标请求被服务器拒绝");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(),"目标请求被接收!");
            // std::thread(&NavClient::cancel_goals,this,goal_handle).detach();
        }
    }
    // void cancel_goals(rclcpp_action::ClientGoalHandle<go2_tutorial_inter::action::Nav>::SharedPtr goal_handle){
    //     while (rclcpp::ok()){}
    //     // nav_client->async_cancel_all_goals();
    //     nav_client->async_cancel_goal(goal_handle);
    // }
    // 3-4.处理响应的连续反馈；
    void feedback_callback(rclcpp_action::ClientGoalHandle<go2_tutorial_inter::action::Nav>::SharedPtr goal_handle, 
        const std::shared_ptr<const go2_tutorial_inter::action::Nav::Feedback> feedback){
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(),"距离目标点还有 %.2f 米。",feedback->distance);

    }
    // 3-5.处理最终响应。
    void result_callback(const rclcpp_action::ClientGoalHandle<go2_tutorial_inter::action::Nav>::WrappedResult & result){
        switch (result.code){
        case rclcpp_action::ResultCode::SUCCEEDED :
            RCLCPP_INFO(this->get_logger(),"go2最终坐标:(%.2f,%.2f)",result.result->point.x,result.result->point.y);
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(),"任务被取消");
            break;      
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(),"任务被中止");
            break;   
        default:
            RCLCPP_ERROR(this->get_logger(),"未知异常");
            break;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char const *argv[])
{
    if (argc != 2)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请传入要前进的距离数据");
        return 1;
    }
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc,argv);
    // 4.调用spin函数，并传入节点对象指针；
    auto client = std::make_shared<NavClient>();
    // 发送目标点
    client->send_goal(atof(argv[1]));
    rclcpp::spin(client);
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}