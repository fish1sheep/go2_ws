/*
    需求：向巡航服务端发送请求数据，如果发送的是非0数据，那么就开始巡航，否则就终止巡航，
         不论何种请求，服务端响应的数据是机器人的坐标，客户端还需要解析结果并输出在终端。
    流程：
        1.包含头文件；
        2.初始化ROS2客户端；
        3.自定义节点类；
          3-1.创建客户端；
          3-2.连接服务端；
          3-3.发送请求。
        4.创建自定义类对象，并调用连接服务以及请求发送请求的函数；
        5.处理响应结果；
        6.资源释放。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "go2_tutorial_inter/srv/cruising.hpp"

using namespace std::chrono_literals;
// 3.自定义节点类；
class CruClient: public rclcpp::Node{
public:
    CruClient():Node("cru_client_node"){
        client = this->create_client<go2_tutorial_inter::srv::Cruising>("cruising");
        RCLCPP_INFO(this->get_logger(),"客户端创建，等待连接服务端！");
    }
    bool connect_server(){
      while (!client->wait_for_service(1s))
      {
        if (!rclcpp::ok())
        {
          return false;
        }

        RCLCPP_INFO(this->get_logger(),"服务连接中，请稍候...");
      }
      return true;
    }
    rclcpp::Client<go2_tutorial_inter::srv::Cruising>::FutureAndRequestId send_request(int32_t flag){
      auto request = std::make_shared<go2_tutorial_inter::srv::Cruising::Request>();
      request->flag = flag;
      return client->async_send_request(request);
    }
private:
    rclcpp::Client<go2_tutorial_inter::srv::Cruising>::SharedPtr client;
};

int main(int argc, char const *argv[])
{
    // 处理通过终端提交的数据
    if (argc != 2){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请提交一个整型数据！");
      return 1;
    }
    // 2.初始化ROS2客户端；
    rclcpp::init(argc,argv);

    // 4.创建自定义类对象，并调用连接服务以及请求发送请求的函数；
    auto client = std::make_shared<CruClient>();
    // 连接服务端
    bool flag = client->connect_server();
    if (!flag)
    {
      RCLCPP_INFO(client->get_logger(),"服务连接失败!");
      return 0;
    }
    auto response = client->send_request(atoi(argv[1]));

    // 5.处理响应结果；
    if (rclcpp::spin_until_future_complete(client,response) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(client->get_logger(),"请求正常处理");
      auto cru_res = response.get();
      RCLCPP_INFO(client->get_logger(),"响应坐标:(%.3f,%.3f)", cru_res->point.x,cru_res->point.y);

    } else {
      RCLCPP_INFO(client->get_logger(),"请求异常");
    }

    // 6.资源释放。
    rclcpp::shutdown();
    return 0;
}
