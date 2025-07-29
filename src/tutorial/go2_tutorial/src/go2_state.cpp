/*
    需求：订阅里程计消息，每当机器狗位移距离超过指定值，即在终端输出机器狗当前坐标。
    流程：
        1.包含头文件；
        2.初始化ROS2客户端；
        3.自定义节点类；
          3-1.创建里程计订阅方；
          3-2.解析里程计数据，并当条件满足时，在终端输出坐标。
        4.调用spin函数，并传入节点对象指针；
        5.资源释放。
*/

// 1.包含头文件；
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep
#include "nav_msgs/msg/odometry.hpp" // IWYU pragma: keep
 
using namespace std::placeholders;
// 3.自定义节点类；
class SubOdom: public rclcpp::Node{
public:
    SubOdom():Node("sub_odom_node"){
        last_x = 0.0;
        last_y = 0.0;
        is_first = true;
        this->declare_parameter<double>("distance",0.5);
        // 3-1.创建里程计订阅方；
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",10,std::bind(&SubOdom::on_timer,this,_1));
    }
private:
    // 用于记录上一次输出坐标的变量
    double last_x, last_y;
    bool is_first;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    // 3-2.解析里程计数据，并当条件满足时，在终端输出坐标。
    void on_timer(const nav_msgs::msg::Odometry & odom){
      double x = odom.pose.pose.position.x;
      double y = odom.pose.pose.position.y;

      if (is_first)
      {
        last_x = x;
        last_y = y;
        is_first = false;
        return;
      }


      // 计算当前坐标与上一次输出坐标的直线距离
      double distance_x = x - last_x;
      double distance_y = y - last_y;
      double distance = sqrt(distance_x * distance_x + distance_y * distance_y);

      // 判断是否符合条件
      if(distance >= this->get_parameter("distance").as_double()){
        // 输出
        RCLCPP_INFO(this->get_logger(),"当前坐标(%.2f,%.2f)",x,y);
        // 重赋值
        last_x = x;
        last_y = y;
      }

    }
};

int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端；
    rclcpp::init(argc,argv);
    // 4.调用spin函数，并传入节点对象指针；
    rclcpp::spin(std::make_shared<SubOdom>());
    // 5.资源释放。
    rclcpp::shutdown();
    return 0;
}
