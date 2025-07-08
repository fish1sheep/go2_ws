"""
需求: 订阅twist消息，并将之转换为go2所需的request消息以控制机器狗运动。
实现:
  1. 创建一个Request的发布对象;
  2. 创建twist订阅对象;
  3. 在回调函数中实现消息的转换以及发布。
"""
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
from geometry_msgs.msg import Twist
from .sport_model import ROBOT_SPORT_API_IDS
import json
class TwistBridge(Node):
    def __init__(self):
        super().__init__('twist_bridge')
        self.request_pub = self.create_publisher(Request,'/api/sport/request',10)
        self.twist_sub = self.create_subscription(Twist,"cmd_vel",self.twist_cb,10)

    def twist_cb(self,twist:Twist):
        request = Request()
        x = twist.linear.x
        y = twist.linear.y
        z = twist.angular.z

        api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]
        if x!=0 or y!=0 or z!=0:
            api_id = ROBOT_SPORT_API_IDS["MOVE"]
            js = {"x": x, "y": y, "z": z}
            request.parameter = json.dumps(js)
            self.get_logger().info(f'Current speed: {request.parameter}')

        request.header.identity.api_id = api_id
        self.request_pub.publish(request)

def main():
    rclpy.init()
    rclpy.spin(TwistBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
