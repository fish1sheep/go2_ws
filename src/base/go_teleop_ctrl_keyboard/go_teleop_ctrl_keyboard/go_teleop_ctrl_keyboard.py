"""
需求：
    编写键盘控制节点，控制GO2运动

    移动按键
        w:向前移动。
        e:向后并向左转移动(在普通模式下)。
        a:向左转。
        d:向右转。
        q:向前并向右转移动(在普通模式下)。
        s:向后移动。
        c:向后并向左转移动(在全向模式下，与e相对)。
        z:向前并向右转移动(在全向模式下，与 q相对)。
        当按下 shift 键时，机器人进入全向模式(Holonomic mode)，允许它进行侧向移动(strafing)



        W:向前移动(全向模式与 w 相同)。
        E:向左移动。
        A:向后并向左移动。
        D:向后并向右移动，
        Q:向右移动。
        S:停止移动。
        C:向前并向左移动.
        Z:向前并向右移动。

    速度调整按键
        r:增加最大速度和转向速度10%。
        t:减少最大速度和转向速度10%。
        f:仅增加线性速度10%(不影响转向速度)。
        g:仅减少线性速度10%。
        v:仅增加转向速度10%(不影响线性速度)
        b:仅减少转向速度10%。

    运动模式切换
        h:打招呼。
        j:前跳。
        k:伸懒腰。
        n:坐下。
        m:从坐下恢复。
        y:跳舞1。
        u:跳舞2

实现流程：
    1.引入键盘捕获的库
    2.编写ROS2节点
    3.编写按键映射逻辑
"""

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request

import termios
import sys
import tty
import threading

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_ctrl_keyboard')
        self.pub = self.create_publisher(Request,"/api/sport/request",10)

    def publish(self,apid_id):
        req = Request()
        req.header.identity.api_id = apid_id
        self.pub.publish(req)

def getKey(settings):
    # 设置读取模式
    tty.setraw(sys.stdin.fileno())
    # 获取一个按键
    key = sys.stdin.read(1)
    # 恢复终端原始设置
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
    # 返回按键值
    return key

def main():
    # 读取键盘录入
    # 1.获取标准输入流终端属性并返回
    settings = termios.tcgetattr(sys.stdin)
    # ros2 node 实现，需要单独子线程处理
    rclpy.init()
    teleopNode = TeleopNode()
    spinner = threading.Thread(target=rclpy.spin,args=(teleopNode,))
    spinner.start()

    while True:
        key = getKey(settings)
        print(key)
        if key == 'h':
            teleopNode.publish(1016)


if __name__ == '__main__':
    main()
