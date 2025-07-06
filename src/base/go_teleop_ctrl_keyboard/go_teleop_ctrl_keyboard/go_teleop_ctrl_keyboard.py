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
import json

ROBOT_SPORT_API_ID5 = {
    # 基础控制指令
    "DAMP":                1001,  # 阻尼控制
    "BALANCESTAND":        1002,  # 平衡站立
    "STOPMOVE":            1003,  # 停止运动
    "STANDUP":             1004,  # 站立
    "STANDDOWN":           1005,  # 站立下降
    "RECOVERYSTAND":       1006,  # 恢复站立

    # 运动控制指令
    "EULER":               1007,  # 欧拉角控制
    "MOVE":                1008,  # 移动
    "SIT":                 1009,  # 坐下
    "RISESIT":             1010,  # 从坐下恢复站立
    "SWITCHGAIT":          1011,  # 切换步态
    "TRIGGER":             1012,  # 触发

    # 参数调整指令
    "BODYHEIGHT":          1013,  # 身体高度调整
    "FOOTRAISEHEIGHT":     1014,  # 脚部抬起高度调整
    "SPEEDLEVEL":          1015,  # 速度级别调整

    # 交互动作指令
    "HELLO":               1016,  # 打招呼
    "STRETCH":             1017,  # 伸展
    "TRAJECTORYFOLLOW":    1018,  # 轨迹跟随
    "CONTINUOUSGAIT":      1019,  # 连续步态
    "WALLOW":              1021,  # 打滚
    "DANCE1":              1022,  # 舞蹈1
    "DANCE2":              1023,  # 舞蹈2

    # 状态获取指令
    "GETBODYHEIGHT":       1024,  # 获取身体高度
    "GETFOOTRAISEHEIGHT":  1025,  # 获取脚部抬起高度
    "GETSPEEDLEVEL":       1026,  # 获取速度级别

    # 高级控制指令
    "SWITCHJOYSTICK":      1027,  # 切换操纵杆
    "POSE":                1028,  # 姿态
    "SCRAPE":              1029,  # 刮擦
    "FRONTFLIP":           1030,  # 前空翻
    "FRONTJUMP":           1031,  # 前跳
    "FRONTPOUNCE":         1032   # 前扑
}
sportModel = {
    'h': ROBOT_SPORT_API_ID5["HELLO"],
    'j': ROBOT_SPORT_API_ID5["FRONTJUMP"],
    'k': ROBOT_SPORT_API_ID5["STRETCH"],
    'n': ROBOT_SPORT_API_ID5["SIT"],
    'm': ROBOT_SPORT_API_ID5["RISESIT"],
    'y': ROBOT_SPORT_API_ID5["DANCE1"],
    'u': ROBOT_SPORT_API_ID5["DANCE2"]
}
moveBindings = {
    'w': (1, 0, 0, 0),  # x * 1 ,y * 0,z * 0,角速度 * 0
    'e': (1, 0, 0, -1),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
    'q': (1, 0, 0, 1),
    's': (-1, 0, 0, 0),
    'c': (-1, 0, 0, 1),
    'z': (-1, 0, 0, -1),
    'E': (1, -1, 0, 0),
    'W': (1, 0, 0, 0),
    'A': (0, 1, 0, 0),
    'D': (0, -1, 0, 0),
    'Q': (1, 1, 0, 0),
    'S': (-1, 0, 0, 0),
    'C': (-1, -1, 0, 0),
    'Z': (-1, 1, 0, 0),
}


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_ctrl_keyboard')
        self.pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.declare_parameter("speed", 0.2)  # x和上的线速度一致
        self.declare_parameter("angular", 0.5)


        self.speed = self.get_parameter("speed").value
        self.angular = self.get_parameter("angular").value
        # 如果参数为None，则手动设置默认值
        self.speed = self.speed if self.speed is not None else 0.2
        self.angular = self.angular if self.angular is not None else 0.5

    def publish(self, apid_id, x=0.0, y=0.0, z=0.0):
        req = Request()
        req.header.identity.api_id = apid_id
        js = {"x": x, "y": y, "z": z}
        req.parameter = json.dumps(js)
        self.pub.publish(req)


def getKey(settings):
    # 设置读取模式
    tty.setraw(sys.stdin.fileno())
    # 获取一个按键
    key = sys.stdin.read(1)
    # 恢复终端原始设置
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # 返回按键值
    return key


def main():
    # 读取键盘录入
    # 1.获取标准输入流终端属性并返回
    settings = termios.tcgetattr(sys.stdin)
    # ros2 node 实现，需要单独子线程处理
    rclpy.init()
    teleopNode = TeleopNode()
    spinner = threading.Thread(target=rclpy.spin, args=(teleopNode,))
    spinner.start()

    # 3.循环读取按键，并且映射
    # 异常处理，主要利用finally 保证资源释放
    try:
        while True:
            key = getKey(settings)
            print(key)
            # if key == 'h':
            #     teleopNode.publish(1016)

            # 1.结束终端判断 ctrl+c-->站立平衡
            if key == '\x03':
                break
            # 2.运动模式切换（设置 api_id）
            elif key in sportModel.keys():
                teleopNode.publish(sportModel[key])
            # 3.运动控制（设置 api_id和速度消息）
            elif key in moveBindings.keys():
                x_bind = moveBindings[key][0]
                y_bind = moveBindings[key][1]
                z_bind = moveBindings[key][3]
                teleopNode.publish(ROBOT_SPORT_API_ID5["MOVE"], x = x_bind * teleopNode.speed,
                               y = y_bind * teleopNode.speed, z = z_bind * teleopNode.angular)
            # 4.速度控制（设置 api_id和速度消息）
    finally:
        # go2进入站立平衡状态
        teleopNode.publish(ROBOT_SPORT_API_ID5["BALANCESTAND"])
        rclpy.shutdown()


if __name__ == '__main__':
    main()
