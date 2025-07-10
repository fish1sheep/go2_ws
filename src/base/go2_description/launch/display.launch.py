import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions.if_condition import IfCondition
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# from launch.substitutions import Command
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
# 文件包含相关-------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    go2_decription_pkg = get_package_share_directory("go2_description")
    use_joint_state_publisher = DeclareLaunchArgument(
        name = "use_joint_state_publisher",
        default_value= "true"
    )
    model = DeclareLaunchArgument(
        name= "urdf_path",
        default_value= os.path.join(go2_decription_pkg,"urdf","go2_description.urdf")
    )

    robot_desc = ParameterValue(Command(["xacro ",LaunchConfiguration("urdf_path")]))

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable= "robot_state_publisher",
        parameters=[{"robot_description":robot_desc}]
    )
    # 启动是有条件的
    joint_state_publisher = Node(
        package= "joint_state_publisher",
        executable="joint_state_publisher",
        condition = IfCondition(LaunchConfiguration("use_joint_state_publisher"))
    )
    return LaunchDescription([
        model,
        use_joint_state_publisher,
        robot_state_publisher,
        joint_state_publisher,
    ])
