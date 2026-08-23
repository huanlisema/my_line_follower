from launch import LaunchDescription
from launch_ros.actions import Node

# 封装终端指令相关类
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable

# 参数声明与获取
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

# 文件包含相关
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 分组相关
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction

# 事件相关
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo

# 获取功能包下share目录路径
from ament_index_python.packages import get_package_share_directory 
import os
def generate_launch_description():
    ld = LaunchDescription()
    #启动底盘
    base_launch = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("controller"),
                "launch",
                "controller.launch.py"
            )
        )
    )
    ld.add_action(base_launch)
    #启动相机
    cam_launch = IncludeLaunchDescription(
                launch_description_source = PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("peripherals"),
                        "launch",
                        "depth_camera.launch.py"
                    )
                )
            )
    ld.add_action(cam_launch)
    #启动雷达
    lidar_launch = IncludeLaunchDescription(
            launch_description_source = PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("peripherals"),
                    "launch",
                    "include",
                    "ms200_scan.launch.py"
                )
            )
        )
    ld.add_action(lidar_launch)
    return ld