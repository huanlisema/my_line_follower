from launch import LaunchDescription
from launch_ros.actions import Node

# 封装终端指令相关类
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable

# 参数声明与获取
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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
    use_sim_time = DeclareLaunchArgument("use_sim_time",default_value="false")
    #包含slam建图launch
    slam_pkg = get_package_share_directory("mycar_slam_slam_toolbox")
    slam_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(slam_pkg, "launch", "online_sync.launch.py")
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time"))
        ]
    )
    # 包含导航 launch
    nav2_pkg = get_package_share_directory("mycar_navigation2")
    nav2_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(nav2_pkg, "launch", "nav2.launch.py")
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))]
    )

    return LaunchDescription([
        use_sim_time,
        nav2_launch,
        slam_launch,
    ])