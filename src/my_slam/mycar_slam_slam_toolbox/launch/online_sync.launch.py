from launch import LaunchDescription
from launch_ros.actions import Node

# 封装终端指令相关类
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable

# 参数声明与获取
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# 文件包含相关
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource

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
    params_path = DeclareLaunchArgument("params_path",default_value=os.path.join(get_package_share_directory("mycar_slam_slam_toolbox"),"params","online_sync_slam.yaml"))
    #params_path = os.path.join(get_package_share_directory("mycar_slam_slam_toolbox"),"params","online_sync_slam.yaml")
    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="my_slam_toolbox",
        parameters=[LaunchConfiguration("params_path"),
                    {"use_sim_time":LaunchConfiguration("use_sim_time")}]
    )
    return LaunchDescription([
        params_path,use_sim_time,slam_node,
    ])