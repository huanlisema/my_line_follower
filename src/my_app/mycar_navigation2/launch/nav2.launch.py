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
# 导航服务器的核心节点实现

# 1. 基础部分实现
#     1-1. 生命周期管理器节点
#     在Nav2中，所有节点都是具有生命周期的节点
#     生命周期管理器节点会提供一套标准的方法，来管理Nav2中节点
#     1-2. 行为树服务器节点
#     控制导航的执行流程的
# 2. 规划器实现
# 生成从当前位置到目标点的路径
#     2-1. 规划器服务节点
#     2-2. 全局代价地图
#     规划器节点依赖于全局代价地图
# 3. 运动控制实现
# 控制机器人按照规划的路径移动
#     3-1. 运动控制节点
#     3-2. 局部地图
#     运动控制节点依赖于局部代价地图

# 4. 恢复行为实现
# 当机器人陷入困境时，自行脱困
# 5. 路点跟踪
# 设置一系列的目标点集合，机器人可以依次到达这些目标点
# 6. 路径平滑节点
# 平滑路径，使机器人运行更流畅、安全且可以减少硬件磨损
# 7. 速度平滑实现
# 平滑速度，使机器人运行更流畅、安全且可以减少硬件磨损
def generate_launch_description():
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    current_pkg = get_package_share_directory("mycar_navigation2")
    bt_yaml = os.path.join(current_pkg, "params", "bt.yaml")
    pose_xml = os.path.join(current_pkg, "bts", "nav2_pose.xml")
    poses_xml = os.path.join(current_pkg, "bts", "nav2_poses.xml")
    planner_yaml = os.path.join(current_pkg, "params", "planner.yaml")
    controller_yaml = os.path.join(current_pkg, "params", "controller.yaml")
    behavior_yaml = os.path.join(current_pkg, "params", "behavior.yaml")
    waypoint_yaml = os.path.join(current_pkg, "params", "waypoint.yaml")
    smoother_yaml = os.path.join(current_pkg, "params", "smoother.yaml")
    velocity_yaml = os.path.join(current_pkg, "params", "velocity.yaml")
    
 # 速度平滑
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            velocity_yaml
        ],
        # 应该发布能控制机器人运动的话题：cmd_vel,订阅controller 发布的话题
        remappings=[
            ("cmd_vel", "cmd_vel_nav"),#订阅控制器速度
            ("cmd_vel_smoothed", "cmd_vel"),#发布机器人底盘接收的话题
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
     
    )
# 路径平滑
    smoother_node = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            smoother_yaml
        ],
        remappings = [
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
                ]
    )
# 路点跟踪节点
    waypoint_node = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            waypoint_yaml
        ],
        remappings = [
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
                ]
    )
# 恢复行为节点
    behavior_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            behavior_yaml
        ],
        remappings = [
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
                ]
    )
# 运动控制节点
    controller_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            controller_yaml
        ],
        # controller 发布的速度话题，如果要被速度平滑器处理的话
        # 应该速度平滑器发布的速度话题与机器人一致，controller的速度不应该能控制机器人
        # remappings=[
        #     ("cmd_vel", "cmd_vel_nav")
        # ]
        remappings = [
            ('cmd_vel', 'cmd_vel_nav'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    #规划器节点
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            planner_yaml
        ],
        remappings = [
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    #行为树服务设置
    bt_server = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            # 自定义行为树并载入
            {"default_nav_to_pose_bt_xml": pose_xml},
            {"default_nav_through_poses_bt_xml": poses_xml},
            # 传入参数文件
            bt_yaml,
        ],
        remappings = [
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
# 生命周期管理器节点
    life_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"autostart": True},
            {"node_names": [  # 被托管的节点列表
                "bt_navigator",
                "planner_server",
                "controller_server",
                "behavior_server",
                "waypoint_follower",
                "smoother_server",
                "velocity_smoother",
            ]}
        ]
    )
    return LaunchDescription([
        use_sim_time,
        velocity_smoother,
        smoother_node,
        waypoint_node,
        behavior_node,
        bt_server,
        planner_server,
        controller_node,
        life_manager,
    ])