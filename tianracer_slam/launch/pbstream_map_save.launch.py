import os
import time
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

default_namespace = os.environ.get("TIANBOT_NAME", "")
default_namespace = f"" if default_namespace == '' or default_namespace =='/' else default_namespace
service_namespace_prefix = f"" if default_namespace == '' else f"/{default_namespace}"

def generate_launch_description():

    # map named is map+current time
    current_time = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())
    map_filename = 'map_' + current_time + '.pbstream'

    # pbstream 地图保存目录（cartographer 产物）
    map_directory = os.path.join(
        get_package_share_directory('tianracer_slam'), 'pbstreams')

    # set and check save files
    os.makedirs(map_directory, exist_ok=True)

    map_save_config = os.path.join(map_directory, map_filename)

    return LaunchDescription(
        [
            # 结束当前轨迹，再写出 pbstream 状态文件。
            # 用 argv 列表 + shell=False：请求体（YAML 字符串）作为单个参数传入，
            # 不经 shell 二次解析，避免路径含空格/引号时被拆坏。
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                     f'{service_namespace_prefix}/finish_trajectory',
                     'cartographer_ros_msgs/srv/FinishTrajectory',
                     '{trajectory_id: 0}'],
            ),
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                     f'{service_namespace_prefix}/write_state',
                     'cartographer_ros_msgs/srv/WriteState',
                     "{filename: '%s', include_unfinished_submaps: true}"
                     % map_save_config],
            ),
        ]
    )