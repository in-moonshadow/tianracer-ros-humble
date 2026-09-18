import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch.conditions import IfCondition

default_namespace = os.environ.get("TIANBOT_NAME", "")
default_namespace = f"" if default_namespace == '' or default_namespace =='/' else default_namespace
default_frame_id = f"map" if default_namespace == '' else f"{default_namespace}/map"

# 出生位姿单一来源：tianracer_gazebo/scripts/track_spawn.py（与 gz spawn、裁判 reset 共用）。
_GZ_LIB_DIR = os.path.abspath(os.path.join(
    get_package_share_directory('tianracer_gazebo'), '..', '..', 'lib', 'tianracer_gazebo'))
if _GZ_LIB_DIR not in sys.path:
    sys.path.insert(0, _GZ_LIB_DIR)
from track_spawn import DEFAULT_TRACK, get_spawn  # noqa: E402

# 默认赛道 / 规划器（与下面的 DeclareLaunchArgument 保持一致）。
DEFAULT_MAP = 'tianracer_racetrack'
DEFAULT_PLANNER = 'navfn_dwb'
# 注入后的 params 写到这里（固定名 + 覆盖写）：不用 mkstemp，避免每轮仿真残留一个
# 随机命名的 /tmp 文件堆积（dwb_round.sh 会跑很多轮）。
SPAWN_PARAMS_DIR = '/tmp/tianracer_nav2_params'

# 恢复行为树：装在 tianracer_navigation2/behavior_trees/ 下的本车专用 BT。
# 该文件 = Nav2 原生版逐字一致，只把恢复序列从
#   清地图 → Spin(1.57) → Wait(5s) → BackUp(0.30@0.05)
# 改成
#   清地图 → BackUp(0.35@0.30) → Wait(2s)
# 理由见文件头注释（类车不能原地旋转，Spin 必失败却排第一，吃光裁判 10s 停车预算）。
#
# 此前**没有任何 params 设过 bt_navigator.default_nav_to_pose_bt_xml**，params 里只有一行
# 注释说「用默认值」，于是 Nav2 一直回退用 /opt/ros/humble 里的原生版
# （实测整批轮次 nav2.log 里 `Running backup` 计数恒为 0、卡死轮都伴随 `spin failed`，
# 即此回退的指纹）。这里由父 launch 注入路径，是 Nav2 官方示例认可的方式
# （见 nav2_bringup/params/nav2_params.yaml 该键上方注释）；路径在 install 侧解析，
# 不把绝对路径写死进 params（换机器/换工作区即失效）。
BT_XML_REL = os.path.join('behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')


def _params_with_spawn(src_params, world, out_dir=SPAWN_PARAMS_DIR):
    """改写 params（AMCL 初始位姿 + 恢复行为树），返回改写后 yaml 的路径。

    为什么必须改写初始位姿：amcl.initial_pose 与 gz 里车的实际生成点必须一致。本仓库把
    AMCL 的随机粒子注入关掉了（recovery_alpha_slow/fast = 0.0），初始位姿错了无法自愈——
    实测初始 xy 偏 16m 时定位必然发散。历史上二者各写一份硬编码值，换赛道漏改即出错
    （2026-09-14 racetrack_1 / raicom 生成在 16m / 6.7m 外而 AMCL 仍以为在原点）。

    为什么必须改写 BT 路径：见 BT_XML_REL 处的说明——BT 文件早就在仓库里，只是从没接上。
    """
    with open(src_params, encoding='utf-8') as f:
        params = yaml.safe_load(f)
    changed = False
    if 'bt_navigator' in params:
        params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = os.path.join(
            get_package_share_directory('tianracer_navigation2'), BT_XML_REL)
        changed = True
    if 'amcl' in params:
        x, y, _z, yaw = get_spawn(world)
        params['amcl']['ros__parameters']['initial_pose'] = {
            'x': float(x), 'y': float(y), 'z': 0.0, 'yaw': float(yaw)}
        changed = True
    if not changed:
        # 既无定位段也无 bt_navigator 段 → 原样返回，不做无谓改写
        return src_params
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, '%s_params.yaml' % (world.replace('.world', '') or DEFAULT_TRACK))
    with open(out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(params, f, allow_unicode=True)
    return out


def generate_launch_description():
    nav2_dir = os.path.join(get_package_share_directory('tianracer_navigation2'))
    # 地图存放在 tianracer_gazebo/maps，可用 map:=<path> 覆盖
    map_src_dir = os.path.join(get_package_share_directory('tianracer_gazebo'), 'maps')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('tianracer_navigation2'),'launch')
    rviz_config_file = os.path.join(get_package_share_directory('tianracer_rviz'),'rviz_cfg','nav2_namespaced_view.rviz')

    # use_sim_time 默认 true：本工作区已剥离全部实机内容、只剩纯仿真栈（见 README「范围说明」）。
    # 该值会透传给 bringup_launch.py 与 rviz_launch.py；rviz 若吃 false 会走墙钟，
    # 而 TF/scan 带的是仿真时间戳（从 0 累积），于是 tf2 缓存判定「消息比缓存里所有
    # 数据都早」并持续丢弃："Message Filter dropping message: frame 'odom' ... the
    # timestamp on the message is earlier than all the data in the transform cache"。
    # 原默认 'false' 直跑 nav2.launch.py 时必然触发（use_rviz 默认 true 会一并起 rviz）。
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_namespace = LaunchConfiguration('use_namespace', default='true')
    use_map = LaunchConfiguration('use_map', default=DEFAULT_MAP)
    use_planner = LaunchConfiguration('use_planner', default=DEFAULT_PLANNER)

    # https://answers.ros.org/question/358655/ros2-concatenate-string-to-launchargument/
    namespace = LaunchConfiguration(
        'namespace',
        default = f'{default_namespace}'
    )

    map_dir = LaunchConfiguration(
        'map',
        default = [map_src_dir, '/', use_map, '.yaml']
    )

    param_dir = LaunchConfiguration(
        'params_file',
        default = [nav2_dir, '/params/', use_planner, '_nav2_params.yaml']
    )

    # 出生位姿：由 track_spawn 决定（与 gz spawn / 裁判 reset 同源）。
    world = LaunchConfiguration('world', default=DEFAULT_TRACK)

    # 把 AMCL 初始位姿改写进一份临时 params，再交给 bringup_launch。
    # 用 OpaqueFunction：此时 LaunchContext 才可求值（LaunchConfiguration 的默认值是
    # substitution 列表，须经 perform_substitutions 展开成字符串，不能取 .default）。
    def _per_track_params(planner, world, base):
        """按赛道选参数：优先 <planner>_<world>_nav2_params.yaml，无则回退 base。

        竞赛赛道是随机的，而各赛道可行配置不同（racetrack_1 需 max_vel_x 2.5 +
        global inflation_radius 0.45 才能从确定性 2/9 变成 5/5）。把专属值放进同名文件，
        竞赛入口即可保持 use_planner:=navfn_dwb 不变而自动取到该赛道的最优配置。
        base 即 params_file 的解析值；调用方显式传了 params_file 时直接尊重它，
        不做赛道覆盖（否则用户的自定义参数文件会被静默忽略）。

        ⚠️ 查的是 install/ 侧（nav2_dir 即 install share）。本工作区 --symlink-install
        只为构建时已存在的文件建软链，新建的 params 必须
        `colcon build --symlink-install --packages-select tianracer_navigation2`，
        否则永远回退，表现为「改了配置却不生效」。
        """
        generic = os.path.join(nav2_dir, 'params', '%s_nav2_params.yaml' % planner)
        if base != generic:
            return base, 'explicit'
        w = (world or DEFAULT_TRACK).replace('.world', '')
        cand = os.path.join(nav2_dir, 'params', '%s_%s_nav2_params.yaml' % (planner, w))
        return (cand, 'per_track') if os.path.exists(cand) else (generic, 'generic')

    def _bringup_with_spawn(context):
        lc = context.launch_configurations
        planner = lc.get('use_planner') or DEFAULT_PLANNER
        world = lc.get('world') or DEFAULT_TRACK
        base = perform_substitutions(context, [param_dir])
        src, kind = _per_track_params(planner, world, base)
        # 显式打日志：不做声明的回退正是「配置改了没生效」这类问题的温床。
        if kind == 'per_track':
            msg = '导航参数文件【赛道专属】%s' % src
        elif kind == 'explicit':
            msg = '导航参数文件【显式指定】%s' % src
        else:
            msg = ('导航参数文件【通用】%s（无 %s_%s_nav2_params.yaml）'
                   % (src, planner, world.replace('.world', '')))
        out = _params_with_spawn(src, world)
        return [LogInfo(msg=msg), IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'namespace': lc.get('namespace', default_namespace),
                'use_namespace': lc.get('use_namespace', 'true'),
                'use_composition': 'False',
                'map': perform_substitutions(context, [map_dir]),
                'use_sim_time': lc.get('use_sim_time', 'true'),
                'params_file': out,
            }.items(),
        )]

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time',default_value=use_sim_time,description='Use simulation (Gazebo) clock if true'),
            DeclareLaunchArgument('use_rviz', default_value='True', description='Whether to start RVIZ'),
            DeclareLaunchArgument('map',default_value = map_dir,description = 'Full path to map file to load'),
            DeclareLaunchArgument('params_file',default_value = param_dir,description = 'Full path to param file to load'),
            # 赛道名（裸名或带 .world 后缀都可）：决定 AMCL 初始位姿，须与实际生成点一致。
            DeclareLaunchArgument('world', default_value=DEFAULT_TRACK,
                                  description='赛道名，决定 AMCL 初始位姿（与 gz spawn 同源）'),

            OpaqueFunction(function=_bringup_with_spawn),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'rviz_launch.py')),
                condition=IfCondition(use_rviz),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'rviz_config': rviz_config_file,
                }.items(),
            )
        ])
