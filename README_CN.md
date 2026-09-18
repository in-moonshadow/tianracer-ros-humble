# 天驰无人竞速车 Tianracer — ROS 2 Humble

[English](README.md)

> **本仓库是上游项目 [tianracer](https://github.com/tianbot/tianracer)（天之博特 Tianbot 的天驰无人竞速车平台）的
> ROS 2 Humble 版本。** 上游项目基于 **ROS 1**；本项目是其 **ROS 2 Humble 移植与持续开发版本**，
> 在上游 `humble-devel` 分支基础上继续开发。归属与许可证说明见 [NOTICE](NOTICE)。

Tianracer 是一款低成本无人竞速车平台。本工作区提供其 **ROS 2 Humble 的完整仿真栈**：
Gazebo Sim 世界、底盘控制链、SLAM、Nav2 导航，以及竞速裁判系统。

## 包结构

| 包 | 说明 |
|---|---|
| `tianracer` | 元包 |
| `tianracer_description` | URDF/xacro 机器人模型与 TF |
| `tianracer_gazebo` | 仿真栈：Gazebo Sim 7 (Fortress) 世界、底盘控制器、里程计桥接、竞速裁判 |
| `tianracer_navigation2` | Nav2 配置与启动（多套规划器/控制器参数档、按赛道自动选档、`cmd_vel` → `ackermann_cmd` 转换） |
| `tianracer_slam` | SLAM 启动：slam_toolbox / Cartographer / GMapping，以及地图保存 |
| `tianracer_rviz` | RViz 配置与查看入口（激光、IMU、里程计、图像、模型、建图） |
| `tianracer_vision` | 视觉功能包（巡线节点） |
| `tianracer_competition` | 竞速算法（disparity extender）与路点导航 |

## 环境要求

- Ubuntu 22.04 + **ROS 2 Humble**（colcon / ament）
- **Gazebo Sim 7 (Fortress)** 及 `ros_gz_sim`、`ros_gz_bridge`、`gz_ros2_control`
- `ros2_control` / `ros2_controllers`、Nav2、`slam_toolbox`、`ackermann_msgs`
- 可选：Cartographer（`cartographer_ros`）、TEB（`teb_local_planner` + `costmap_converter`）

本项目**不使用 Gazebo Classic**（与已安装的 Gazebo Sim 冲突），仿真统一走 Gazebo Sim 7 (Fortress)。

## 构建

各包位于仓库根目录，因此**仓库本身就是 colcon 工作区**：

```bash
git clone https://github.com/in-moonshadow/tianracer-ros-humble
cd tianracer-ros-humble
rosdep install --from-paths . --ignore-src -r -y   # 安装可由 apt 解析的依赖
colcon build --symlink-install
source install/setup.bash
```

TEB 是可选依赖，Humble 下无 apt 包。若需要 TEB 相关变体，先拉取源码依赖：

```bash
vcs import src < tianracer.repos
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
```

默认导航栈是 `navfn_dwb`，**不需要**上述任何源码依赖。

## 运行

### 一键竞速（仿真 + 控制链 + Nav2 + 裁判）

```bash
ros2 launch tianracer_gazebo tianracer_race.launch.py
```

常用参数（括号内为默认值）：

| 参数 | 含义 |
|---|---|
| `world_name`（`tianracer_racetrack`） | 赛道名。同时决定地图 `<world_name>.yaml` 与该赛道的 Nav2 参数档 |
| `gui`（`false`） | 是否启动 Gazebo Sim 3D 界面（默认关闭以省 CPU） |
| `use_rviz`（`true`） | 是否启动 RViz |
| `use_planner`（`navfn_dwb`） | 规划器/控制器组合 |
| `enable_display`（`true`） | 是否显示裁判计分板窗口（tkinter，需要 `DISPLAY`） |
| `lap_count`（`3`） | 完赛圈数 |
| `checkpoint_timeout`（`30.0`） | 单个检查门的超时秒数 |

启动完成后在**裁判计分板窗口**点「启动」发车。

### 仅启动仿真

```bash
ros2 launch tianracer_gazebo tianracer_on_racetrack.launch.py gui:=true world:=tianracer_racetrack.world
```

或使用封装脚本（会**先清理上次残留的 Gazebo Sim 进程**——强烈建议，
上一次的 `gz sim` 残留会破坏下一次启动）：

```bash
bash tianracer_gazebo/scripts/run_sim.sh gui:=true
```

### 建图

```bash
ros2 launch tianracer_slam slam_toolbox.launch.py     # 推荐
ros2 launch tianracer_slam cartographer.launch.py
ros2 launch tianracer_slam gmapping.launch.py
ros2 launch tianracer_rviz view_mapping.launch.py     # 建图时查看

ros2 launch tianracer_slam map_save.launch.py         # 保存地图
```

### 导航

```bash
ros2 launch tianracer_navigation2 nav2.launch.py
ros2 launch tianracer_navigation2 nav2.launch.py world:=raicom use_planner:=navfn_teb
```

`use_planner` 的取值对应 `tianracer_navigation2/params/` 下的参数档：
`navfn_dwb`（默认）、`navfn_teb`、`navfn_mppi`、`navfn_mppi_fwd`、`smac_dwb`、`smac_graceful`、
`theta_star_dwb`、`theta_star_mppi`、`theta_star_rpp`、`theta_star_vector_pur`。

赛道专属参数档（`navfn_dwb_<赛道>_nav2_params.yaml`）会按 `world:=` **自动选中**，取不到时回退通用档。

### 反应式竞速（disparity extender）

```bash
ros2 launch tianracer_competition closed_map.launch.py speed_param:=3.5 P_param:=0.0
ros2 launch tianracer_competition open_map.launch.py   speed_param:=3.5 P_param:=0.0
```

## 环境变量

由 `tianracer_description` 注入（见 `env-hooks/99.tianracer.dsv.in`）：

| 变量 | 取值 | 作用 |
|---|---|---|
| `TIANRACER_BASE` | `compact`（默认）/ `standard` / `fullsize` | 车型：决定 TF 分支与轴距 |
| `TIANRACER_LIDAR` | `richbeam`（默认）/ `rplidar_a1` / `velodyne` / `rslidar` | 雷达型号：决定激光/IMU 静态变换分支 |
| `TIANBOT_NAME` | 任意 | 机器人命名空间（空 = 无命名空间） |

## 范围说明：纯仿真

本工作区**已剥离全部实机（真车）内容**。驱动物理小车的包 ——
`tianracer_bringup`（硬件总装、激光/相机/GPS/手柄 launch、udev/systemd 部署脚本）、
`tianracer_core`（TianBoard Mini 串口底盘驱动 + EKF）、`tianracer_gps`、`tianracer_teleop`、
`tianracer_jetson`、`tianracer_test` —— 均已删除。需要真机部署时，可从上游仓库
[tianbot/tianracer](https://github.com/tianbot/tianracer) 恢复。

### 无头环境运行

Gazebo Sim 的渲染在 GUI 进程中进行；在没有 GPU/DRI 设备的机器上，传感器数据
（激光、IMU、相机）可能无法产出。可以接显示器运行，或强制使用软件渲染：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
```

## 许可证

本项目以 **GNU General Public License v3.0** 发布，全文见 [`LICENSE`](LICENSE)。

本项目是 [tianbot/tianracer](https://github.com/tianbot/tianracer) 的**衍生作品**；
上游以 GPL-3.0 发布，依该许可证的传染性条款，衍生作品必须同样以 GPL-3.0 分发。
上游版权归其原作者（天之博特 Tianbot）所有，上游的许可证声明均已保留 ——
完整归属声明（含上游 `tianracer_navigation2` 包的 MIT 声明）见 [`NOTICE`](NOTICE)。

上游项目本身基于 [HyphaROS RaceCar](https://github.com/Hypha-ROS/hypharos_racecar) 开发，
在此致谢其作者 HaoChih LIN、KaiChun Wu。
