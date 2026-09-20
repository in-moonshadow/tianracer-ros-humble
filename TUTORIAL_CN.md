# 天驰无人竞速车 · ROS 2 Humble 仿真教程

> 本教程面向**第一次接触本项目**的使用者，目标是从零把仿真竞速跑起来：
> 装依赖 → 构建 → 一键发车 → 看计分板 → 建图导航 → 换赛道 → 排障。
>
> 本仓库是上游项目 [tianracer](https://github.com/tianbot/tianracer)（ROS 1，天之博特/Tianbot）的
> **ROS 2 Humble 版本**，只保留可独立运行的**纯仿真栈**。与上游的关系、许可证与归属见 [NOTICE](NOTICE)。

## 目录

- [1. 开始之前：本仓库有什么、没有什么](#1-开始之前本仓库有什么没有什么)
- [2. 环境准备](#2-环境准备)
- [3. 获取与构建](#3-获取与构建)
- [4. 跑通一键竞速](#4-跑通一键竞速)
- [5. 计分规则与判罚](#5-计分规则与判罚)
- [6. 建图与导航](#6-建图与导航)
- [7. 换赛道](#7-换赛道)
- [8. 环境变量与多车](#8-环境变量与多车)
- [9. 无头 / 远程环境](#9-无头--远程环境)
- [10. 排障速查](#10-排障速查)
- [11. 本仓库不含测试工具链](#11-本仓库不含测试工具链)
- [12. 许可证与出处](#12-许可证与出处)

---

## 1. 开始之前：本仓库有什么、没有什么

**有**（8 个 ROS 2 包）：

| 包 | 作用 |
|---|---|
| `tianracer` | 元包 |
| `tianracer_description` | 机器人 URDF/xacro 模型与 TF |
| `tianracer_gazebo` | 仿真栈：Gazebo 世界、底盘控制器、里程计桥接、**竞速裁判** |
| `tianracer_navigation2` | Nav2 配置与启动（多套规划器/控制器参数档） |
| `tianracer_slam` | slam_toolbox / Cartographer / GMapping 建图与存图 |
| `tianracer_rviz` | RViz 配置与查看入口 |
| `tianracer_vision` | 视觉巡线节点 |
| `tianracer_competition` | 反应式竞速（disparity extender）与路点导航 |

**没有**（读教程时请先知道，避免走死路）：

- **全部实机内容**。驱动物理小车的 `tianracer_bringup`（硬件总装、雷达/相机/GPS/手柄 launch、
  udev/systemd 部署）、`tianracer_core`（串口底盘驱动 + EKF）、`tianracer_gps`、`tianracer_teleop`、
  `tianracer_jetson`、`tianracer_test` 都已移除。需要真机请回到上游仓库
  [tianbot/tianracer](https://github.com/tianbot/tianracer)。
- **测试/调参工具链**（轮次 harness、参数生成器、地图配准工具等）。详见[第 11 节](#11-本仓库不含测试工具链)。

---

## 2. 环境准备

### 2.1 系统要求

| 项 | 要求 |
|---|---|
| 操作系统 | Ubuntu 22.04 (Jammy) |
| ROS | **ROS 2 Humble**（colcon / ament，Python 3.10） |
| 仿真后端 | **Gazebo Sim 6（Fortress）** + `ros_gz` + `gz_ros2_control` |
| 图形 | 建议有显示（计分板是 tkinter 窗口）；无 GPU 也能跑，见[第 9 节](#9-无头--远程环境) |

> 本教程假设你**已经装好 ROS 2 Humble**。若还没有，按 ROS 2 官方安装文档
> （`docs.ros.org` → Humble → Installation → Ubuntu (Debian packages)）完成
> `ros-humble-desktop` 的安装，并确认 `printenv ROS_DISTRO` 输出 `humble`。

### 2.2 安装仿真与导航依赖

**推荐用 `rosdep` 自动解析**（依赖已写在各包的 `package.xml` 里）：

```bash
sudo rosdep init 2>/dev/null || true   # 已初始化过会报错，忽略即可
rosdep update
sudo apt update
```

装完第三节的代码后再执行 `rosdep install`（见 [3.2](#32-安装依赖并构建)）。

**手动安装**（不想用 rosdep 时，以下包名均在 Ubuntu 22.04 + Humble 上验证过）：

```bash
sudo apt install \
  ros-humble-ros-gz ros-humble-gz-ros2-control \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-slam-toolbox ros-humble-ackermann-msgs \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-cartographer-ros          # 可选：只在用 Cartographer 建图时需要
```

### 2.3 ★ 关于 Gazebo 版本：必须是 **6（Fortress）**

这一点必须单独强调，因为它是最容易踩的坑：

- `ros-humble-ros-gz-sim` 与 `ros-humble-gz-ros2-control` 的依赖**指向 `libignition-gazebo6`**，
  即 Fortress（Gazebo Sim **6**）。装上它们就会得到正确的版本，**不需要**另外装 v7。
- 本项目启动仿真时显式写死 `ign gazebo --force-version 6`。原因是 **`gz_ros2_control` 不为
  Gazebo Sim 7（Garden）导出插件**，用 v7 运行会直接报：

  ```
  does not export any plugins
  ```

- 如果你的机器上 v6/v7 都装了，`gz sim` 默认走 v7 —— 所以**不要**把脚本里的
  `--force-version 6` 去掉。

**自检：**

```bash
ign gazebo --versions      # 应输出 6.x（如 6.18.0）
```

若提示 `ign: command not found`，补装 Fortress 集合包（它提供 `ign` 命令行工具）：

```bash
sudo apt install ignition-fortress
```

---

## 3. 获取与构建

### 3.1 克隆

**本仓库本身就是 colcon 工作区**（各包位于仓库根目录，不在 `src/` 下）：

```bash
git clone https://github.com/in-moonshadow/tianracer-ros-humble
cd tianracer-ros-humble
```

### 3.2 安装依赖并构建

```bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 很重要：它让源码改动（尤其 launch 与参数档）无需重新构建即可生效。

### 3.3 可选：TEB 规划器

默认导航栈是 `navfn_dwb`，**不需要**任何源码依赖。只有想用 TEB 变体时才需要拉源码包
（Humble 下 TEB 与 costmap_converter 没有 apt 包）：

```bash
sudo apt install python3-vcstool        # 提供 vcs 命令（ros-dev-tools 也包含它）
vcs import src < tianracer.repos
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
```

#### 关于 OpenCV 版本（只有装了第二套 OpenCV 才需要看）

**如果你的机器上只有 ROS 2 配套的那套 OpenCV**（`apt install libopencv-dev` 装的 4.5.4），
上面的命令**直接就能编过，不需要任何额外参数**。

只有当你**另外还装了一套 OpenCV**（常见于在 `/usr/local` 下源码编译的 4.11）时，
`costmap_converter` 才会编译失败，报：

```
error: invalid new-expression of abstract class type 'BlobDetector'
```

原因：`find_package(OpenCV)` 会优先命中 `/usr/local` 的 4.11，而 4.11 把
`cv::SimpleBlobDetector::setParams/getParams` 改成了纯虚函数。把 OpenCV 指回 apt 那套即可，
路径按**当前机器的架构**自动展开（x86_64 / aarch64 通用，无需手改）：

```bash
colcon build --symlink-install --cmake-args \
  -DOpenCV_DIR=/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)/cmake/opencv4
```

本工作区只有 `costmap_converter` 用到 OpenCV，其余包不受影响。

> ⚠️ **不要**把这个参数写死成仓库里的 `colcon_defaults.yaml`。除了「机器相关配置不该入库」，
> 还有一个更隐蔽的原因：CMake 遇到**不存在的** `OpenCV_DIR` 是**静默回退**的（不报错）。
> 于是一个硬编码 x86_64 路径的文件，在 aarch64/Jetson 上会毫无提示地失效，
> 恰恰在最需要它的平台上不起作用——必须报错才好排查。

### 3.4 构建自检

```bash
source install/setup.bash
ros2 pkg list | grep tianracer        # 应列出 8 个包
ros2 launch tianracer_gazebo tianracer_race.launch.py --show-args   # 应打印参数表
```

---

## 4. 跑通一键竞速

### 4.1 ★ 启动前：先清掉上一次的残留进程

**这是本项目最容易导致「启动不起来」的原因。** 上一次仿真若没有正常退出，残留的
gz 进程会占住端口/共享内存，让下一次启动失败或行为异常。

```bash
pkill -9 -f "ign gazebo"
```

> 注意：进程名是 **`ign gazebo`**，不是 `gz sim`——用 `pkill -f "gz sim"` 匹配不到。
>
> 清理**刻意没有**写进 launch 文件：清理动作与 gz 启动是并发的，写进去会误杀本次刚启动的
> 服务端。所以它必须由你在 launch **之前**手动执行。

### 4.2 一键启动

```bash
ros2 launch tianracer_gazebo tianracer_race.launch.py
```

这一条命令会依次拉起四件事：

1. **Gazebo Sim**（加载赛道世界）
2. **控制链**：`gz_ros2_control` 加载底盘控制器 → `servo_commands` 把阿克曼指令写进控制器
3. **Nav2**：按赛道自动选择参数档，并把 AMCL 初始位姿改写成该赛道的出生点
4. **竞速裁判**：计分板窗口 + 检查门判定 + 评分

### 4.3 可用参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `world_name` | `tianracer_racetrack` | 赛道名（**不带** `.world` 后缀）。同时决定地图 `<world_name>.yaml` 与专属参数档 |
| `gui` | `false` | 是否开 Gazebo 3D 界面（默认关，省 CPU） |
| `use_rviz` | `true` | 是否启动 RViz |
| `use_planner` | `navfn_dwb` | 规划器/控制器组合，取值见 [6.3](#63-规划器变体) |
| `enable_display` | `true` | 是否显示计分板窗口（tkinter，**需要 `DISPLAY`**） |
| `lap_count` | `3` | 完赛圈数（3 圈 × 3 门 = 9 门） |
| `checkpoint_timeout` | `30.0` | 单个检查门允许的最长间隔秒数 |

常用组合示例：

```bash
# 带 3D 界面 + RViz
ros2 launch tianracer_gazebo tianracer_race.launch.py gui:=true

# 无头（服务器上跑，无计分板窗口）
ros2 launch tianracer_gazebo tianracer_race.launch.py gui:=false use_rviz:=false enable_display:=false

# 换赛道
ros2 launch tianracer_gazebo tianracer_race.launch.py world_name:=raicom
```

### 4.4 发车：点计分板上的「启动」

启动完成后**车不会自己动**。计分板窗口上有三个按钮：

| 按钮 | 作用 |
|---|---|
| **启动** | 开始比赛：先做模型完整性校验，通过后拉起竞速主程序并开始计时 |
| **刹车** | 终止本轮 |
| **重置** | 把车与比赛环境复位到出生状态（模型回位、重发初始位姿、清代价地图、重置计分） |

窗口初始提示是「**请点击启动按钮开始测评**」；点「启动」后状态变为「目标代码已启动」。

> 没有显示设备时用 `enable_display:=false`：裁判逻辑照常运行，只是没有窗口——
> 此时你需要从日志或话题看状态，见 [9](#9-无头--远程环境)。

### 4.5 看什么

- **Gazebo 3D 界面**（需 `gui:=true`）：车、赛道、锥体
- **RViz**（默认开）：激光、代价地图、全局/局部路径、检查门标记
- **计分板**：机器人名、赛道名、总分数、用时、状态
- **日志**：终端里各节点的输出；裁判的判罚与分数变化都会打印

---

## 5. 计分规则与判罚

理解规则才能看懂分数为什么是这个数。

### 5.1 总分 = 门分 + 速度分

```
总分 = 门分累计 + 速度分
门分：9 个门 → [5,5,5,5,5,5,5,5,10]（前 8 门各 5 分，末门 10 分），满分 50
速度分：35 × min(1, 42 / 总用时)      ← 跑进 42s 拿满 35，否则按 42/T 衰减
满分 = 50 + 35 = 85
总分保留 3 位小数（四舍五入）
```

**关键细节：速度分只在完赛结算时并入一次。** 比赛中和提前终止时，界面显示的是**纯门分**。
例如用时 90.06s 时界面显示 25（门分），最终才是 `25 + 35×42/90.06 ≈ 41.3`。

由此可得一个重要结论：**未完赛 = 速度分 0 分**，只剩已过门的门分。所以「跑得快但失败」
远不如「跑得慢但完赛」——失败一轮可能掉 20~40 分。

### 5.2 裁判的两条判罚（都会终止本轮）

| 判据 | 阈值 | 参数 | 含义 |
|---|---|---|---|
| **停车判罚** | 位移 < 1mm 持续 10s | `stop_time_threshold=10.0` | 车卡住不动（只要偶尔蠕动 ≥1mm 就重置计时） |
| **过门超时** | 30s 未通过下一个门 | `checkpoint_timeout=30.0` | 车在动但没有推进（绕圈/漂移） |

两条判据都会把成绩记为「已终止」，速度分不并入。

> 这两个值在 `judge.launch.py` 里是 launch 参数，可临时调大用于调试；但 `checkpoint_timeout`
> 在赛规里是**官方规则值**，改大就不是同一场比赛了。

### 5.3 模型完整性校验（重要）

点「启动」时裁判会先校验模型文件的 md5（清单见
`tianracer_gazebo/waypoint_race/official_model_hash.yaml`，共 40 条，覆盖 `urdf/` 与 `worlds/`）。

**如果你改动了这些文件**，校验不过，日志会打印：

```
Model Checking Unpassed, judge aborted
```

表现为：**车完全不动、里程计全零**——因为竞速主程序根本没被拉起。
这不是 bug，是防篡改设计。自行改过模型后如需跑比赛，要同步更新那份哈希清单。

---

## 6. 建图与导航

### 6.1 建图

三种 SLAM 任选（推荐 slam_toolbox）：

```bash
ros2 launch tianracer_slam slam_toolbox.launch.py    # 推荐
ros2 launch tianracer_slam cartographer.launch.py    # 需 cartographer_ros
ros2 launch tianracer_slam gmapping.launch.py
```

建图时另开一个终端看效果：

```bash
ros2 launch tianracer_rviz view_mapping.launch.py
```

> 要让车动起来才能建图。本仓库不含遥控节点（`tianracer_teleop` 已随实机内容移除），
> 可以用 Nav2 发目标点，或自行 `ros2 topic pub` 发布速度指令。

### 6.2 存图

```bash
ros2 launch tianracer_slam map_save.launch.py
```

存出来是一对文件：`xxx.pgm`（图像）+ `xxx.yaml`（分辨率与原点）。

**把它接入本项目**：放进 `tianracer_gazebo/maps/`，让 yaml 与 pgm 同名，
然后导航时用 `use_map:=xxx` 指定（见下）。

> ⚠️ `.gitignore` 里有 `*.pgm` 规则（本意是挡住建图产物）。本仓库自带的地图已通过
> 白名单放行；**你自己新加的地图需要 `git add -f`** 才能入库。

### 6.3 规划器变体

单独启动 Nav2：

```bash
ros2 launch tianracer_navigation2 nav2.launch.py
ros2 launch tianracer_navigation2 nav2.launch.py use_map:=raicom use_planner:=navfn_teb
ros2 launch tianracer_navigation2 nav2.launch.py map:=/绝对路径/my_map.yaml
```

| 参数 | 说明 |
|---|---|
| `use_map` | 地图名（对应 `tianracer_gazebo/maps/<名>.yaml`），默认 `tianracer_racetrack` |
| `map` | 直接给 yaml 的完整路径，**优先于** `use_map` |
| `use_planner` | 规划器/控制器组合，见下表，默认 `navfn_dwb` |
| `world` | 赛道名，用于自动选赛道专属参数档（见 [7.2](#72-赛道专属参数档怎么自动选中)） |
| `use_rviz` | 是否启动 RViz，默认 `true` |

`use_planner` 的全部取值（对应 `tianracer_navigation2/params/<名>_nav2_params.yaml`）：

| 值 | 规划器 + 控制器 |
|---|---|
| `navfn_dwb`（默认） | NavFn + DWB |
| `navfn_teb` | NavFn + TEB（需源码依赖，见 3.3） |
| `navfn_mppi` / `navfn_mppi_fwd` | NavFn + MPPI（后者允许倒车） |
| `smac_dwb` / `smac_graceful` | Smac + DWB / Graceful |
| `theta_star_dwb` / `theta_star_mppi` / `theta_star_rpp` / `theta_star_vector_pur` | Theta* + 各控制器 |

> 类车（阿克曼）不能原地旋转，因此各参数档都禁用了原地旋转；`footprint` 也按车体实际尺寸配置，
> 不要照搬差速车的圆形 `robot_radius`。

### 6.4 反应式竞速（不走 Nav2）

`tianracer_competition` 提供基于激光的 disparity extender：

```bash
ros2 launch tianracer_competition closed_map.launch.py speed_param:=3.5 P_param:=0.0
ros2 launch tianracer_competition open_map.launch.py   speed_param:=3.5 P_param:=0.0
```

`closed_map` 用于已知地图内，`open_map` 用于未知环境。

### 6.5 RViz 查看入口

```bash
ros2 launch tianracer_rviz view_lidar.launch.py     # 激光
ros2 launch tianracer_rviz view_imu.launch.py       # IMU
ros2 launch tianracer_rviz view_odom.launch.py      # 里程计
ros2 launch tianracer_rviz view_robot.launch.py     # 模型 / TF
ros2 launch tianracer_rviz view_image.launch.py     # 相机图像
ros2 launch tianracer_rviz view_mapping.launch.py   # 建图
```

---

## 7. 换赛道

### 7.1 内置赛道一览

**能直接竞速的只有 4 条**（竞速需要「世界 + 地图 + 路点 + 检查门」四件套齐全）：

| 赛道 | world | map | 路点/检查门 | 专属参数档 |
|---|---|---|---|---|
| `tianracer_racetrack` | ✅ | ✅ | ✅ | 无（用通用档） |
| `racetrack_1` | ✅ | ✅ | ✅ | ✅ `navfn_dwb_racetrack_1_...yaml` |
| `raicom` | ✅ | ✅ | ✅ | ✅ `navfn_dwb_raicom_...yaml` |
| `test_indoor` | ✅ | ✅ | ✅ | ✅ `navfn_dwb_test_indoor_...yaml` |
| `room_mini` | ✅ | ✅ | ❌ 缺路点 | — |
| `race_with_cones` | ✅ | ❌ 缺地图 | — | — |

`room_mini` 与 `race_with_cones` 只能用来做仿真演示，**不能跑竞速**
（前者缺 `waypoint_race/room_mini_points.yaml` 等路点文件，后者缺地图）。

换赛道就是加一个 `world_name:=` 参数：

```bash
ros2 launch tianracer_gazebo tianracer_race.launch.py world_name:=racetrack_1
```

### 7.2 赛道专属参数档怎么自动选中

`nav2.launch.py` 会按 `world` 去 `tianracer_navigation2/params/` 找
`<planner>_<world>_nav2_params.yaml`：找得到就用专属档，找不到回退通用档
`<planner>_nav2_params.yaml`，并在日志里说明用了哪个：

```
导航参数文件【赛道专属】/.../navfn_dwb_raicom_nav2_params.yaml
导航参数文件【通用】/.../navfn_dwb_nav2_params.yaml（无 navfn_dwb_xxx_nav2_params.yaml）
```

> ⚠️ **新建参数档后必须重新构建**，否则永远回退通用档，表现为「改了配置却不生效」。
> 原因是该函数查的是 `install/` 侧，而 `--symlink-install` 只为**构建时已存在**的文件建软链：
>
> ```bash
> colcon build --symlink-install --packages-select tianracer_navigation2
> ```
>
> 判据：看 launch 日志里是【赛道专属】还是【通用】。

### 7.3 自己加一条赛道

需要准备四样：

1. **世界文件** `tianracer_gazebo/worlds/<赛道>.world`
2. **地图** `tianracer_gazebo/maps/<赛道>.pgm` + `.yaml`（见 [6.2](#62-存图)）
3. **路点与检查门** `tianracer_gazebo/waypoint_race/<赛道>_points.yaml` 与
   `<赛道>_check_points.yaml`（可参照现有赛道的格式：检查门是**两点配对**成一条线段）
4. **出生位姿**：加到 `tianracer_gazebo/scripts/track_spawn.py` —— 它是出生点的**单一来源**，
   Gazebo 生成、Nav2 的 AMCL 初始位姿、裁判的重置三者共用它。
   改这一处，三处同时生效。

然后（可选）为这条赛道做一份专属参数档 `params/navfn_dwb_<赛道>_nav2_params.yaml` 并重新构建。

> ⚠️ 出生位姿必须实际测量确认，不要凭地图猜：z 要高于地板顶面（否则车生成在地板下），
> 朝向也不能简单取第一个路点的朝向。用 `view_robot` 看一眼再用。

---

## 8. 环境变量与多车

`tianracer_description` 为 `TIANRACER_BASE` 注入默认值（见 `tianracer_description/env-hooks/99.tianracer.dsv.in`）；
`TIANBOT_NAME` 由你自己设置（不设置即为空 = 无命名空间）：

| 变量 | 取值 | 作用 |
|---|---|---|
| `TIANRACER_BASE` | `compact`（默认）/ `standard` / `fullsize` | 车型：决定轴距取值 |
| `TIANBOT_NAME` | 任意 | 机器人命名空间（空 = 无命名空间） |

多车联调时给每台车设不同的命名空间：

```bash
TIANBOT_NAME=tianracer_01 ros2 launch tianracer_gazebo tianracer_race.launch.py
```

各 launch 也都有 `namespace:=` 参数，效果相同。

---

## 9. 无头 / 远程环境

Gazebo 的**渲染在 GUI 进程里做**，而激光/IMU/相机数据依赖渲染。所以在**没有 GPU/DRI 设备**的
机器上，可能出现「仿真起来了但传感器没有数据」。

两种办法：

1. 接显示器（或 X11 转发）运行；或
2. 强制软件渲染：

   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   export GALLIUM_DRIVER=llvmpipe
   ```

无界面运行比赛（服务器上）：

```bash
ros2 launch tianracer_gazebo tianracer_race.launch.py \
  gui:=false use_rviz:=false enable_display:=false
```

此时如何发车与看分：

- 比赛仍需触发 `/start`。没有计分板按钮时，可以自行调用裁判的 `start` 服务；
- 分数发布在 `/score_display` 话题上，可以 `ros2 topic echo /score_display` 观察；
- 也可以用 `ros2 launch tianracer_gazebo door_markers.launch.py` 把检查门画到 RViz，
  配合 `/score_display` 高亮看进程。

---

## 10. 排障速查

| 症状 | 可能原因 | 处置 |
|---|---|---|
| 启动失败 / gz 起不来 | 上一次仿真残留进程 | `pkill -9 -f "ign gazebo"` 后重试（注意是 `ign gazebo`） |
| 报 `does not export any plugins` | 用 Gazebo Sim 7 跑了 | 必须 v6：保留 `--force-version 6`；`ign gazebo --versions` 确认是 6.x |
| 车完全不动、里程计全零 | 模型 md5 校验未过（改过 `urdf/` 或 `worlds/`） | 看日志有无 `Model Checking Unpassed, judge aborted`；恢复原文件或更新哈希清单 |
| 点「启动」没反应 | 计分板没起来 / 无 `DISPLAY` | 有显示时确认 `enable_display:=true`；无显示时按 [9](#9-无头--远程环境) 自行触发 `/start` |
| 仿真起来但激光/IMU 无数据 | 无 GPU/渲染不可用 | 见 [9](#9-无头--远程环境)，加软件渲染环境变量 |
| 改了参数档却不生效 | 新档未构建（查的是 install 侧） | `colcon build --symlink-install --packages-select tianracer_navigation2` |
| 车在窄处卡住不动 | 代价地图上 footprint 贴上致命格 | 调 `inflation_radius`（global 侧）；不要缩小 footprint 硬挤 |
| 速度上不去 | `max_vel_x` 与 `velocity_smoother` 的上限不一致 | 两处必须成组改（改一处会被平滑器夹回） |
| `vcs: command not found` | 未装 `python3-vcstool` | `sudo apt install python3-vcstool`（仅 TEB 路径需要，见 [3.3](#33-可选teb-规划器)） |
| `costmap_converter` 报 `abstract class type 'BlobDetector'` | 机器上装了两套 OpenCV | 加 `-DOpenCV_DIR=/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)/cmake/opencv4`（见 [3.3](#33-可选teb-规划器)） |
| 计分板分数看着「不对」 | 速度分只在完赛结算时并入 | 比赛中显示的是纯门分，属预期行为（见 [5.1](#51-总分--门分--速度分)） |

---

## 11. 本仓库不含测试工具链

上游与本项目的开发过程中曾有一套**参数调优 / 轮次测试工具链**（批量跑轮次、记录每个控制周期
的全部候选轨迹、判废轮、地图↔世界配准等）。为保持公开仓库精简，**这些工具未随本仓库发布**。

这不影响跑仿真竞速——它们只用于「改进参数」这件事。参数档（`params/*.yaml`）只描述
各参数的取值与含义，不含调参记录；想改进参数请自行搭建测试链，并至少记录：

- 每轮的完赛情况与用时、车的位置轨迹、卡住时的位置与当时的控制指令——只有这样才分得清
  「配置问题」和「随机性」；
- 每次只改一组相关联的参数。例如线速度上限同时受 `FollowPath.max_vel_x` 与
  `velocity_smoother` 的 `max_velocity[0]` 约束（后者在控制链路上、是硬限），只改一处会出现
  「规划器以为能跑 X、实际被钳到 Y」；角速度上限要随线速度按 `ω = v·κ` 同抬。

---

## 12. 许可证与出处

本项目以 **GNU General Public License v3.0** 发布，全文见 [`LICENSE`](LICENSE)。

它是上游 [tianbot/tianracer](https://github.com/tianbot/tianracer) 的**衍生作品**：上游以
GPL-3.0 发布，依该许可证的传染性条款，衍生作品必须同样以 GPL-3.0 分发。上游版权归其原作者
（天之博特/Tianbot）所有，上游的许可证声明均已保留——完整归属声明（含上游包原有的 MIT 声明）
见 [`NOTICE`](NOTICE)。

上游项目本身基于 [HyphaROS RaceCar](https://github.com/Hypha-ROS/hypharos_racecar) 开发，
在此致谢其作者 HaoChih LIN、KaiChun Wu。
