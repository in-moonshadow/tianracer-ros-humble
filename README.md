# Tianracer — ROS 2 Humble

[中文版说明](README_CN.md) | [详细中文教程](TUTORIAL_CN.md)

> **This repository is the ROS 2 Humble version of the upstream project
> [tianracer](https://github.com/tianbot/tianracer)** — the Tianracer autonomous racing car
> platform by [Tianbot](https://github.com/tianbot). The upstream project targets **ROS 1**;
> this project is its **ROS 2 Humble** port and continued development, built on the upstream
> `humble-devel` branch. See [NOTICE](NOTICE) for attribution and licensing.

Tianracer is a low-cost autonomous racing car platform. This workspace contains a
**self-contained simulation stack** for ROS 2 Humble: Gazebo Sim world, chassis control chain,
SLAM, Nav2 navigation, and a race judge.

## Packages

| Package | Description |
|---|---|
| `tianracer` | Metapackage for the Tianracer racecar |
| `tianracer_description` | URDF/xacro robot models and TF trees |
| `tianracer_gazebo` | Simulation stack: Gazebo Sim 6 (Fortress) worlds, chassis controller, odometry bridge, race judge |
| `tianracer_navigation2` | Nav2 configuration and launch (multiple planner/controller parameter sets, per-track auto-selection, `cmd_vel` → `ackermann_cmd` conversion) |
| `tianracer_slam` | SLAM launches: slam_toolbox, Cartographer, GMapping; map saving |
| `tianracer_rviz` | RViz configurations and view launches (lidar, IMU, odom, image, robot, mapping) |
| `tianracer_vision` | Vision package (line-following node) |
| `tianracer_competition` | Racing algorithms (disparity extender) and waypoint navigation |

## Requirements

- Ubuntu 22.04 with **ROS 2 Humble** (colcon / ament)
- **Gazebo Sim 6 (Fortress)** with `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`
- `ros2_control` / `ros2_controllers`, Nav2, `slam_toolbox`, `ackermann_msgs`
- Optional: Cartographer (`cartographer_ros`), TEB (`teb_local_planner` + `costmap_converter`)

Gazebo Classic is **not** used by this project. The simulation runs on **Gazebo Sim 6
(Fortress)** through `ros_gz`, and is launched explicitly with `ign gazebo --force-version 6`:
`gz_ros2_control` does not export plugins for Gazebo Sim 7 (Garden), so a v7 runtime fails with
`does not export any plugins`. On a machine with both versions installed `gz sim` defaults to v7,
which is why the version is pinned.

## Build

The packages live at the repository root, so the repository itself is the colcon workspace:

```bash
git clone https://github.com/in-moonshadow/tianracer-ros-humble
cd tianracer-ros-humble
rosdep install --from-paths . --ignore-src -r -y   # installs the apt-resolvable dependencies
colcon build --symlink-install
source install/setup.bash
```

TEB is optional and has no apt package in Humble. If you want the TEB variants, pull the
source dependencies first:

```bash
vcs import src < tianracer.repos
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
```

The default navigation stack is `navfn_dwb`, which needs **none** of these extra sources.

## Run

### One-click race (simulation + control chain + Nav2 + judge)

```bash
ros2 launch tianracer_gazebo tianracer_race.launch.py
```

Useful arguments (defaults in parentheses):

| Argument | Meaning |
|---|---|
| `world_name` (`tianracer_racetrack`) | Track name. Also selects the map: `<world_name>.yaml` and the per-track Nav2 parameter file |
| `gui` (`false`) | Start the Gazebo Sim 3D window (off by default to save CPU) |
| `use_rviz` (`true`) | Start RViz |
| `use_planner` (`navfn_dwb`) | Planner/controller combination |
| `enable_display` (`true`) | Show the judge scoreboard (tkinter window, requires `DISPLAY`) |
| `lap_count` (`3`) | Number of laps to complete |
| `checkpoint_timeout` (`30.0`) | Seconds allowed per checkpoint |

The race is started from the judge scoreboard window once everything is up.

### Simulation only

```bash
ros2 launch tianracer_gazebo tianracer_on_racetrack.launch.py gui:=true world:=tianracer_racetrack.world
```

> **Before launching:** a leftover `gz sim` process from a previous run will break the
> next start — kill it first (`pkill -9 -f "ign gazebo"`; the process name is
> `ign gazebo`, not `gz sim`). Cleanup is deliberately **not** done inside the launch
> file: it would race with, and kill, the very server the launch is starting.

### Mapping

```bash
ros2 launch tianracer_slam slam_toolbox.launch.py     # recommended
ros2 launch tianracer_slam cartographer.launch.py
ros2 launch tianracer_slam gmapping.launch.py
ros2 launch tianracer_rviz view_mapping.launch.py     # view while mapping

ros2 launch tianracer_slam map_save.launch.py         # save the map
```

### Navigation

```bash
ros2 launch tianracer_navigation2 nav2.launch.py
ros2 launch tianracer_navigation2 nav2.launch.py world:=raicom use_planner:=navfn_teb
```

Available `use_planner` values correspond to the parameter files in
`tianracer_navigation2/params/`: `navfn_dwb` (default), `navfn_teb`, `navfn_mppi`,
`navfn_mppi_fwd`, `smac_dwb`, `smac_graceful`, `theta_star_dwb`, `theta_star_mppi`,
`theta_star_rpp`, `theta_star_vector_pur`.

Per-track parameter files (`navfn_dwb_<track>_nav2_params.yaml`) are selected automatically from
`world:=`; the generic file is used as a fallback.

### Reactive racing (disparity extender)

```bash
ros2 launch tianracer_competition closed_map.launch.py speed_param:=3.5 P_param:=0.0
ros2 launch tianracer_competition open_map.launch.py   speed_param:=3.5 P_param:=0.0
```

## Environment variables

Injected by `tianracer_description` (see `env-hooks/99.tianracer.dsv.in`):

| Variable | Values | Effect |
|---|---|---|
| `TIANRACER_BASE` | `compact` (default), `standard`, `fullsize` | Vehicle model / TF branch and wheelbase |
| `TIANRACER_LIDAR` | `richbeam` (default), `rplidar_a1`, `velodyne`, `rslidar` | Lidar/IMU static transform branch |
| `TIANBOT_NAME` | any | Robot namespace (empty = none) |

## Scope: simulation only

This workspace has been **stripped of all real-hardware content**. The packages that drove the
physical car — `tianracer_bringup` (hardware bringup, lidar/camera/GPS/joystick launches,
udev/systemd deployment), `tianracer_core` (TianBoard Mini serial chassis driver + EKF),
`tianracer_gps`, `tianracer_teleop`, `tianracer_jetson` and `tianracer_test` — have been removed.
To deploy on a real car, restore them from the upstream repository
[tianbot/tianracer](https://github.com/tianbot/tianracer).

### Running headless

Gazebo Sim renders in a GUI process; on a machine without a GPU/DRI device, sensor data
(lidar, IMU, camera) may not be produced. Either run with a display, or force software
rendering:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
```

## License

This project is distributed under the **GNU General Public License v3.0** — see
[`LICENSE`](LICENSE).

It is a **derivative work** of [tianbot/tianracer](https://github.com/tianbot/tianracer), which is
released under GPL-3.0; a derivative work must therefore be distributed under the same license.
Upstream copyright remains with the original authors (Tianbot), and upstream license notices are
preserved — see [`NOTICE`](NOTICE) for the full attribution statement, including the MIT notice
of the upstream `tianracer_navigation2` package.

Upstream itself was developed based on
[HyphaROS RaceCar](https://github.com/Hypha-ROS/hypharos_racecar) — thanks to its authors.
