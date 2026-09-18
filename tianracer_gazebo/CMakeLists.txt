cmake_minimum_required(VERSION 3.8)
project(tianracer_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/initialpose_pub.py
  scripts/servo_commands.py
  scripts/transform.py
  scripts/f1tenth_racer.py
  scripts/judge_system.py
  scripts/judge_system_node.py
  scripts/judge_display.py
  scripts/door_markers.py
  scripts/md5_check.py
  scripts/movebase_goal_cancel.py
  scripts/position_check.py
  DESTINATION lib/${PROJECT_NAME}
)

# waypoint_race 子包（utils.py 被 f1tenth_racer / position_check 以 `import waypoint_race.utils` 引用，
# 需与可执行脚本同目录可见；安装到 lib/<pkg>/waypoint_race/）
install(DIRECTORY scripts/waypoint_race
  DESTINATION lib/${PROJECT_NAME}
)

# track_spawn.py：各赛道出生位姿的单一来源，被 launch（经 sys.path 注入 lib/<pkg>）
# 与 judge_system.py 共同 import，故按模块安装（FILES，不加可执行位）。
install(FILES scripts/track_spawn.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf config launch worlds maps rviz waypoint_race
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
