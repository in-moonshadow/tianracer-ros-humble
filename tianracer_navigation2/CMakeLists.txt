cmake_minimum_required(VERSION 3.8)
project(tianracer_navigation2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch params behavior_trees tools
DESTINATION share/${PROJECT_NAME})

# cmd_vel -> ackermann_cmd 转换（真机路径，按车型 wheelbase 算 δ=atan(L·ω/v)）
# 由 navigation_launch.py 启动，仿真/导航链自持
install(PROGRAMS
  script/cmd_vel_to_ackermann_drive.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
