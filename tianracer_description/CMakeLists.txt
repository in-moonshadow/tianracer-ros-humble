cmake_minimum_required(VERSION 3.5)
project(tianracer_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
)

install(DIRECTORY urdf launch param meshes
  DESTINATION share/${PROJECT_NAME}
)

# 车辆几何读取工具（唯一真理源 param/vehicle_geometry.yaml），供各包 launch 复用
ament_python_install_package(${PROJECT_NAME})

# 仿真栈默认环境变量（TIANRACER_BASE）。
# 原属 tianracer_bringup，该包删除后迁入本包 —— 本包是这些变量的消费者。
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/99.tianracer.dsv.in"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
