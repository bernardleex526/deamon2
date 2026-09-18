cmake_minimum_required(VERSION 3.8)
project(jie_deamon)

# 设置C++17标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# 包含目录
include_directories(include)

# 编译robot_nexus节点（模块化版本）
add_executable(robot_nexus
  src/robot_nexus.cpp
  src/web_comm.cpp
  src/android_comm.cpp
)
ament_target_dependencies(robot_nexus
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)
target_link_libraries(robot_nexus
  ${OpenCV_LIBS}
  Threads::Threads
)
target_include_directories(robot_nexus PUBLIC ${OpenCV_INCLUDE_DIRS})

# 编译keyboard_cmd节点
add_executable(keyboard_cmd src/keyboard_cmd.cpp)
ament_target_dependencies(keyboard_cmd
"rclcpp" "geometry_msgs"
)
# NEW-06: 上游keyboard_cmd.cpp使用STDIN_FILENO但未包含<unistd.h>（该文件不在本次授权修改清单，
# 故在构建层强制包含修复Foxy工具链下的编译错误；不改动上游源码）
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(keyboard_cmd PRIVATE -include unistd.h)
endif()

# 安装可执行文件
install(TARGETS
  robot_nexus
  keyboard_cmd
  DESTINATION lib/${PROJECT_NAME}
)

# NEW-06: 安装NavCmd预览转换脚本（仅算法预览）
install(PROGRAMS
  scripts/nav_cmd_preview.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装web静态文件
install(DIRECTORY
  web
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
