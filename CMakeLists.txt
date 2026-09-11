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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
ament_python_install_package(m20_adapter)
install(PROGRAMS scripts/m20_bridge scripts/m20_mock_inputs DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/m20_cloud_probe.py scripts/m20_status_probe.py scripts/m20_select_target.py scripts/m20_enable_follow.py
  DESTINATION lib/${PROJECT_NAME})

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
  std_srvs
  rcl_interfaces
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

# 安装可执行文件
install(TARGETS
  robot_nexus
  keyboard_cmd
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# 安装web静态文件
install(DIRECTORY
  web
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME m20_python_core COMMAND ${Python3_EXECUTABLE} -m unittest
    discover -s ${CMAKE_CURRENT_SOURCE_DIR}/test -p test_m20_*.py -v)
  add_test(NAME m20_ros_bridge_contract COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/ros_m20_bridge_contract.py)
  add_test(NAME m20_ros_smoke COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/ros_m20_smoke.py)
  set_tests_properties(m20_python_core m20_ros_bridge_contract m20_ros_smoke PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}"
    TIMEOUT 90 RUN_SERIAL TRUE)
  add_executable(m20_scan_replay test/m20_scan_replay.cpp)
  ament_target_dependencies(m20_scan_replay sensor_msgs geometry_msgs)
  target_include_directories(m20_scan_replay PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(m20_scan_replay ${OpenCV_LIBS} Threads::Threads)
  set(M20_UPSTREAM_INCLUDE "" CACHE PATH "Offline normalized upstream headers only")
  if(M20_UPSTREAM_INCLUDE)
    add_executable(m20_scan_replay_upstream test/m20_scan_replay.cpp)
    target_compile_definitions(m20_scan_replay_upstream PRIVATE UPSTREAM_BASELINE)
    target_include_directories(m20_scan_replay_upstream BEFORE PRIVATE ${M20_UPSTREAM_INCLUDE})
    ament_target_dependencies(m20_scan_replay_upstream sensor_msgs geometry_msgs)
    target_include_directories(m20_scan_replay_upstream PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(m20_scan_replay_upstream ${OpenCV_LIBS} Threads::Threads)
  endif()
  add_executable(m20_follow_scenario test/m20_follow_scenario.cpp)
  ament_target_dependencies(m20_follow_scenario sensor_msgs geometry_msgs)
  target_include_directories(m20_follow_scenario PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(m20_follow_scenario ${OpenCV_LIBS} Threads::Threads)
  foreach(distance IN ITEMS 2.0 1.2)
    add_test(NAME m20_follow_offline_${distance}
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/m20_follow_offline.py
        --tracker $<TARGET_FILE:m20_follow_scenario> --follow-distance ${distance}
        --output ${CMAKE_CURRENT_BINARY_DIR}/offline-${distance})
  endforeach()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_m20_node test/test_m20_node.cpp src/web_comm.cpp src/android_comm.cpp)
  ament_target_dependencies(test_m20_node rclcpp sensor_msgs geometry_msgs std_msgs std_srvs rcl_interfaces)
  target_include_directories(test_m20_node PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_m20_node ${OpenCV_LIBS} Threads::Threads)
  set_tests_properties(test_m20_node PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE)
  ament_add_gtest(test_m20_tracker test/test_m20_tracker.cpp)
  ament_target_dependencies(test_m20_tracker sensor_msgs geometry_msgs)
  target_include_directories(test_m20_tracker PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_m20_tracker ${OpenCV_LIBS} Threads::Threads)
  set_property(TEST m20_python_core m20_ros_bridge_contract m20_ros_smoke
    m20_follow_offline_2.0 m20_follow_offline_1.2 test_m20_node test_m20_tracker
    APPEND PROPERTY LABELS m20_functional)
  find_package(ament_lint_auto REQUIRED)
  # Vendored httplib is C++, not C; do not reformat third-party source.
  set(ament_cmake_cppcheck_LANGUAGE "c++")
  set(ament_cmake_uncrustify_ADDITIONAL_ARGS LANGUAGE "C++"
    EXCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/include/httplib.h")
  set(ament_cmake_cpplint_ADDITIONAL_EXCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/include/httplib.h")
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
