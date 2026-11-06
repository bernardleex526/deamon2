cmake_minimum_required(VERSION 3.16)
project(m20_follow_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(m20_follow_core SHARED
  src/follow_controller.cpp
  src/m20_protocol.cpp
  src/pointcloud_wire.cpp
)
target_include_directories(m20_follow_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(m20_follow_node src/follow_node.cpp)
target_include_directories(m20_follow_node PRIVATE include)
ament_target_dependencies(m20_follow_node rclcpp sensor_msgs geometry_msgs std_msgs)
target_link_libraries(m20_follow_node m20_follow_core Threads::Threads)

add_executable(m20_udp_bridge src/m20_udp_bridge_node.cpp)
target_include_directories(m20_udp_bridge PRIVATE include)
ament_target_dependencies(m20_udp_bridge rclcpp geometry_msgs std_msgs)
target_link_libraries(m20_udp_bridge m20_follow_core Threads::Threads)

install(TARGETS
  m20_follow_core
  m20_follow_node
  m20_udp_bridge
  EXPORT export_m20_follow_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_follow_controller test/test_follow_controller.cpp)
  if(TARGET test_follow_controller)
    target_include_directories(test_follow_controller PRIVATE include)
    target_link_libraries(test_follow_controller m20_follow_core)
  endif()
  ament_add_gtest(test_m20_protocol test/test_m20_protocol.cpp)
  if(TARGET test_m20_protocol)
    target_include_directories(test_m20_protocol PRIVATE include)
    target_link_libraries(test_m20_protocol m20_follow_core)
  endif()
  ament_add_gtest(test_pointcloud_wire test/test_pointcloud_wire.cpp)
  if(TARGET test_pointcloud_wire)
    target_include_directories(test_pointcloud_wire PRIVATE include)
    target_link_libraries(test_pointcloud_wire m20_follow_core)
  endif()
endif()

ament_export_targets(export_m20_follow_control HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(m20_follow_core)
ament_package()
