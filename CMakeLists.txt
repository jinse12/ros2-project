cmake_minimum_required(VERSION 3.8)
project(imu_turtle_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2 REQUIRED)

set(DEPS rclcpp sensor_msgs geometry_msgs std_msgs turtlesim tf2)

foreach(target imu_filter_node obstacle_avoider sim_imu_publisher)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} ${DEPS})
endforeach()

install(TARGETS
  imu_filter_node
  obstacle_avoider
  sim_imu_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/serial_imu_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME serial_imu_bridge
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
