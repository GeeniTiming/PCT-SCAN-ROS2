cmake_minimum_required(VERSION 3.16)
project(foxy_subscription_smoke LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(foxy_subscription_smoke subscription_callbacks.cpp)
target_compile_features(foxy_subscription_smoke PRIVATE cxx_std_17)
ament_target_dependencies(
  foxy_subscription_smoke
  nav_msgs
  rclcpp
  sensor_msgs
)
