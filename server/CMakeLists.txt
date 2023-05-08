cmake_minimum_required(VERSION 3.5)
project(rosweb)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories( ${Boost_INCLUDE_DIRS} include )
file(GLOB SOURCES "src/*.cpp")

add_executable(server ${SOURCES} )
ament_target_dependencies(server rclcpp sensor_msgs cv_bridge OpenCV)