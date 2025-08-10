cmake_minimum_required(VERSION 3.8)
project(arcane-daffodil)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(demo
    src/main.cpp
    src/speed_controller.cpp
    src/proximity_sensor_dummy.cpp)
target_link_libraries(demo PUBLIC 
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS} sensor_msgs::sensor_msgs_library)
install(TARGETS demo DESTINATION lib/${PROJECT_NAME})

ament_package()
