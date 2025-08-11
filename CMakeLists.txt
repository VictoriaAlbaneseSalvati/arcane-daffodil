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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(demo
    src/main.cpp
    src/speed_controller.cpp)
target_link_libraries(demo PUBLIC 
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS} sensor_msgs::sensor_msgs_library
    ${std_msgs_TARGETS})
install(TARGETS demo DESTINATION lib/${PROJECT_NAME})


add_executable(run_proximity_sensor 
    src/run_proximity_sensor_keyboard.cpp
    src/proximity_sensor_keyboard.cpp
    src/keyboard_input_processor.cpp)
target_link_libraries(run_proximity_sensor PUBLIC 
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS} sensor_msgs::sensor_msgs_library)
install(TARGETS run_proximity_sensor DESTINATION lib/${PROJECT_NAME})


add_executable(run_estop 
    src/run_estop.cpp
    src/estop.cpp
    src/keyboard_input_processor.cpp)
target_link_libraries(run_estop PUBLIC 
    rclcpp::rclcpp
    ${std_msgs_TARGETS})
install(TARGETS run_estop DESTINATION lib/${PROJECT_NAME})


add_executable(keyboard_demo 
    src/keyboard_input_processor_demo.cpp
    src/keyboard_input_processor.cpp)
install(TARGETS keyboard_demo DESTINATION lib/${PROJECT_NAME})

ament_package()
