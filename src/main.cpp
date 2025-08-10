////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 9, 2025
// Filename: main.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "proximity_sensor_dummy.hpp"
#include "speed_controller.hpp"

using NodePtr = rclcpp::Node::SharedPtr;

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    NodePtr sensor_node = std::make_shared<DummyProximitySensor>();
    NodePtr speed_controller_node = std::make_shared<SpeedController>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(sensor_node);
    executor.add_node(speed_controller_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
