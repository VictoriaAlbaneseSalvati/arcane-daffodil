////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 10, 2025
// Filename: speed_controller.hpp
//
////////////////////////////////////////////////////////////////////////////////

#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief The speed controller does the following:
 * 
 *        (1) Subscribes to a LaserScan topic and picks out the closest 
 *            sensor reading so we can know how close the nearest obstacle 
 *            is at any given time. 
 *        (2) Logs the closest sensor reading.
 */
class SpeedController : public rclcpp::Node
{
public:

    SpeedController();
    
private:

    rclcpp::Subscription<LaserScan>::SharedPtr subscription;
    float min_laser_dist_meters;

    void callback_LaserScan(const LaserScan & msg);
};

////////////////////////////////////////////////////////////////////////////////

#endif // SPEED_CONTROLLER_H
