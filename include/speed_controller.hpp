////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 10, 2025
// Filename: speed_controller.hpp
//
////////////////////////////////////////////////////////////////////////////////

#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

using BoolMsg = std_msgs::msg::Bool;
using CommandVelocity = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;

enum SpeedState 
{
    FULL_SPEED,
    SLOW,
    STOP
};

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief The speed controller does the following:
 * 
 *        (1) Subscribes to a /robot/base_scan topic and picks out the closest 
 *            sensor reading so we can know how close the nearest obstacle 
 *            is at any given time. 
 *        (2) Subscribes to a /estop topic and listens to if the estop is on 
 *        (3) Based on whether or not the estop is on and how close the nearest 
 *            obstacle is, set the speed state to FULL_SPEED, SLOW, or STOP 
 *            based on the requirements.
 *        (3) Based on the speed state, publish a command velocity to set
 *            the speed of the robot.
 */
class SpeedController : public rclcpp::Node
{
public:

    SpeedController(
        const bool& silence_logging = false,
        const bool& is_estop_on = false,
        const float& min_laser_dist_meters = 2.0);

    bool silence_logging;
 
    bool get_is_estop_on() const ;
    void set_is_estop_on(const bool& new_is_estop_on);
    
    float get_min_laser_dist_meters() const; 
    void set_min_laser_dist_meters(const float& new_min_laser_dist_meters); 

    SpeedState get_speed_state() const; 
    void set_speed_state(const SpeedState& new_speed_state); 
   
private:

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<CommandVelocity>::SharedPtr publisher;
    rclcpp::Subscription<LaserScan>::SharedPtr laser_subscription;
    rclcpp::Subscription<BoolMsg>::SharedPtr estop_subscription;
    float min_laser_dist_meters;
    bool is_estop_on; 
    SpeedState speed_state;

    const float CLOSE_LASER_DIST_METERS = 0.8;
    const float TOO_CLOSE_LASER_DIST_METERS = 0.4;
    const float FULL_SPEED_MPS = 5.0;
    const float SLOW_SPEED_MPS = 2.0;
    const float STOP_SPEED_MPS = 0.0;

    void determine_speed_state();
    void publish_CommandVelocity();
    void callback_EStop(const BoolMsg& msg);
    void callback_LaserScan(const LaserScan& msg);
};

////////////////////////////////////////////////////////////////////////////////

#endif // SPEED_CONTROLLER_H
