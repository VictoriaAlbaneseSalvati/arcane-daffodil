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

using CommandVelocity = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief The speed controller does the following:
 * 
 *        (1) Subscribes to a LaserScan topic and picks out the closest 
 *            sensor reading so we can know how close the nearest obstacle 
 *            is at any given time. 
 *        (2) Based on how close the nearest obstacle is, set the speed state
 *            to FULL_SPEED, SLOW, or STOP based on the requirements.
 *        (3) Based on the speed state, publish a command velocity to set
 *            the speed of the robot.
 */
class SpeedController : public rclcpp::Node
{
public:

    SpeedController();

    enum SpeedState 
    {
        FULL_SPEED,
        SLOW,
        STOP
    };
    std::unordered_map<SpeedState, std::string> speed_state_to_str = {
        { SpeedState::FULL_SPEED, "FULL_SPEED" },
        { SpeedState::SLOW, "SLOW" },
        { SpeedState::STOP, "STOP" },
    };
    
private:

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<CommandVelocity>::SharedPtr publisher;
    rclcpp::Subscription<LaserScan>::SharedPtr subscription;
    float min_laser_dist_meters;
    SpeedState speed_state;

    const float CLOSE_LASER_DIST_METERS = 0.8;
    const float TOO_CLOSE_LASER_DIST_METERS = 0.4;
    const float FULL_SPEED_MPS = 5.0;
    const float SLOW_SPEED_MPS = 2.0;
    const float STOP_SPEED_MPS = 0.0;

    void determine_speed_state();
    void publish_CommandVelocity();
    void callback_LaserScan(const LaserScan & msg);
};

////////////////////////////////////////////////////////////////////////////////

#endif // SPEED_CONTROLLER_H
