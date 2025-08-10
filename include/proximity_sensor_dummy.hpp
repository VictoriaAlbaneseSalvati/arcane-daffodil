////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 9, 2025
// Filename: proximity_sensor_dummy.hpp
//
////////////////////////////////////////////////////////////////////////////////

#ifndef PROXIMITY_SENSOR_DUMMY_H
#define PROXIMITY_SENSOR_DUMMY_H

#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief This DummyProximitySensor publishes LaserScan messages every 0.5 
 *        seconds.  The sensor has 180* range, and emits 5 beams at even 
 *        intervals (so at 45* increments).  The sensor has a range between 
 *        0.1 meters and 2 meters.
 *     
 *        It's setup such that each beam reads the same value, and that value 
 *        bounces back and forth continuously between the min and max sensor 
 *        values at increments of 0.1 meters.  Just a simple dummy pattern.
 */
class DummyProximitySensor : public rclcpp::Node
{
public:

    DummyProximitySensor();

private:

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<LaserScan>::SharedPtr publisher;
    std::vector<float> laser_range_pattern;
    size_t count;

    const int NUM_BEAMS = 5;
    const float ANGLE_MIN_RAD = -M_PI / 2.0; 
    const float ANGLE_MAX_RAD = M_PI / 2.0; 
    const float ANGLE_INCREMENT_RAD = M_PI / this->NUM_BEAMS; 
    const float TIME_INCREMENT_SEC = 0.0001; 
    const float SCAN_TIME_SEC = 0.5; 
    const float RANGE_MIN_METERS = 0.1; 
    const float RANGE_MAX_METERS = 2.0; 
    
    std::vector<float> init_dummy_laser_range_pattern();
    LaserScan init_dummy_LaserScan();
    void publish_LaserScan();
};

////////////////////////////////////////////////////////////////////////////////

#endif // PROXIMITY_SENSOR_DUMMY_H
