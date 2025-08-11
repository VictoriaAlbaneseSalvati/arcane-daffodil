////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 10, 2025
// Filename: proximity_sensor_base.hpp
//
////////////////////////////////////////////////////////////////////////////////

#ifndef PROXIMITY_SENSOR_BASE_H
#define PROXIMITY_SENSOR_BASE_H

#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief This KeyboardProximitySensor publishes LaserScan messages every 0.5 
 *        seconds.  The sensor has 180* range, and emits 5 beams at even 
 *        intervals (so at 45* increments).  The sensor has a range between 
 *        0.1 meters and 2 meters.
 *
 *        The exact method for filling out the sensor ranges is not determined
 *        in this base class, only by the derived classes.
 */
class ProximitySensor : public rclcpp::Node
{
public:

    ProximitySensor();

protected:

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<LaserScan>::SharedPtr publisher;

    const int NUM_BEAMS = 5;
    const float ANGLE_MIN_RAD = -M_PI / 2.0; 
    const float ANGLE_MAX_RAD = M_PI / 2.0; 
    const float ANGLE_INCREMENT_RAD = M_PI / this->NUM_BEAMS; 
    const float TIME_INCREMENT_SEC = 0.0001; 
    const float SCAN_TIME_SEC = 0.5; 
    const float RANGE_MIN_METERS = 0.1; 
    const float RANGE_MAX_METERS = 2.0; 

    // This will be added to the laser ranges that are being manually set, 
    // and which are meant to be even increments of 0.1; however, because of
    // floating point precision, sometimes the values are slightly less than
    // the value they are meant to be, which can make the visualization 
    // misleading since that has exact values.  So it's better to have the
    // laserscan values be slightly bigger than the exact value that smaller.
    const float EPSILON = 0.000001;

    LaserScan init_LaserScan_params() const;
    virtual LaserScan init_LaserScan() = 0;
    void publish_LaserScan();
};

////////////////////////////////////////////////////////////////////////////////

#endif // PROXIMITY_SENSOR_BASE_H
