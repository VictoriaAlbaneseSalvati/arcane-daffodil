////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 10, 2025
// Filename: proximity_sensor_base.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <chrono>
#include <functional>

#include "proximity_sensor_keyboard.hpp"

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Constructs an instance of the ProximitySensor class
 */
ProximitySensor::ProximitySensor()
: Node("proximity_sensor")
{
    this->timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ProximitySensor::publish_LaserScan, this));
    this->publisher = this->create_publisher<LaserScan>("robot/base_scan", 10);
}


/**
 * @brief Creates a dummy LaserScan message with values in accordance with
 *        this sensor, as defined in the header file.  Only basic parameters
 *        are filled in, not the actual sensor ranges values.
 * @returns a partially initialized LaserScan message 
 */
LaserScan
ProximitySensor::init_LaserScan_params() const
{
    LaserScan laser_scan;
    laser_scan.angle_min = this->ANGLE_MIN_RAD;
    laser_scan.angle_max = this->ANGLE_MAX_RAD;
    laser_scan.angle_increment = this->ANGLE_INCREMENT_RAD; 
    laser_scan.time_increment = this->TIME_INCREMENT_SEC;
    laser_scan.scan_time = this->SCAN_TIME_SEC;
    laser_scan.range_min = this->RANGE_MIN_METERS;
    laser_scan.range_max = this->RANGE_MAX_METERS;
    laser_scan.ranges.resize(this->NUM_BEAMS);        
    return laser_scan;
}


/**
 * @brief Creates and publishes a LaserScan messages
 */
void 
ProximitySensor::publish_LaserScan() 
{
    LaserScan laser_scan = this->init_LaserScan();
    this->publisher->publish(laser_scan);
}

////////////////////////////////////////////////////////////////////////////////
