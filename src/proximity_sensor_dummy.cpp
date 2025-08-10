////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 9, 2025
// Filename: proximity_sensor_dummy.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include <chrono>
#include <functional>

#include "proximity_sensor_dummy.hpp"

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Constructs an instance of the DummyProximitySensor class
 */
DummyProximitySensor::DummyProximitySensor()
: Node("proximity_sensor_dummy")
, count(0)
{
    this->timer = this->create_wall_timer(
        std::chrono::milliseconds((int)(this->SCAN_TIME_SEC * 1000)),
        std::bind(&DummyProximitySensor::publish_LaserScan, this));
    this->publisher = this->create_publisher<LaserScan>("robot/base_scan", 10);
    this->laser_range_pattern = this->init_dummy_laser_range_pattern();
}


/**
 * @brief Creates a vector of floats which are meant to represent dummy
 *        values for the LaserScan range sensors. The values in the vector 
          start at the range_max sensor value, decrease until we get to the
          range_min sensor value, and then we climb back up to 2.0, all in 
          increments of 0.1.  
 * @returns Vector of floats representing dummy values for LaserScan
 */
std::vector<float> 
DummyProximitySensor::init_dummy_laser_range_pattern() 
{
    std::vector<float> dummy_pattern;
    for (int i = 20; i > 0; i--) 
    {
        dummy_pattern.push_back(i / 10.0);
    }
    for (int i = 1; i <= 20; i++) 
    {
        dummy_pattern.push_back(i / 10.0);
    }
    return dummy_pattern;    
} 


/**
 * @brief Creates a dummy LaserScan message with values in accordance with
 *        this sensor, as defined in the header file.
 * @returns an initialized dummy LaserScan message 
 */
LaserScan
DummyProximitySensor::init_dummy_LaserScan()
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
    
    // We are going to fill the ranges with uniform values for simplicity
    // Each value is going to be determined by the count modulo the size of 
    // the dummy laser range pattern, this way we iterate over the dummy
    // pattern continuously.  The pattern is meant to represent an obstacle
    // getting closer and closer and then further and further away.
    int laser_range_pattern_idx = count % (this->laser_range_pattern.size() - 1);
    for (int i = 0; i < this->NUM_BEAMS; i++) 
    {
        laser_scan.ranges[i] = this->laser_range_pattern[laser_range_pattern_idx];
    }
    this->count++;
   
    return laser_scan;
}


/**
 * @brief Creates and publishes dummy LaserScan messages
 */
void 
DummyProximitySensor::publish_LaserScan()
{
    LaserScan laser_scan = this->init_dummy_LaserScan();
    this->publisher->publish(laser_scan);
}

////////////////////////////////////////////////////////////////////////////////
