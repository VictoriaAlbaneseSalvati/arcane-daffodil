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
: ProximitySensor()
, count(0)
{
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
        dummy_pattern.push_back((i / 10.0) + this->EPSILON);
    }
    for (int i = 1; i <= 20; i++) 
    {
        dummy_pattern.push_back((i / 10.0) + this->EPSILON);
    }
    return dummy_pattern;    
} 


/**
 * @brief Creates a dummy LaserScan message with values in accordance with
 *        this sensor, as defined in the header file.
 * @returns an initialized dummy LaserScan message 
 */
LaserScan
DummyProximitySensor::init_LaserScan()
{
    LaserScan laser_scan = this->init_LaserScan_params();
    
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

////////////////////////////////////////////////////////////////////////////////
