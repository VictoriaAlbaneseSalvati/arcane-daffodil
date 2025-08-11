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

#include "proximity_sensor_base.hpp"

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief This ProximitySensor iss setup such that each beam reads the same 
 *        value, and that value bounces back and forth continuously between 
 *        the min and max sensor values at increments of 0.1 meters.  Just a 
 *        simple dummy pattern.
 */
class DummyProximitySensor : public ProximitySensor
{
public:

    DummyProximitySensor();

private:

    std::vector<float> laser_range_pattern;
    size_t count;

    std::vector<float> init_dummy_laser_range_pattern();
    LaserScan init_LaserScan();
};

////////////////////////////////////////////////////////////////////////////////

#endif // PROXIMITY_SENSOR_DUMMY_H
