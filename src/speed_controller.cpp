////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 10, 2025
// Filename: speed_controller.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <functional>

#include "speed_controller.hpp"

using std::placeholders::_1;

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Constructs an instance of the SpeedController class
 */
SpeedController::SpeedController()
: Node("speed_controller")
, min_laser_dist_meters(2.0)
{
    this->subscription = this->create_subscription<LaserScan>(
        "/robot/base_scan", 
        10, 
        std::bind(&SpeedController::callback_LaserScan, this, _1));
}


/**
 * @brief This function listens to the LaserScan information from the
 *        DummyProximitySensor, picks out the smallest reading in the list
 *        of ranges and sets that value to be the min_laser_dist_meters, 
 *        a class member which then helps to determine the speed state.
 * @param msg - the LaserScan message we are receiving from the subscription
 */
void 
SpeedController::callback_LaserScan(const LaserScan& msg)
{
    std::vector<float> laser_ranges = msg.ranges;
    std::vector<float>::iterator it = std::min_element(
        laser_ranges.begin(), 
        laser_ranges.end());

    if (it != laser_ranges.end()) 
    {
        this->min_laser_dist_meters = *it;
        RCLCPP_INFO(
            this->get_logger(), 
            "Min laser dist: %f meters", 
            this->min_laser_dist_meters);
    }
    else 
    {
        RCLCPP_INFO(this->get_logger(), "LaserScan ranges list is empty");
    }
}

////////////////////////////////////////////////////////////////////////////////
