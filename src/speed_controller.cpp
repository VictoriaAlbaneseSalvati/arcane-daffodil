////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 10, 2025
// Filename: speed_controller.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <chrono>
#include <functional>

#include "speed_controller.hpp"

using std::placeholders::_1;

std::unordered_map<SpeedState, std::string> speed_state_to_str = {
    { SpeedState::FULL_SPEED, "FULL_SPEED" },
    { SpeedState::SLOW, "SLOW" },
    { SpeedState::STOP, "STOP" },
};

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Constructs an instance of the SpeedController class 
 *        with the given arguments
 * @param silence_logging - whether or not to print log statements
 * @param is_estop_on - whether or not the estop is on
 * @param min_laser_dist_meters - the min laser reading in meters
 */
SpeedController::SpeedController(
    const bool& silence_logging,
    const bool& is_estop_on,
    const float& min_laser_dist_meters)
: Node("speed_controller")
, min_laser_dist_meters(min_laser_dist_meters)
, is_estop_on(is_estop_on)
{
    this->silence_logging = silence_logging;
    this->determine_speed_state();

    this->timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&SpeedController::publish_CommandVelocity, this));
    this->publisher = this->create_publisher<CommandVelocity>("robot/cmd_vel", 10);
    this->estop_subscription = this->create_subscription<BoolMsg>(
        "/estop", 
        10, 
        std::bind(&SpeedController::callback_EStop, this, _1));
    this->laser_subscription = this->create_subscription<LaserScan>(
        "/robot/base_scan", 
        10, 
        std::bind(&SpeedController::callback_LaserScan, this, _1));
}


/**
 * @brief Setter & getter for is_estop_on class member
 */    
bool 
SpeedController::get_is_estop_on() const 
{ 
    return this->is_estop_on; 
}
void 
SpeedController::set_is_estop_on(const bool& new_is_estop_on) 
{ 
    this->is_estop_on = new_is_estop_on;
}

 
/**
 * @brief Setter & getter for min_laser_dist_meters class member
 */    
float 
SpeedController::get_min_laser_dist_meters() const 
{ 
    return this->min_laser_dist_meters;
}
void 
SpeedController::set_min_laser_dist_meters(
    const float& new_min_laser_dist_meters) 
{ 
    this->min_laser_dist_meters = new_min_laser_dist_meters; 
}

 
/**
 * @brief Setter & getter for is_estop_on class member
 */    
SpeedState
SpeedController::get_speed_state() const 
{ 
    return this->speed_state; 
}
void 
SpeedController::set_speed_state(const SpeedState& new_speed_state) 
{ 
    this->speed_state = new_speed_state; 
}


/**
 * @brief Determine what the speed state should be based on:
 *          (1) the estop reading - if that is on, SpeedState is STOP
 *          (2) the min_laser_dist_meters reading - determine the proper
 *              speed state based on the constants defined in this class
 *        and then set the speed_state class member accordingly.
 *
 * @note I took "within x" to mean < x and not <= x, but if that 
 *       assumption was incorrect all that would need to be done is 
 *       change these < to <=.
 */
void 
SpeedController::determine_speed_state() 
{
    if (this->get_is_estop_on()) 
    {
        this->set_speed_state(SpeedState::STOP);
    }
    else 
    {
        if (this->get_min_laser_dist_meters() < this->TOO_CLOSE_LASER_DIST_METERS) 
        {
            this->set_speed_state(SpeedState::STOP);
        }
        else if (this->get_min_laser_dist_meters() < this->CLOSE_LASER_DIST_METERS) 
        {
            this->set_speed_state(SpeedState::SLOW);
        }
        else
        {
            this->set_speed_state(SpeedState::FULL_SPEED);
        }
    }

    if (!this->silence_logging) 
    {
        RCLCPP_INFO(
            this->get_logger(), 
            "E-Stop: %-3s - Speed State: %s", 
            this->get_is_estop_on() ? "ON" : "OFF",
            speed_state_to_str[this->get_speed_state()].c_str());
    }
}


/**
 * @brief Based on the speed_state member, determine the speed that the robot
 *        should be going based on the constants defined in the speed controller
 *        and publish the appropriate speed. 
 */
void 
SpeedController::publish_CommandVelocity() 
{
    this->determine_speed_state();

    CommandVelocity velocity;
    switch(this->get_speed_state()) 
    {
        case SpeedState::STOP:
            velocity.linear.x = this->STOP_SPEED_MPS;
            break;
        case SpeedState::SLOW:
            velocity.linear.x = this->SLOW_SPEED_MPS;
            break;
        case SpeedState::FULL_SPEED:
            velocity.linear.x = this->FULL_SPEED_MPS;
            break;
    }

    this->publisher->publish(velocity);
}


/**
 * @brief This function listens to the /estop topic and sets the is_estop_on
 *        class member to that value value.
 * @param msg - the bool message from the /estop topic
 */
void 
SpeedController::callback_EStop(const BoolMsg& msg) 
{
    this->set_is_estop_on(msg.data);
}


/**
 * @brief This function listens to the /robot/base_scan topic, picks out
 *        the smallest reading in the list of ranges and then sets the
 *        min_laser_dist_meters class member to that value. 
 * @param msg - the LaserScan message from the /robot/base_scan topic
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
        this->set_min_laser_dist_meters(*it);
    }
    else 
    {
        if (!this->silence_logging)
        { 
            RCLCPP_INFO(this->get_logger(), "LaserScan ranges list is empty");
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
