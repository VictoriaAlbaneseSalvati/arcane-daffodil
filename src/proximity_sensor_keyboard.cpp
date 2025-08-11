////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 10, 2025
// Filename: proximity_sensor_keyboard.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <chrono>
#include <functional>

#include "proximity_sensor_keyboard.hpp"

// This will be added to the laser ranges that are being manually set, 
// and which are meant to be even increments of 0.1; however, because of
// floating point precision, sometimes the values are slightly less than
// the value they are meant to be, which can make the visualization 
// misleading since that has exact values.  So it's better to have the
// laserscan values be slightly bigger than the exact value that smaller.
#define EPSILON 0.000001

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Constructs an instance of the KeyboardProximitySensor class
 */
KeyboardProximitySensor::KeyboardProximitySensor()
: Node("proximity_sensor_keyboard")
, keyboard_input_processor(STARTUP_MESSAGE)
, last_sensor_value(2.0 + EPSILON)
{
    this->timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&KeyboardProximitySensor::publish_LaserScan, this));
    this->publisher = this->create_publisher<LaserScan>("robot/base_scan", 10);
    std::cout << this->make_ascii_laser();
}


/**
 * @brief Make a simple ascii visualization of a robot with a single laser
 *        beam coming out of it to represent the min sensor reading.
 */
std::string
KeyboardProximitySensor::make_ascii_laser() 
{
    std::string laser_beam = "";
    for (int i = 1; i <= 20; i++) 
    {
        if (i < std::round(this->last_sensor_value * 10))
        {
            laser_beam+= "<<";
        }
        else if (i == std::round(this->last_sensor_value * 10))
        {
            laser_beam+= "<|";
        }
        else 
        {
            laser_beam+= "  ";
        }
    }

    std::stringstream ascii_art;
    ascii_art << STARTUP_MESSAGE; 
    ascii_art << "//                                                                    //\n";
    ascii_art << "// +-------+                                                          //\n";
    ascii_art << "// | robot =" << laser_beam << "                  //\n";
    ascii_art << "// +-------+                                                          //\n";
    ascii_art << "//         |       |       |       |       |       |                  //\n";
    ascii_art << "//        0.0 m   0.4 m   0.8 m   1.2 m   1.6 m   2.0 m               //\n";
    ascii_art << "//                                                                    //\n";
    ascii_art << "////////////////////////////////////////////////////////////////////////\n";

    return ascii_art.str();
}


/**
 * @brief Creates a dummy LaserScan message with values in accordance with
 *        this sensor, as defined in the header file.  The scan values are 
 *        determined by user keyboard inputs.
 * @returns an initialized dummy LaserScan message 
 */
LaserScan
KeyboardProximitySensor::init_keyboard_LaserScan()
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

    char key = this->keyboard_input_processor.get_last_clicked_key();
    switch(key) 
    {
        case '>':
            this->last_sensor_value = std::min(
                this->RANGE_MAX_METERS, 
                static_cast<float>(this->last_sensor_value + 0.1 + EPSILON));
                std::cout << this->make_ascii_laser();
            break;
        case '<':
            this->last_sensor_value = std::max(
                this->RANGE_MIN_METERS,
                static_cast<float>(this->last_sensor_value - 0.1 + EPSILON));
                std::cout << this->make_ascii_laser();
            break;
        case 'Q':
            RCLCPP_INFO(
                this->get_logger(), 
                "Laser has been shut down, goodbye!");
            this->keyboard_input_processor.restore_terminal_settings();
            exit(EXIT_FAILURE);
            break;
        default:
            break;
    }
   
    for (int i = 0; i < this->NUM_BEAMS; i++) 
    {
        laser_scan.ranges[i] = this->last_sensor_value;
    }
   
    return laser_scan;
}


/**
 * @brief Creates and publishes dummy LaserScan messages
 */
void 
KeyboardProximitySensor::publish_LaserScan() 
{
    LaserScan laser_scan = this->init_keyboard_LaserScan();
    this->publisher->publish(laser_scan);
}

////////////////////////////////////////////////////////////////////////////////
