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

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Constructs an instance of the KeyboardProximitySensor class
 */
KeyboardProximitySensor::KeyboardProximitySensor()
: ProximitySensor()
, keyboard_input_processor(STARTUP_MESSAGE)
, last_sensor_value(2.0 + EPSILON)
{
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
 * @returns a fully initialized LaserScan message 
 */
LaserScan
KeyboardProximitySensor::init_LaserScan()
{
    LaserScan laser_scan = this->init_LaserScan_params();

    char key = this->keyboard_input_processor.get_last_clicked_key();
    switch(key) 
    {
        case '>':
            this->last_sensor_value = std::min(
                this->RANGE_MAX_METERS, 
                static_cast<float>(this->last_sensor_value + 0.1 + this->EPSILON));
                std::cout << this->make_ascii_laser();
            break;
        case '<':
            this->last_sensor_value = std::max(
                this->RANGE_MIN_METERS,
                static_cast<float>(this->last_sensor_value - 0.1 + this->EPSILON));
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

////////////////////////////////////////////////////////////////////////////////
