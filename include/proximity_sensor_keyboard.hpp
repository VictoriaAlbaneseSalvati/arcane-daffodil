////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 10, 2025
// Filename: proximity_sensor_keyboard.hpp
//
////////////////////////////////////////////////////////////////////////////////

#ifndef PROXIMITY_SENSOR_KEYBOARD_H
#define PROXIMITY_SENSOR_KEYBOARD_H

#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "keyboard_input_processor.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;

const std::string STARTUP_MESSAGE = "\n\
////////////////////////////////////////////////////////////////////////\n\
//                                                                    //\n\
// Laser Controls:                                                    //\n\
//                                                                    //\n\
// >> Press '<' to decrease the values on the laser ranges            //\n\
// >> Press '>' to increase the values on the laser ranges            //\n\
//                                                                    //\n\
// >> Press 'Q' to quit the program                                   //\n\
//                                                                    //\n\
// Warning: if you do not follow instructions and exit the program    //\n\
// with Ctrl-C instead of Q, the terminal will not be reset properly. //\n\
// If this happens, you can fix it with the following command:        //\n\
//                                                                    //\n\
// $ reset                                                            //\n\
//                                                                    //\n\
////////////////////////////////////////////////////////////////////////\n";

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief This KeyboardProximitySensor publishes LaserScan messages every 0.5 
 *        seconds.  The sensor has 180* range, and emits 5 beams at even 
 *        intervals (so at 45* increments).  The sensor has a range between 
 *        0.1 meters and 2 meters.
 *     
 *        It's setup such that the user can control the sensor readings with
 *        the '<'/'>' keys on the keyboard to make for an interactive demo.
 */
class KeyboardProximitySensor : public rclcpp::Node
{
public:

    KeyboardProximitySensor();

private:

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<LaserScan>::SharedPtr publisher;
    KeyboardInputProcessor keyboard_input_processor;
    float last_sensor_value;

    const int NUM_BEAMS = 5;
    const float ANGLE_MIN_RAD = -M_PI / 2.0; 
    const float ANGLE_MAX_RAD = M_PI / 2.0; 
    const float ANGLE_INCREMENT_RAD = M_PI / this->NUM_BEAMS; 
    const float TIME_INCREMENT_SEC = 0.0001; 
    const float SCAN_TIME_SEC = 0.5; 
    const float RANGE_MIN_METERS = 0.1; 
    const float RANGE_MAX_METERS = 2.0; 
    
    std::string make_ascii_laser();
    LaserScan init_keyboard_LaserScan();
    void publish_LaserScan();
};

////////////////////////////////////////////////////////////////////////////////

#endif // PROXIMITY_SENSOR_KEYBOARD_H
