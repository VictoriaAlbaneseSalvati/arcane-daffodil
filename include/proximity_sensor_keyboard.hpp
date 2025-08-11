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

#include "proximity_sensor_base.hpp"
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
 * @brief The KeyboardProximitySensor  is setup such that the user can control 
 *        the sensor readings with the '<'/'>' keys on the keyboard to make for 
 *        an interactive demo.
 */
class KeyboardProximitySensor : public ProximitySensor
{
public:

    KeyboardProximitySensor();

private:

    KeyboardInputProcessor keyboard_input_processor;
    float last_sensor_value;

    std::string make_ascii_laser();
    LaserScan init_LaserScan();
};

////////////////////////////////////////////////////////////////////////////////

#endif // PROXIMITY_SENSOR_KEYBOARD_H
