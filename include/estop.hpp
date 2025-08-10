////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 10, 2025
// Filename: estop.hpp
//
////////////////////////////////////////////////////////////////////////////////

#ifndef ESTOP_H
#define ESTOP_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "keyboard_input_processor.hpp"

using BoolMsg = std_msgs::msg::Bool;

const std::string STARTUP_MESSAGE = "\
////////////////////////////////////////////////////////////////////////\n\
//                                                                    //\n\
// E-STOP Controls:                                                   //\n\
//                                                                    //\n\
// >> Press 'E' to turn the E-STOP on                                 //\n\
// >> Press 'R' to reset and shut the E-STOP off                      //\n\
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
 * @brief The E-STOP has a KeyboardInputProcessor that drives it's output, 
 *        a /estop topic which publishes a std_msgs::msg::Bool whenever a new
 *        key is pressed.  Only a few keys evoke change though:
 *              >> E - publishes true
 *              >> R - publishes false
 *              >> Q - exit the program
 *        any other key press will simply repeat the last published message.
 */
class EStop : public rclcpp::Node
{
public:

    EStop();

private:

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<BoolMsg>::SharedPtr publisher;
    KeyboardInputProcessor keyboard_input_processor;
    bool is_estop_on;

    void publish_EStopState();
};

////////////////////////////////////////////////////////////////////////////////

#endif // ESTOP_H
