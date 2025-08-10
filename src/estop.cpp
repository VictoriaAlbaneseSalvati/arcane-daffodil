////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 10, 2025
// Filename: estop.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include <chrono>
#include <cstdlib>
#include <functional>

#include "estop.hpp"

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Constructs and instance of the EStop class
 */
EStop::EStop()
: Node("e_stop")
, keyboard_input_processor(STARTUP_MESSAGE)
, is_estop_on(false)
{ 
    this->timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&EStop::publish_EStopState, this));
    this->publisher = this->create_publisher<BoolMsg>("estop", 10);
}


/**
 * @brief Read a key from the terminal and use the result to change the  
 *        EStop State, if applicable, or exit the program.  The EStop State
 *        is then published-- a simple boolean message, true if the EStop 
 *        is on, false otherwise. 
 */
void 
EStop::publish_EStopState() 
{
    char key = this->keyboard_input_processor.get_last_clicked_key();
    switch(key) 
    {
        case 'E':
            this->is_estop_on = true;
            RCLCPP_INFO(this->get_logger(), "E-STOP on!");
            break;
        case 'R':
            this->is_estop_on = false;
            RCLCPP_INFO(this->get_logger(), "E-STOP off!");
            break;
        case 'Q':
            RCLCPP_INFO(
                this->get_logger(), 
                "E-STOP has been shut down, goodbye!");
            this->keyboard_input_processor.restore_terminal_settings();
            exit(EXIT_FAILURE);
            break;
        default:
            break;
    }

    BoolMsg msg;
    msg.data = this->is_estop_on;
    this->publisher->publish(msg);
}

////////////////////////////////////////////////////////////////////////////////
