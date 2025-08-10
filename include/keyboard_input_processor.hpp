////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 10, 2025
// Filename: keyboard_input_processor.hpp
//
////////////////////////////////////////////////////////////////////////////////

#ifndef KEYBOARD_INPUT_PROCESSOR_H
#define KEYBOARD_INPUT_PROCESSOR_H

#include <string>
#include <termios.h> 

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief This class prints a message to the user prompting them for input, 
 *        then puts the terminal into raw mode and reads each character
 *        that is typed individually into last_clicked_key. 
 */
class KeyboardInputProcessor
{
public:

    KeyboardInputProcessor(std::string startup_message);
    ~KeyboardInputProcessor();
    
    char get_last_clicked_key();
    void restore_terminal_settings();

private:

    char last_clicked_key;    
    struct termios terminal_settings;

    void save_terminal_settings();
    void setup_terminal_for_input();
};

////////////////////////////////////////////////////////////////////////////////

#endif // KEYBOARD_INPUT_PROCESSOR_H
