////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 10, 2025
// Filename: keyboard_input_processor.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <unistd.h> 

#include "keyboard_input_processor.hpp"

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Constructs an instance of the KeyboardInputProcessor class
 */
KeyboardInputProcessor::KeyboardInputProcessor(std::string startup_message)
: last_clicked_key('R')
{
    this->save_terminal_settings();
    this->setup_terminal_for_input();
    std::cout << startup_message;
}


/**
 * @brief Destroys an instance of the KeyboardInputProcessor class
 */
KeyboardInputProcessor::~KeyboardInputProcessor() 
{
    this->restore_terminal_settings();
}


/**
 * @brief Save the original terminal settings before setting it up to 
 *        read input from the terminal.
 */
void
KeyboardInputProcessor::save_terminal_settings() 
{
    tcgetattr(STDIN_FILENO, &this->terminal_settings);
}


/**
 * @brief Restore the previously saved terminal settings.
 */
void
KeyboardInputProcessor::restore_terminal_settings() 
{
    tcsetattr(STDIN_FILENO, TCSANOW, &this->terminal_settings);
}


/**
 * @brief Set the terminal to raw mode for reading keyboard input.
 */
void
KeyboardInputProcessor::setup_terminal_for_input() 
{
    struct termios raw_termios = this->terminal_settings;
    raw_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_termios);
}


/**
 * @brief Check to see if there has been any new keyboard input, and if
 *        there is, update last_clicked_key; otherwise, return the 
 *        last_clicked_key whether it is new or stale.
 * @returns char - the last clicked key, whether new or stale.
 */
char
KeyboardInputProcessor::get_last_clicked_key() 
{
    char key;

    // if there is a fresh key click from the keyboard, 
    // update the last_clicked_key 
    if (read(STDIN_FILENO, &key, 1) > 0)
    {
        this->last_clicked_key = std::toupper(key);
    }

    // return the last_clicked_key, whether new or stale
    return this->last_clicked_key;
}

////////////////////////////////////////////////////////////////////////////////
