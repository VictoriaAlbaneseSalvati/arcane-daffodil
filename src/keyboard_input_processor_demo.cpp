////////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 10, 2025
// Filename: keyboard_input_processor_demo.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <sstream>

#include "keyboard_input_processor.hpp"

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief This demo is simply meant for a human being to be able to manually
 *        interact with the keyboard input processor.  It is not actually 
 *        connected to any kind of E-Stop, it's just a dummy.
 */
int main([[maybe_unused]]int argc, [[maybe_unused]]char * argv[])
{
    std::stringstream ss;
    ss << "////////////////////////////////////////////////////" << std::endl;
    ss << "//                                                //" << std::endl;
    ss << "// E-STOP Controls:                               //" << std::endl;
    ss << "//                                                //" << std::endl;
    ss << "// >> Press 'E' to turn the E-STOP on             //" << std::endl;
    ss << "// >> Press 'R' to reset and shut the E-STOP off  //" << std::endl;
    ss << "//                                                //" << std::endl;
    ss << "// >> Press 'Q' to quit the program               //" << std::endl;
    ss << "//                                                //" << std::endl;
    ss << "// Warning: if you do not follow instructions and //" << std::endl;
    ss << "// exit the program with Ctrl-C instead of Q, the //" << std::endl;
    ss << "// terminal will not be reset properly.  If this  //" << std::endl;
    ss << "// happens, you can fix it with the following:    //" << std::endl;
    ss << "//                                                //" << std::endl;
    ss << "// $ reset                                        //" << std::endl;
    ss << "//                                                //" << std::endl;
    ss << "////////////////////////////////////////////////////" << std::endl;
 
    KeyboardInputProcessor kip(ss.str());

    char key;
    while ((key = kip.get_last_clicked_key()) != 'Q') 
    {
        std::string suffix = "";

        switch(key) 
        {
            case 'E':
                suffix = " - E-Stop on";
                break;
            case 'R':
                suffix = " - E-Stop off";
                break;
            default:
                break;
        }

        std::cout << key << suffix << std::endl;
    }
    
    std::cout << std::endl;
    std::cout << "So long, and thanks for all the fish!" << std::endl;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
