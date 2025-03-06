#pragma once

#include <termios.h>

class KeyboardInput {
public:
    KeyboardInput();
    ~KeyboardInput();
    
    // Get single keypress, returns -1 if no key pressed
    char getKey();

private:
    struct termios orig_termios;
    void enableRawMode();
    void disableRawMode();
};