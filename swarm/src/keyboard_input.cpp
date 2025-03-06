#include "keyboard_input.hpp"
#include <unistd.h>
#include <fcntl.h>

KeyboardInput::KeyboardInput() {
    enableRawMode();
}

KeyboardInput::~KeyboardInput() {
    disableRawMode();
}

void KeyboardInput::enableRawMode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

void KeyboardInput::disableRawMode() {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

char KeyboardInput::getKey() {
    char c;
    int result = read(STDIN_FILENO, &c, 1);
    return result > 0 ? c : -1;
}