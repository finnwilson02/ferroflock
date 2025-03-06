#include "multi_tello_controller.hpp"
#include <iostream>

int main() {
    MultiTelloController controller;
    
    if (!controller.initialize()) {
        std::cerr << "Failed to initialize any Tello connections!" << std::endl;
        return 1;
    }
    
    controller.run();
    return 0;
}