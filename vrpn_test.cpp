#include <vrpn_Tracker.h>
#include <iostream>

// callback function to handle tracker updates
void VRPN_CALLBACK handle_tracker(void *userData, const vrpn_TRACKERCB t) {
    std::cout << "Tracker Pos: " << t.pos[0] << ", " << t.pos[1] << ", " << t.pos[2] << std::endl;
    std::cout << "Tracker Quat: " << t.quat[0] << ", " << t.quat[1] << ", " << t.quat[2] << ", " << t.quat[3] << std::endl;
}

int main(int argc, char** argv) {
    // create a tracker remote object, replace with your motive server's IP and tracker name
    vrpn_Tracker_Remote *tracker = new vrpn_Tracker_Remote("Tracker1@192.168.1.100:3883");

    // register the callback function to handle tracker data
    tracker->register_change_handler(0, handle_tracker);

    // loop to keep receiving tracker data
    while (true) {
        tracker->mainloop();  // calls the mainloop to keep the client connected and processing
    }

    return 0;
}
