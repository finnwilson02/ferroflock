#include <vrpn_Tracker.h>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

// setup the socket
int create_socket() {
    int server_fd;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        std::cerr << "Socket creation failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        std::cerr << "Socket options failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(8080);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cerr << "Socket bind failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 3) < 0) {
        std::cerr << "Socket listen failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    int new_socket;
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0) {
        std::cerr << "Socket accept failed" << std::endl;
        exit(EXIT_FAILURE);
    }
    return new_socket;
}

// callback function to handle tracker updates
void VRPN_CALLBACK handle_tracker(void *sock_fd, const vrpn_TRACKERCB t) {
    int sock = *(int*)sock_fd;
    std::string data = "Tracker Pos: " + std::to_string(t.pos[0]) + ", " + std::to_string(t.pos[1]) + ", " + std::to_string(t.pos[2]) + "\n";
    data += "Tracker Quat: " + std::to_string(t.quat[0]) + ", " + std::to_string(t.quat[1]) + ", " + std::to_string(t.quat[2]) + ", " + std::to_string(t.quat[3]) + "\n";
    send(sock, data.c_str(), data.length(), 0);
}

int main(int argc, char **argv) {
    int sock = create_socket();

    vrpn_Tracker_Remote *tracker = new vrpn_Tracker_Remote("Test@192.168.1.100:3883");
    tracker->register_change_handler(&sock, handle_tracker);

    while (true) {
        tracker->mainloop();
    }

    return 0;
}
