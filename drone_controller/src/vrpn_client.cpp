#include "vrpn_client.h"
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>

VrpnClient::VrpnClient(const std::string& server_ip, int port)
    : server_ip(server_ip), server_port(port) {
}

VrpnClient::~VrpnClient() {
    disconnect();
}

bool VrpnClient::connect() {
    // Create socket
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }

    // Set up server address
    struct sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(server_ip.c_str());
    server_addr.sin_port = htons(server_port);

    // Connect to VRPN server
    if (::connect(socket_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to connect to VRPN server at " << server_ip << ":" << server_port << std::endl;
        close(socket_fd);
        socket_fd = -1;
        return false;
    }

    // Set socket to non-blocking
    int flags = fcntl(socket_fd, F_GETFL, 0);
    fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK);

    connected = true;
    return true;
}

void VrpnClient::disconnect() {
    if (socket_fd >= 0) {
        close(socket_fd);
        socket_fd = -1;
    }
    connected = false;
}

bool VrpnClient::isConnected() const {
    return connected;
}

void VrpnClient::setTrackerCallback(const TrackerCallback& callback) {
    std::lock_guard<std::mutex> lock(mutex);
    tracker_callback = callback;
}

void VrpnClient::update() {
    if (!connected || socket_fd < 0) {
        return;
    }

    // Read data from socket
    char buffer[4096];
    ssize_t received = recv(socket_fd, buffer, sizeof(buffer) - 1, 0);
    
    if (received > 0) {
        buffer[received] = '\0';
        processTrackerData(buffer);
    }
}

void VrpnClient::processTrackerData(const std::string& data) {
    std::lock_guard<std::mutex> lock(mutex);
    
    // Parse tracker data
    std::istringstream stream(data);
    std::string line;
    
    while (std::getline(stream, line)) {
        std::istringstream linestream(line);
        std::string name;
        float x, y, z, qw, qx, qy, qz;
        
        std::getline(linestream, name, ',');
        if (linestream >> x >> y >> z >> qw >> qx >> qy >> qz) {
            // Call callback if set
            if (tracker_callback.callback) {
                tracker_callback.callback(name, {x, y, z}, {qw, qx, qy, qz});
            }
        }
    }
}