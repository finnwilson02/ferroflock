#pragma once

#include <string>
#include <functional>
#include <array>
#include <mutex>

// Simple VRPN client implementation
class VrpnClient {
public:
    struct TrackerCallback {
        std::function<void(const std::string&, const std::array<float, 3>&, const std::array<float, 4>&)> callback;
    };

    VrpnClient(const std::string& server_ip = "192.168.1.100", int port = 3883);
    ~VrpnClient();

    bool connect();
    void disconnect();
    bool isConnected() const;
    
    // Set callback for tracker data
    void setTrackerCallback(const TrackerCallback& callback);

    // Process VRPN messages
    void update();

private:
    std::string server_ip;
    int server_port;
    int socket_fd{-1};
    bool connected{false};
    std::mutex mutex;
    TrackerCallback tracker_callback;
    
    // Process received tracker data
    void processTrackerData(const std::string& data);
};