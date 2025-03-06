#include <vrpn_Tracker.h>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sstream>

struct TrackerData {
    double x{0}, y{0}, z{0};
    double qw{1}, qx{0}, qy{0}, qz{0};  // Added quaternion data
    bool updated{false};
    std::chrono::system_clock::time_point last_update;
};

std::map<std::string, TrackerData> g_trackers;
std::mutex g_mutex;
bool running = true;

// VRPN callback
void VRPN_CALLBACK handle_tracker(void* userData, const vrpn_TRACKERCB t) {
    std::string* name = static_cast<std::string*>(userData);
    std::lock_guard<std::mutex> lock(g_mutex);
    
    g_trackers[*name].x = t.pos[0];
    g_trackers[*name].y = t.pos[1];
    g_trackers[*name].z = t.pos[2];
    g_trackers[*name].qw = t.quat[0];
    g_trackers[*name].qx = t.quat[1];
    g_trackers[*name].qy = t.quat[2];
    g_trackers[*name].qz = t.quat[3];
    g_trackers[*name].updated = true;
    g_trackers[*name].last_update = std::chrono::system_clock::now();
}

void socket_server(int port) {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd, 3);

    while (running) {
        int client_socket = accept(server_fd, NULL, NULL);
        if (client_socket >= 0) {
            while (running) {
                std::stringstream ss;
                {
                    std::lock_guard<std::mutex> lock(g_mutex);
                    for (const auto& tracker : g_trackers) {
                        ss << tracker.first << ","
                           << tracker.second.x << ","
                           << tracker.second.y << ","
                           << tracker.second.z << ","
                           << tracker.second.qw << ","
                           << tracker.second.qx << ","
                           << tracker.second.qy << ","
                           << tracker.second.qz << "\n";
                    }
                }
                std::string data = ss.str();
                send(client_socket, data.c_str(), data.length(), 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            close(client_socket);
        }
    }
    close(server_fd);
}

int main() {
    std::vector<std::pair<std::string*, vrpn_Tracker_Remote*>> trackers;

    // Initialize trackers
    auto add_tracker = [&](const std::string& base_name, int idx) {
        std::string name = base_name + std::to_string(idx);
        std::cout << "Connecting to " << name << "@192.168.1.100:3883" << std::endl;
        
        auto tracker_name = new std::string(name);
        auto tracker = new vrpn_Tracker_Remote((name + "@192.168.1.100:3883").c_str());
        tracker->register_change_handler(tracker_name, handle_tracker);
        trackers.push_back({tracker_name, tracker});
    };

    // Add your trackers
    for (int i = 1; i <= 9; i++) {
        add_tracker("Tracker", i);
    }

    // Start socket server in separate thread
    std::thread server_thread(socket_server, 8080);

    // Main VRPN loop
    while (running) {
        for (auto& t : trackers) {
            t.second->mainloop();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Cleanup
    server_thread.join();
    for (auto& t : trackers) {
        delete t.first;
        delete t.second;
    }

    return 0;
}