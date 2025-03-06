#include "../include/optitrack_viz.h"
#include <opencv2/opencv.hpp>
#include <vrpn_Tracker.h>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <algorithm>

// Structure to hold position data
struct TrackerData {
    double x{0}, y{0}, z{0};
    double qx{0}, qy{0}, qz{0}, qw{1.0}; // Quaternion for orientation
    cv::Scalar color;
    bool updated{false};
    std::chrono::system_clock::time_point last_update;
    std::vector<cv::Point> path;  // Store path history
    static const size_t MAX_PATH_LENGTH = 2500;  // Maximum number of points to store
};

// Global data with mutex protection
std::map<std::string, TrackerData> g_trackers;
std::mutex g_mutex;
std::vector<std::pair<std::string*, vrpn_Tracker_Remote*>> trackers;
bool viz_initialized = false;
std::thread viz_thread;
bool running = true;

// Visualization settings
const double SCALE = 50.0;  // Scale factor for visualization
const double OFFSET_X = 0.0;  // Offset to center the visualization
const double OFFSET_Y = 0.0;

// Convert world coordinates to screen coordinates
cv::Point worldToScreen(double x, double y, const cv::Point& center) {
    return cv::Point(
        center.x + static_cast<int>((x + OFFSET_X) * SCALE),
        center.y - static_cast<int>((y + OFFSET_Y) * SCALE)  // Flip Y for screen coordinates
    );
}

// Convert quaternion to yaw angle
double quaternionToYaw(double qw, double qx, double qy, double qz) {
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

// VRPN callback function
void VRPN_CALLBACK handle_tracker(void* userData, const vrpn_TRACKERCB t) {
    std::string* name = static_cast<std::string*>(userData);
    std::lock_guard<std::mutex> lock(g_mutex);
    
    g_trackers[*name].x = t.pos[0];
    g_trackers[*name].y = t.pos[1];
    g_trackers[*name].z = t.pos[2];
    g_trackers[*name].qx = t.quat[0];
    g_trackers[*name].qy = t.quat[1];
    g_trackers[*name].qz = t.quat[2];
    g_trackers[*name].qw = t.quat[3];
    g_trackers[*name].updated = true;
    g_trackers[*name].last_update = std::chrono::system_clock::now();
    
    // Add current position to path history
    cv::Point center(500, 400); // Must match the center in the visualization loop
    cv::Point pos = worldToScreen(t.pos[0], t.pos[1], center);
    g_trackers[*name].path.push_back(pos);
    
    // Keep path length limited
    if (g_trackers[*name].path.size() > TrackerData::MAX_PATH_LENGTH) {
        g_trackers[*name].path.erase(g_trackers[*name].path.begin());
    }
}

// Initialize trackers
void initialize_trackers() {
    if (viz_initialized) return;
    
    // Predefined colors
    std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),   // Blue
        cv::Scalar(0, 255, 0),   // Green
        cv::Scalar(0, 0, 255),   // Red
        cv::Scalar(255, 255, 0), // Cyan
        cv::Scalar(255, 0, 255), // Magenta
        cv::Scalar(0, 255, 255), // Yellow
        cv::Scalar(128, 0, 0),   // Dark Blue
        cv::Scalar(0, 128, 0),   // Dark Green
        cv::Scalar(0, 0, 128)    // Dark Red
    };
    
    int color_idx = 0;
    
    auto add_tracker = [&](const std::string& base_name, int idx) {
        std::string name = base_name + std::to_string(idx);
        
        auto tracker_name = new std::string(name);
        auto tracker = new vrpn_Tracker_Remote((name + "@192.168.1.100:3883").c_str());
        
        // Disable VRPN's error logging
        tracker->shutup = true;
        
        tracker->register_change_handler(tracker_name, handle_tracker);
        trackers.push_back({tracker_name, tracker});
        
        // Initialize tracker with empty path
        TrackerData td;
        td.color = colors[color_idx++ % colors.size()];
        td.updated = false;
        g_trackers[name] = td;
    };
    
    // Initialize the same trackers as in vrpn_viz.cpp
    for (int i = 1; i <= 3; i++) {
        add_tracker("Tracker", i);
    }
    for (int i = 1; i <= 6; i++) {
        add_tracker("Bird", i);
    }
    
    viz_initialized = true;
}

// Visualization thread function
void visualization_thread_func() {
    // Create visualization window
    cv::namedWindow("Tracker Visualization", cv::WINDOW_AUTOSIZE);
    cv::Mat display(800, 1000, CV_8UC3);  // Larger display
    
    while (running) {
        // Process VRPN messages for all trackers
        for (auto& t : trackers) {
            t.second->mainloop();
        }
        
        // Clear display
        display = cv::Scalar(0, 0, 0);
        
        // Draw grid
        cv::Point center(display.cols/2, display.rows/2);
        const int grid_size = 10;
        
        // Draw coordinate grid
        for (int i = -grid_size; i <= grid_size; i++) {
            // Vertical lines
            cv::Point p1 = worldToScreen(i, -grid_size, center);
            cv::Point p2 = worldToScreen(i, grid_size, center);
            cv::line(display, p1, p2, cv::Scalar(50, 50, 50), 1);
            
            // Horizontal lines
            p1 = worldToScreen(-grid_size, i, center);
            p2 = worldToScreen(grid_size, i, center);
            cv::line(display, p1, p2, cv::Scalar(50, 50, 50), 1);
        }
        
        // Draw axes
        cv::line(display, center, worldToScreen(grid_size, 0, center), cv::Scalar(0, 0, 255), 2);  // X axis
        cv::line(display, center, worldToScreen(0, grid_size, center), cv::Scalar(0, 255, 0), 2);  // Y axis
        
        cv::putText(display, "X", worldToScreen(grid_size + 0.5, 0, center), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
        cv::putText(display, "Y", worldToScreen(0, grid_size + 0.5, center), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
        
        // Draw status text
        int text_y = 20;
        cv::putText(display, "Tracking Status:", cv::Point(10, text_y), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
        text_y += 20;
        
        // Draw trackers
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            auto now = std::chrono::system_clock::now();
            
            // Collect and sort tracker names for consistent display order
            std::vector<std::string> tracker_names;
            for (const auto& t : g_trackers) {
                tracker_names.push_back(t.first);
            }
            std::sort(tracker_names.begin(), tracker_names.end());
            
            // Draw all trackers' status info in the HUD
            for (const auto& name : tracker_names) {
                const auto& t = g_trackers[name];
                auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(
                    now - t.last_update).count();
                
                // Status text
                cv::Scalar status_color = time_diff < 3 ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);
                
                std::string status_text = name + ": " + (time_diff < 3 ? 
                    "Active (" + std::to_string(t.x) + ", " + std::to_string(t.y) + ")" : 
                    "No Signal");
                cv::putText(display, status_text, cv::Point(10, text_y), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, status_color);
                text_y += 20;
                
                // Only draw visible trackers
                if (time_diff < 3) {
                    // Draw tracker position
                    cv::Point pos = worldToScreen(t.x, t.y, center);
                    
                    // Draw path history
                    if (t.path.size() > 1) {
                        // Draw lines between consecutive points with fading transparency
                        for (size_t i = 1; i < t.path.size(); i++) {
                            // Calculate alpha based on position in path (newer points are more visible)
                            double alpha = std::min(1.0, 0.3 + 0.7 * (static_cast<double>(i) / t.path.size()));
                            cv::Scalar pathColor = t.color * alpha;
                            
                            // Draw line segment
                            cv::line(display, t.path[i-1], t.path[i], pathColor, 1);
                        }
                    }
                    
                    // Draw orientation arrow showing yaw (heading) direction
                    double yaw = quaternionToYaw(t.qw, t.qx, t.qy, t.qz);
                    
                    // Convert yaw angle to a direction vector in XY plane
                    double vx = std::cos(yaw);
                    double vy = std::sin(yaw);
                    
                    // Arrow length in pixels
                    double length = 20.0;
                    
                    // Draw arrow
                    cv::Point arrowEnd(pos.x + static_cast<int>(vx * length), 
                                      pos.y - static_cast<int>(vy * length)); // Flip Y for screen coords
                    cv::arrowedLine(display, pos, arrowEnd, t.color, 2, cv::LINE_AA, 0, 0.3);
                    
                    // Draw position circle
                    cv::circle(display, pos, 5, t.color, -1);
                    
                    // Draw position text
                    std::string pos_text = name + " Z:" + std::to_string(t.z);
                    cv::putText(display, pos_text, pos + cv::Point(10, 10), 
                              cv::FONT_HERSHEY_SIMPLEX, 0.5, t.color);
                }
            }
        }
        
        cv::imshow("Tracker Visualization", display);
        
        char key = cv::waitKey(1);
        if (key == 27) // ESC
            running = false;
        else if (key == '+' || key == '=') {
            // Increase scale
            const_cast<double&>(SCALE) *= 1.1;
        }
        else if (key == '-' || key == '_') {
            // Decrease scale
            const_cast<double&>(SCALE) /= 1.1;
        }
        else if (key == 'c' || key == 'C') {
            // Clear all paths
            std::lock_guard<std::mutex> lock(g_mutex);
            for (auto& tracker : g_trackers) {
                tracker.second.path.clear();
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    cv::destroyAllWindows();
}

// Start visualization thread
void start_visualization() {
    if (!viz_initialized) {
        initialize_trackers();
        running = true;
        viz_thread = std::thread(visualization_thread_func);
    }
}

// Stop visualization thread
void stop_visualization() {
    if (viz_initialized) {
        running = false;
        if (viz_thread.joinable()) {
            viz_thread.join();
        }
        
        // Cleanup trackers
        for (auto& t : trackers) {
            delete t.first;
            delete t.second;
        }
        trackers.clear();
        viz_initialized = false;
    }
}

// Main interface function that's called from drone_control.cpp
void update_optitrack_viz(double x, double y, double yaw) {
    if (!viz_initialized) {
        start_visualization();
    }
    
    // We're not using these parameters directly since we're getting data from VRPN,
    // but we could use them to highlight the current commanded position
    // std::cout << "OptiTrack visualization updated: x=" << x << ", y=" << y << ", yaw=" << yaw << std::endl;
}

// Get OptiTrack X position from the "Bird1" tracker (or first available)
double get_optitrack_x() {
    std::lock_guard<std::mutex> lock(g_mutex);
    
    // First try Bird1 which is typically the one we control
    if (g_trackers.find("Bird1") != g_trackers.end()) {
        const auto& tracker = g_trackers["Bird1"];
        auto now = std::chrono::system_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - tracker.last_update).count();
        
        if (time_diff < 3 && tracker.updated) {
            return tracker.x;
        }
    }
    
    // Fall back to first active tracker
    for (const auto& tracker_pair : g_trackers) {
        const auto& tracker = tracker_pair.second;
        auto now = std::chrono::system_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - tracker.last_update).count();
        
        if (time_diff < 3 && tracker.updated) {
            return tracker.x;
        }
    }
    
    return 0.0; // Default if no trackers available
}

// Get OptiTrack Y position from the "Bird1" tracker (or first available)
double get_optitrack_y() {
    std::lock_guard<std::mutex> lock(g_mutex);
    
    // First try Bird1 which is typically the one we control
    if (g_trackers.find("Bird1") != g_trackers.end()) {
        const auto& tracker = g_trackers["Bird1"];
        auto now = std::chrono::system_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - tracker.last_update).count();
        
        if (time_diff < 3 && tracker.updated) {
            return tracker.y;
        }
    }
    
    // Fall back to first active tracker
    for (const auto& tracker_pair : g_trackers) {
        const auto& tracker = tracker_pair.second;
        auto now = std::chrono::system_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - tracker.last_update).count();
        
        if (time_diff < 3 && tracker.updated) {
            return tracker.y;
        }
    }
    
    return 0.0; // Default if no trackers available
}

// Get OptiTrack yaw from the "Bird1" tracker (or first available)
double get_optitrack_yaw() {
    std::lock_guard<std::mutex> lock(g_mutex);
    
    // First try Bird1 which is typically the one we control
    if (g_trackers.find("Bird1") != g_trackers.end()) {
        const auto& tracker = g_trackers["Bird1"];
        auto now = std::chrono::system_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - tracker.last_update).count();
        
        if (time_diff < 3 && tracker.updated) {
            // Convert quaternion to yaw in degrees
            double yaw_rad = quaternionToYaw(tracker.qw, tracker.qx, tracker.qy, tracker.qz);
            return yaw_rad * 180.0 / M_PI; // Convert to degrees
        }
    }
    
    // Fall back to first active tracker
    for (const auto& tracker_pair : g_trackers) {
        const auto& tracker = tracker_pair.second;
        auto now = std::chrono::system_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - tracker.last_update).count();
        
        if (time_diff < 3 && tracker.updated) {
            // Convert quaternion to yaw in degrees
            double yaw_rad = quaternionToYaw(tracker.qw, tracker.qx, tracker.qy, tracker.qz);
            return yaw_rad * 180.0 / M_PI; // Convert to degrees
        }
    }
    
    return 0.0; // Default if no trackers available
}