#include <vrpn_Tracker.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>

// Forward declaration
cv::Point worldToScreen(double x, double y, const cv::Point& center);

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
int selected_tracker_idx = -1; // -1 means show all trackers

// Visualization settings
const double SCALE = 50.0;  // Scale factor for visualization
const double OFFSET_X = 0.0;  // Offset to center the visualization
const double OFFSET_Y = 0.0;

// Callback function
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
    cv::Point center(500, 400); // Must match the center in main loop (display.cols/2, display.rows/2)
    cv::Point pos = worldToScreen(t.pos[0], t.pos[1], center);
    g_trackers[*name].path.push_back(pos);
    
    // Keep path length limited
    if (g_trackers[*name].path.size() > g_trackers[*name].MAX_PATH_LENGTH) {
        g_trackers[*name].path.erase(g_trackers[*name].path.begin());
    }
    
    // Silent updates - don't print position data
    // std::cout << "Received update from " << *name << ": "
    //           << "x=" << t.pos[0] << " y=" << t.pos[1] << " z=" << t.pos[2] << std::endl;
}

cv::Point worldToScreen(double x, double y, const cv::Point& center) {
    // Convert world coordinates to screen coordinates
    return cv::Point(
        center.x + static_cast<int>((x + OFFSET_X) * SCALE),
        center.y - static_cast<int>((y + OFFSET_Y) * SCALE)  // Flip Y for screen coordinates
    );
}

int main() {
    // Create visualization window
    cv::namedWindow("Tracker Visualization", cv::WINDOW_AUTOSIZE);
    cv::Mat display(800, 1000, CV_8UC3);  // Larger display

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

    std::vector<std::pair<std::string*, vrpn_Tracker_Remote*>> trackers;
    int color_idx = 0;

    // Initialize trackers (same as before)
    auto add_tracker = [&](const std::string& base_name, int idx) {
        std::string name = base_name + std::to_string(idx);
        // Connect to trackers, no console output
        
        auto tracker_name = new std::string(name);
        auto tracker = new vrpn_Tracker_Remote((name + "@192.168.1.100:3883").c_str());
        
        // Disable VRPN's error logging (redirect to nowhere)
        // This prevents messages like "Trying to reconnect" from appearing in the console
        tracker->shutup = true;
        
        tracker->register_change_handler(tracker_name, handle_tracker);
        trackers.push_back({tracker_name, tracker});
        
        // Initialize tracker with empty path
        TrackerData td;
        td.x = 0;
        td.y = 0;
        td.z = 0;
        td.color = colors[color_idx++ % colors.size()];
        td.updated = false;
        g_trackers[name] = td;
    };

    for (int i = 1; i <= 3; i++) {
        add_tracker("Tracker", i);
    }
    for (int i = 1; i <= 6; i++) {
        add_tracker("Bird", i);
    }

    // No console output

    // Main loop
    while (true) {
        for (auto& t : trackers) {
            t.second->mainloop();
        }

        // Clear display
        display = cv::Scalar(0, 0, 0);

        // Draw grid
        cv::Point center(display.cols/2, display.rows/2);
        const int grid_size = 10;
        const int grid_step = 100;
        
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
            
            // Get selected tracker name
            std::string selected_name;
            if (selected_tracker_idx >= 0 && selected_tracker_idx < static_cast<int>(tracker_names.size())) {
                selected_name = tracker_names[selected_tracker_idx];
            }
            
            // Display selection status
            std::string selection_text = "Selected: " + (selected_name.empty() ? "All" : selected_name);
            cv::putText(display, selection_text, cv::Point(10, text_y), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
            text_y += 25; // Add a bit more spacing
            
            // Draw all trackers' status info in the HUD
            for (const auto& name : tracker_names) {
                const auto& t = g_trackers[name];
                auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(
                    now - t.last_update).count();
                
                // Status text - always show all trackers in the HUD
                cv::Scalar status_color;
                if (name == selected_name) {
                    // Highlight selected tracker
                    status_color = cv::Scalar(255, 255, 0); // Yellow for selected
                } else {
                    status_color = time_diff < 3 ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);
                }
                
                std::string status_text = name + ": " + (time_diff < 3 ? 
                    "Active (" + std::to_string(t.x) + ", " + std::to_string(t.y) + ")" : 
                    "No Signal");
                cv::putText(display, status_text, cv::Point(10, text_y), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, status_color);
                text_y += 20;
                
                // Only draw visible trackers (either all when selected_tracker_idx is -1 or just the selected one)
                if ((selected_tracker_idx == -1 || name == selected_name) && time_diff < 3) {
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
                    // Extract yaw from quaternion - this shows the heading direction in the XY plane
                    double siny_cosp = 2.0 * (t.qw * t.qz + t.qx * t.qy);
                    double cosy_cosp = 1.0 - 2.0 * (t.qy * t.qy + t.qz * t.qz);
                    double yaw = std::atan2(siny_cosp, cosy_cosp);
                    
                    // Convert yaw angle to a direction vector in XY plane
                    double vx = std::cos(yaw);
                    double vy = std::sin(yaw);
                    double vz = 0.0; // Not using Z for the arrow
                    
                    // Normalize and scale for arrow length
                    double length = 20.0; // Arrow length in pixels
                    double norm = std::sqrt(vx*vx + vy*vy);
                    if (norm > 1e-6) { // Avoid division by zero
                        vx = vx / norm * length;
                        vy = vy / norm * length;
                    } else {
                        vx = length;
                        vy = 0;
                    }
                    
                    // Draw arrow
                    cv::Point arrowEnd(pos.x + static_cast<int>(vx), pos.y - static_cast<int>(vy)); // Flip Y for screen coords
                    cv::arrowedLine(display, pos, arrowEnd, t.color, 2, cv::LINE_AA, 0, 0.3);
                    
                    // Draw position circle (draw after the arrow to see it on top)
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
            break;
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
        else if (key == ' ') {
            // Cycle through trackers with spacebar
            std::lock_guard<std::mutex> lock(g_mutex);
            std::vector<std::string> tracker_names;
            
            // Collect all tracker names
            for (const auto& tracker : g_trackers) {
                tracker_names.push_back(tracker.first);
            }
            
            // Sort tracker names for consistent cycling order
            std::sort(tracker_names.begin(), tracker_names.end());
            
            if (tracker_names.empty()) {
                selected_tracker_idx = -1; // Show all if no trackers
            } else {
                // Calculate next index (including "show all" option)
                int num_trackers = static_cast<int>(tracker_names.size());
                
                // Increment the index (wrapping around to -1 for "show all")
                if (selected_tracker_idx >= num_trackers - 1) {
                    // We're at the last tracker, go back to "show all" (-1)
                    selected_tracker_idx = -1;
                } else {
                    // Move to next tracker
                    selected_tracker_idx++;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Cleanup
    for (auto& t : trackers) {
        delete t.first;
        delete t.second;
    }

    return 0;
}