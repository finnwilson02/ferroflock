#include "../include/optitrack.h"
#include "../include/logger.h"
#include <algorithm>
#include <cmath>

// Constructor
OptiTrack::OptiTrack() {
    LOG_INFO("Initializing OptiTrack system");
    running_ = false;
    viz_initialized_ = false;
}

// Destructor
OptiTrack::~OptiTrack() {
    LOG_INFO("Shutting down OptiTrack system");
    
    // Stop visualization thread if running
    if (running_) {
        stopVisualization();
    }
    
    // Clean up VRPN trackers
    for (auto& tracker_pair : vrpn_trackers_) {
        delete tracker_pair.first;  // Delete tracker name string
        delete tracker_pair.second; // Delete VRPN tracker object
    }
    vrpn_trackers_.clear();
}

// Initialize OptiTrack system
void OptiTrack::initialize() {
    LOG_INFO("Starting OptiTrack initialization");
    
    if (viz_initialized_) {
        LOG_WARNING("OptiTrack already initialized, skipping");
        return;
    }
    
    initializeTrackers();
    viz_initialized_ = true;
}

// Initialize trackers
void OptiTrack::initializeTrackers() {
    LOG_INFO("Setting up OptiTrack trackers");
    
    if (viz_initialized_) {
        LOG_DEBUG("Trackers already initialized, skipping");
        return;
    }
    
    LOG_DEBUG("Setting up tracker colors");
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
    
    LOG_DEBUG("Setting up VRPN connection to 192.168.1.100:3883");
    auto add_tracker = [&](const std::string& base_name, int idx) {
        std::string name = base_name + std::to_string(idx);
        LOG_DEBUG("Creating tracker " + name);
        
        auto tracker_name = new std::string(name);
        std::string connection_string = name + "@192.168.1.100:3883";
        LOG_DEBUG("VRPN connection string: " + connection_string);
        
        auto tracker = new vrpn_Tracker_Remote(connection_string.c_str());
        
        // Disable VRPN's error logging
        tracker->shutup = true;
        LOG_DEBUG("Registering VRPN callback for " + name);
        
        tracker->register_change_handler(tracker_name, handleTrackerData);
        vrpn_trackers_.push_back({tracker_name, tracker});
        
        // Initialize tracker with empty path
        TrackerData td;
        td.color = colors[color_idx++ % colors.size()];
        td.updated = false;
        
        std::lock_guard<std::mutex> lock(tracker_mutex_);
        trackers_[name] = td;
        
        LOG_INFO("Registered OptiTrack tracker: " + name);
    };
    
    // Initialize trackers
    LOG_DEBUG("Creating Tracker1-3");
    for (int i = 1; i <= 3; i++) {
        add_tracker("Tracker", i);
    }
    
    LOG_DEBUG("Creating Bird1-6");
    for (int i = 1; i <= 6; i++) {
        add_tracker("Bird", i);
    }
    
    viz_initialized_ = true;
    LOG_INFO("Tracker initialization complete, registered 9 trackers");
}

// Start visualization in a separate thread
void OptiTrack::startVisualization() {
    LOG_INFO("Starting OptiTrack visualization");
    
    if (!viz_initialized_) {
        LOG_DEBUG("Initializing trackers first");
        initialize();
    }
    
    if (running_) {
        LOG_WARNING("Visualization already running, skipping");
        return;
    }
    
    running_ = true;
    
    LOG_DEBUG("Creating visualization thread");
    try {
        viz_thread_ = std::thread(&OptiTrack::visualizationThreadFunc, this);
        LOG_INFO("Visualization thread started successfully");
    }
    catch (const std::exception& e) {
        LOG_ERROR("Failed to start visualization thread: " + std::string(e.what()));
        running_ = false;
    }
}

// Stop visualization
void OptiTrack::stopVisualization() {
    LOG_INFO("Stopping OptiTrack visualization");
    
    if (running_) {
        LOG_DEBUG("Setting running flag to false");
        running_ = false;
        
        if (viz_thread_.joinable()) {
            LOG_DEBUG("Joining visualization thread");
            try {
                viz_thread_.join();
                LOG_INFO("Visualization thread joined successfully");
            }
            catch (const std::exception& e) {
                LOG_ERROR("Error joining visualization thread: " + std::string(e.what()));
            }
        } else {
            LOG_WARNING("Visualization thread not joinable");
        }
    } else {
        LOG_DEBUG("Visualization was not running, nothing to stop");
    }
}

// Update visualization with current position and orientation
void OptiTrack::updateVisualization(double x, double y, double yaw) {
    // Only log at periodic intervals (every 50th call) to avoid excessive logging
    static int update_count = 0;
    update_count++;
    
    if (update_count % 50 == 0) {
        LOG_DEBUG("Updating visualization - drone position: x=" + std::to_string(x) + 
                 ", y=" + std::to_string(y) + 
                 ", yaw=" + std::to_string(yaw));
    }
    
    if (!viz_initialized_) {
        LOG_INFO("Visualization not initialized, starting it now");
        startVisualization();
    }
    
    // We're not using these parameters directly since we're getting data from VRPN,
    // but could use them to highlight the current commanded position
}

// Get X position for a specific tracker
double OptiTrack::getXPosition(const std::string& tracker_name) {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    
    // Check if the named tracker exists
    auto it = trackers_.find(tracker_name);
    if (it != trackers_.end()) {
        const auto& tracker = it->second;
        auto now = std::chrono::system_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - tracker.last_update).count();
        
        if (time_diff < 3 && tracker.updated) {
            return tracker.x;
        } else {
            LOG_WARNING("Tracker " + tracker_name + " data too old or not updated (age: " + 
                      std::to_string(time_diff) + "s)");
        }
    } else {
        LOG_WARNING("Tracker " + tracker_name + " not found in tracker list");
    }
    
    return 0.0; // Default if tracker not found or not active
}

// Get Y position for a specific tracker
double OptiTrack::getYPosition(const std::string& tracker_name) {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    
    // Check if the named tracker exists
    auto it = trackers_.find(tracker_name);
    if (it != trackers_.end()) {
        const auto& tracker = it->second;
        auto now = std::chrono::system_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - tracker.last_update).count();
        
        if (time_diff < 3 && tracker.updated) {
            return tracker.y;
        } else {
            LOG_WARNING("Tracker " + tracker_name + " data too old or not updated (age: " + 
                      std::to_string(time_diff) + "s)");
        }
    } else {
        LOG_WARNING("Tracker " + tracker_name + " not found in tracker list");
    }
    
    return 0.0; // Default if tracker not found or not active
}

// Get Z position for a specific tracker
double OptiTrack::getZPosition(const std::string& tracker_name) {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    
    // Check if the named tracker exists
    auto it = trackers_.find(tracker_name);
    if (it != trackers_.end()) {
        const auto& tracker = it->second;
        auto now = std::chrono::system_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - tracker.last_update).count();
        
        if (time_diff < 3 && tracker.updated) {
            return tracker.z;
        } else {
            LOG_WARNING("Tracker " + tracker_name + " data too old or not updated (age: " + 
                      std::to_string(time_diff) + "s)");
        }
    } else {
        LOG_WARNING("Tracker " + tracker_name + " not found in tracker list");
    }
    
    return 0.0; // Default if tracker not found or not active
}

// Get yaw for a specific tracker
double OptiTrack::getYaw(const std::string& tracker_name) {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    
    // Check if the named tracker exists
    auto it = trackers_.find(tracker_name);
    if (it != trackers_.end()) {
        const auto& tracker = it->second;
        auto now = std::chrono::system_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - tracker.last_update).count();
        
        if (time_diff < 3 && tracker.updated) {
            // Return corrected yaw in degrees (original is in radians)
            return tracker.last_corrected_yaw * 180.0 / M_PI;
        } else {
            LOG_WARNING("Tracker " + tracker_name + " data too old or not updated (age: " + 
                      std::to_string(time_diff) + "s)");
        }
    } else {
        LOG_WARNING("Tracker " + tracker_name + " not found in tracker list");
    }
    
    return 0.0; // Default if tracker not found or not active
}

// Get the raw (uncorrected) yaw for a specific tracker
double OptiTrack::getRawYaw(const std::string& tracker_name) {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    
    // Check if the named tracker exists
    auto it = trackers_.find(tracker_name);
    if (it != trackers_.end()) {
        const auto& tracker = it->second;
        auto now = std::chrono::system_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - tracker.last_update).count();
        
        if (time_diff < 3 && tracker.updated) {
            // Return raw yaw in degrees (original is in radians)
            return tracker.last_raw_yaw * 180.0 / M_PI;
        } else {
            LOG_WARNING("Tracker " + tracker_name + " data too old or not updated (age: " + 
                      std::to_string(time_diff) + "s)");
        }
    } else {
        LOG_WARNING("Tracker " + tracker_name + " not found in tracker list");
    }
    
    return 0.0; // Default if tracker not found or not active
}

// Set up drone-to-tracker mapping
void OptiTrack::setupDroneMapping(const std::vector<DroneData>& drones) {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    drones_ = drones;
    LOG_INFO("Set up mapping for " + std::to_string(drones.size()) + " drones");
}

// Get list of all active trackers
std::vector<std::string> OptiTrack::getActiveTrackers() {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    std::vector<std::string> active_trackers;
    
    auto now = std::chrono::system_clock::now();
    for (const auto& [name, tracker] : trackers_) {
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - tracker.last_update).count();
        if (time_diff < 3 && tracker.updated) {
            active_trackers.push_back(name);
        }
    }
    
    return active_trackers;
}

// Check if a tracker is active/visible
bool OptiTrack::isTrackerActive(const std::string& tracker_name) {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    
    auto it = trackers_.find(tracker_name);
    if (it != trackers_.end()) {
        auto now = std::chrono::system_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(now - it->second.last_update).count();
        return time_diff < 3 && it->second.updated;
    }
    
    return false;
}

// Convert world coordinates to screen coordinates for visualization
cv::Point OptiTrack::worldToScreen(double x, double y, const cv::Point& center) {
    return cv::Point(
        center.x + static_cast<int>((x + offset_x_) * scale_),
        center.y - static_cast<int>((y + offset_y_) * scale_)  // Flip Y for screen coordinates
    );
}

// Convert quaternion to yaw angle
double OptiTrack::quaternionToYaw(double qw, double qx, double qy, double qz) {
    // Extract yaw from quaternion - this shows the heading direction in the XY plane
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

// Correct yaw flip (handling OptiTrack's 180° ambiguity)
double OptiTrack::correctYawFlip(double measured_yaw, double previous_yaw) {
    // Normalize angles to [-π, π]
    double normalized_current = measured_yaw;
    while (normalized_current > M_PI) normalized_current -= 2.0 * M_PI;
    while (normalized_current < -M_PI) normalized_current += 2.0 * M_PI;
    
    double normalized_prev = previous_yaw;
    while (normalized_prev > M_PI) normalized_prev -= 2.0 * M_PI;
    while (normalized_prev < -M_PI) normalized_prev += 2.0 * M_PI;
    
    // Calculate angular difference between current and previous
    double diff = normalized_current - normalized_prev;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    
    // Calculate difference if we applied a π flip
    double flipped_current = normalized_current + M_PI;
    if (flipped_current > M_PI) flipped_current -= 2.0 * M_PI;
    
    double flipped_diff = flipped_current - normalized_prev;
    while (flipped_diff > M_PI) flipped_diff -= 2.0 * M_PI;
    while (flipped_diff < -M_PI) flipped_diff += 2.0 * M_PI;
    
    // Choose the yaw value that minimizes the angular difference
    // Use absolute values for comparison
    if (std::abs(diff) < std::abs(flipped_diff)) {
        return normalized_current;
    } else {
        return flipped_current;
    }
}

// VRPN callback function
void VRPN_CALLBACK OptiTrack::handleTrackerData(void* userData, const vrpn_TRACKERCB t) {
    std::string* name = static_cast<std::string*>(userData);
    
    // This is a static method, so we need to get the OptiTrack instance
    // We'll assume there's only one OptiTrack instance for now
    static OptiTrack* optitrack_instance = nullptr;
    
    if (!optitrack_instance) {
        // Just update the data directly in the global tracker map
        // This isn't ideal, but it allows us to avoid having to pass the OptiTrack instance
        // to the callback
        LOG_DEBUG("VRPN callback for " + *name + " - OptiTrack instance not set");
        return;
    }
    
    std::lock_guard<std::mutex> lock(optitrack_instance->tracker_mutex_);
    
    // Get the current timestamp
    auto now = std::chrono::system_clock::now();
    
    // Check if tracker exists
    auto& tracker_map = optitrack_instance->trackers_;
    auto it = tracker_map.find(*name);
    if (it == tracker_map.end()) {
        LOG_WARNING("Unknown tracker in callback: " + *name);
        return;
    }
    
    TrackerData& tracker = it->second;
    bool was_updated = tracker.updated;
    
    // Update position data
    tracker.x = t.pos[0];
    tracker.y = t.pos[1];
    tracker.z = t.pos[2];
    tracker.qx = t.quat[0];
    tracker.qy = t.quat[1];
    tracker.qz = t.quat[2];
    tracker.qw = t.quat[3];
    tracker.updated = true;
    tracker.last_update = now;
    
    // Print debug info if this is first time it becomes visible
    if (!was_updated) {
        LOG_INFO("Tracker '" + *name + "' became visible! Position: (" 
              << t.pos[0] << ", " << t.pos[1] << ", " << t.pos[2] << ")");
    }
    
    // Add current position to path history (for visualization)
    cv::Point center(500, 400); // Must match the center in visualization thread
    cv::Point pos = optitrack_instance->worldToScreen(t.pos[0], t.pos[1], center);
    tracker.path.push_back(pos);
    
    // Keep path length limited
    if (tracker.path.size() > TrackerData::MAX_PATH_LENGTH) {
        tracker.path.erase(tracker.path.begin());
    }
    
    // Calculate the raw yaw from quaternion
    double raw_yaw = optitrack_instance->quaternionToYaw(t.quat[3], t.quat[0], t.quat[1], t.quat[2]);
    
    // Apply yaw flip correction if we have a prior measurement
    double corrected_yaw = raw_yaw;
    if (tracker.has_prior_yaw) {
        corrected_yaw = optitrack_instance->correctYawFlip(raw_yaw, tracker.last_corrected_yaw);
    }
    
    // Save the raw and corrected yaw values for next time
    tracker.last_raw_yaw = raw_yaw;
    tracker.last_corrected_yaw = corrected_yaw;
    tracker.has_prior_yaw = true;
    
    // Log the yaw data for analysis
    tracker.raw_yaw_log.push_back(raw_yaw);
    tracker.corrected_yaw_log.push_back(corrected_yaw);
    tracker.yaw_log_timestamps.push_back(now);
}

// Visualization thread function
void OptiTrack::visualizationThreadFunc() {
    LOG_INFO("Starting OptiTrack visualization thread");
    
    try {
        // Create visualization window with more robust error handling
        LOG_DEBUG("Creating OpenCV window - this will appear on the display");
        try {
            cv::namedWindow("Tracker Visualization", cv::WINDOW_NORMAL);
            cv::setWindowProperty("Tracker Visualization", cv::WND_PROP_TOPMOST, 1);
            LOG_INFO("Visualization window created successfully");
        } catch (const cv::Exception& e) {
            LOG_ERROR("Failed to create visualization window: " + std::string(e.what()));
            throw; // Re-throw to be caught by outer try/catch
        }
        
        // Create display buffer
        cv::Mat display(800, 1000, CV_8UC3);  // Larger display
        LOG_INFO("Visualization display buffer created successfully (800x1000)");
        
        int frame_count = 0;
        auto start_time = std::chrono::steady_clock::now();
        
        while (running_) {
            frame_count++;
            
            // Log FPS every 100 frames
            if (frame_count % 100 == 0) {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
                double fps = 1000.0 * frame_count / elapsed;
                LOG_DEBUG("Visualization running at " + std::to_string(fps) + " FPS");
                
                // Reset counters every 1000 frames to avoid overflow
                if (frame_count >= 1000) {
                    frame_count = 0;
                    start_time = now;
                }
            }
            
            // Process VRPN messages for all trackers
            for (auto& t : vrpn_trackers_) {
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
                std::lock_guard<std::mutex> lock(tracker_mutex_);
                auto now = std::chrono::system_clock::now();
                
                // Find which trackers correspond to drones
                std::map<std::string, std::string> tracker_to_drone;
                for (const auto& drone : drones_) {
                    tracker_to_drone[drone.tracker_id] = drone.name;
                }
                
                // Collect and sort tracker names
                std::vector<std::string> tracker_names;
                for (const auto& t : trackers_) {
                    tracker_names.push_back(t.first);
                }
                std::sort(tracker_names.begin(), tracker_names.end());
                
                // Draw all trackers' status and visuals
                for (const auto& name : tracker_names) {
                    const auto& tracker = trackers_[name];
                    auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(
                        now - tracker.last_update).count();
                    
                    // Status text
                    cv::Scalar status_color = time_diff < 3 ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);
                    std::string display_name = tracker_to_drone.count(name) ? 
                                             tracker_to_drone[name] + " (" + name + ")" : name;
                    
                    std::string status_text = display_name + ": " + (time_diff < 3 ? 
                        "Active (" + std::to_string(tracker.x) + ", " + std::to_string(tracker.y) + ")" : 
                        "No Signal");
                    cv::putText(display, status_text, cv::Point(10, text_y), 
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, status_color);
                    text_y += 20;
                    
                    // Only draw visible trackers
                    if (time_diff < 3) {
                        // Draw tracker position
                        cv::Point pos = worldToScreen(tracker.x, tracker.y, center);
                        
                        // Draw path history
                        if (tracker.path.size() > 1) {
                            // Draw lines between consecutive points with fading transparency
                            for (size_t i = 1; i < tracker.path.size(); i++) {
                                // Calculate alpha based on position in path (newer points are more visible)
                                double alpha = std::min(1.0, 0.3 + 0.7 * (static_cast<double>(i) / tracker.path.size()));
                                cv::Scalar pathColor = tracker.color * alpha;
                                
                                // Draw line segment
                                cv::line(display, tracker.path[i-1], tracker.path[i], pathColor, 1);
                            }
                        }
                        
                        // Draw orientation arrow showing yaw (heading) direction
                        double yaw = tracker.last_corrected_yaw;
                        
                        // Convert yaw angle to a direction vector in XY plane
                        double vx = std::cos(yaw);
                        double vy = std::sin(yaw);
                        
                        // Arrow length in pixels
                        double length = 20.0;
                        
                        // Draw arrow
                        cv::Point arrowEnd(pos.x + static_cast<int>(vx * length), 
                                          pos.y - static_cast<int>(vy * length)); // Flip Y for screen coords
                        cv::arrowedLine(display, pos, arrowEnd, tracker.color, 2, cv::LINE_AA, 0, 0.3);
                        
                        // Draw position circle
                        cv::circle(display, pos, 5, tracker.color, -1);
                        
                        // Draw position text
                        std::string pos_text = name + " Z:" + std::to_string(tracker.z);
                        cv::putText(display, pos_text, pos + cv::Point(10, 10), 
                                  cv::FONT_HERSHEY_SIMPLEX, 0.5, tracker.color);
                        
                        // Draw yaw text
                        int raw_yaw_deg = static_cast<int>(tracker.last_raw_yaw * 180.0 / M_PI);
                        int corr_yaw_deg = static_cast<int>(tracker.last_corrected_yaw * 180.0 / M_PI);
                        std::string yaw_text = "Yaw: " + std::to_string(corr_yaw_deg) + "° ";
                        
                        if (tracker_to_drone.count(name) > 0) {
                            // Find the drone's yaw offset
                            for (const auto& drone : drones_) {
                                if (drone.tracker_id == name) {
                                    yaw_text += "(Offset: " + std::to_string(static_cast<int>(drone.yaw_offset)) + "°)";
                                    break;
                                }
                            }
                        }
                        
                        cv::putText(display, yaw_text, pos + cv::Point(10, 25), 
                                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
                    }
                }
            }
            
            cv::imshow("Tracker Visualization", display);
            
            char key = cv::waitKey(1);
            if (key == 27) // ESC
                running_ = false;
            else if (key == '+' || key == '=') {
                // Increase scale
                const_cast<double&>(scale_) *= 1.1;
            }
            else if (key == '-' || key == '_') {
                // Decrease scale
                const_cast<double&>(scale_) /= 1.1;
            }
            else if (key == 'c' || key == 'C') {
                // Clear all paths
                std::lock_guard<std::mutex> lock(tracker_mutex_);
                for (auto& tracker_pair : trackers_) {
                    tracker_pair.second.path.clear();
                }
            }
            
            // Small delay to avoid consuming too much CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        LOG_INFO("Visualization thread exiting, destroying windows");
        cv::destroyAllWindows();
    }
    catch (const cv::Exception& e) {
        LOG_ERROR("OpenCV exception in visualization thread: " + std::string(e.what()));
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in visualization thread: " + std::string(e.what()));
    }
    catch (...) {
        LOG_ERROR("Unknown error in visualization thread");
    }
    
    LOG_INFO("Visualization thread terminated");
}