#ifndef OPTITRACK_VIZ_H
#define OPTITRACK_VIZ_H

// Update the OptiTrack visualization with current position and orientation
void update_optitrack_viz(double x, double y, double yaw);

// Start the OptiTrack visualization system (called automatically by update_optitrack_viz)
void start_visualization();

// Stop the OptiTrack visualization system
void stop_visualization();

// Get OptiTrack X position from current tracking data
double get_optitrack_x();

// Get OptiTrack Y position from current tracking data
double get_optitrack_y();

// Get OptiTrack yaw from current tracking data
double get_optitrack_yaw();

#endif