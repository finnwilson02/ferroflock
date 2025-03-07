#ifndef TEST_OPTITRACK_LOGGING_H
#define TEST_OPTITRACK_LOGGING_H

#include <string>
#include "optitrack.h"

// Function declarations for test_optitrack_logging.cpp
std::string selectActiveTracker(OptiTrack& optitrack);
void signalHandler(int signal);
void cleanup();

#endif // TEST_OPTITRACK_LOGGING_H