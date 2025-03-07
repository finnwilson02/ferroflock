#pragma once
#include <iostream>
#include <fstream>
#include <string>

namespace FileWriter {
    // Save drone-to-tracker mapping to file
    bool saveTrackerMapping(const std::string& mac, const std::string& trackerId);
    
    // Create a position log file and return the file stream
    std::ofstream createPositionLog(const std::string& ip);
    
    // Log a position entry to an open file stream
    bool logPosition(std::ofstream& file, const std::string& trackerId, 
                     float x, float y, float z);
}