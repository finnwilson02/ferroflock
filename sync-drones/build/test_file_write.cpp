#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <cerrno>

int main() {
    // Simple test to write a file
    std::string filename = "/home/finn/squawkblock/squawkblock/sync-drones/build/test_output.txt";
    
    std::cout << "Testing file write to: " << filename << std::endl;
    
    // Method 1: Simple write
    {
        std::ofstream file(filename);
        if (file.is_open()) {
            file << "Test line 1" << std::endl;
            file << "Test line 2" << std::endl;
            file.flush();
            file.close();
            std::cout << "Method 1: Write successful" << std::endl;
        } else {
            std::cerr << "Method 1: Failed to open file. Error: " << strerror(errno) << std::endl;
        }
    }
    
    // Method 2: Verify the file was written
    {
        std::ifstream file(filename);
        if (file.is_open()) {
            std::string line;
            std::cout << "File contents:" << std::endl;
            while (std::getline(file, line)) {
                std::cout << "  " << line << std::endl;
            }
            file.close();
        } else {
            std::cerr << "Failed to open file for reading. Error: " << strerror(errno) << std::endl;
        }
    }
    
    // Method 3: Append to the file
    {
        std::ofstream file(filename, std::ios::app);
        if (file.is_open()) {
            file << "Appended line 1" << std::endl;
            file << "Appended line 2" << std::endl;
            file.flush();
            file.close();
            std::cout << "Method 3: Append successful" << std::endl;
        } else {
            std::cerr << "Method 3: Failed to open file for append. Error: " << strerror(errno) << std::endl;
        }
    }
    
    // Method 4: Verify append worked
    {
        std::ifstream file(filename);
        if (file.is_open()) {
            std::string content((std::istreambuf_iterator<char>(file)), 
                           std::istreambuf_iterator<char>());
            std::cout << "Final file size: " << content.length() << " bytes" << std::endl;
            file.close();
        } else {
            std::cerr << "Failed to open file for final verification. Error: " << strerror(errno) << std::endl;
        }
    }
    
    // Method 5: Try a different file in the same directory
    {
        std::string filename2 = "/home/finn/squawkblock/squawkblock/sync-drones/build/test_output2.txt";
        std::ofstream file(filename2);
        if (file.is_open()) {
            file << "Test in second file" << std::endl;
            file.close();
            std::cout << "Method 5: Second file write successful" << std::endl;
        } else {
            std::cerr << "Method 5: Failed to write second file. Error: " << strerror(errno) << std::endl;
        }
    }
    
    // Method 6: Try the calibration directory
    {
        std::string calibDir = "/home/finn/squawkblock/squawkblock/sync-drones/build/calibration";
        std::string calibFile = calibDir + "/test_write.txt";
        
        std::ofstream file(calibFile);
        if (file.is_open()) {
            file << "Test write in calibration directory" << std::endl;
            file.close();
            std::cout << "Method 6: Calibration directory write successful" << std::endl;
        } else {
            std::cerr << "Method 6: Failed to write to calibration directory. Error: " << strerror(errno) << std::endl;
        }
    }
    
    return 0;
}