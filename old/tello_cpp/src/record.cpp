#include "ctello.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <filesystem>
#include <ctime>
#include <fstream>

const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};
const cv::Point2i TELLO_CENTER(480, 360);
const std::string RECORDING_DIR = std::string(getenv("HOME")) + "/squawkblock/squawkblock/recordings/";

enum class DroneState {
    IDLE,
    MOVING,
    LANDING,
    TAKING_OFF
};

class KeyboardInput {
private:
    struct termios orig_termios;
    
public:
    KeyboardInput() {
        tcgetattr(STDIN_FILENO, &orig_termios);
        struct termios raw = orig_termios;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }
    
    ~KeyboardInput() {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
    }
    
    char getKey() {
        char c;
        int result = read(STDIN_FILENO, &c, 1);
        if (result < 0) {
            return -1;
        }
        return result ? c : -1;
    }
};

std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
    return ss.str();
}

void DrawFlightData(cv::Mat& frame, const std::string& command, bool busy) {
    std::stringstream ss;
    ss << "Command: " << command;
    cv::putText(frame, ss.str(), cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 2);

    cv::putText(frame, busy ? "BUSY" : "READY", cv::Point(10, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.75, 
                busy ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);
}

void DrawMovementIndicator(cv::Mat& frame, int lr, int fb, int ud, int yaw) {
    if (lr != 0 || fb != 0) {
        cv::Point2i direction(lr, fb);
        cv::arrowedLine(frame, TELLO_CENTER, 
                       TELLO_CENTER + direction * 2,
                       cv::Scalar(0, 0, 255), 2);
    }
    
    if (yaw != 0) {
        cv::circle(frame, TELLO_CENTER, 30, cv::Scalar(255, 0, 0), 2);
        cv::arrowedLine(frame, TELLO_CENTER,
                       cv::Point2i(TELLO_CENTER.x + yaw, TELLO_CENTER.y),
                       cv::Scalar(255, 0, 0), 2);
    }

    if (ud != 0) {
        cv::line(frame, 
                cv::Point(TELLO_CENTER.x, TELLO_CENTER.y - 40),
                cv::Point(TELLO_CENTER.x, TELLO_CENTER.y + 40),
                cv::Scalar(0, 255, 0), 2);
        cv::circle(frame, 
                  cv::Point(TELLO_CENTER.x, TELLO_CENTER.y + (ud > 0 ? 40 : -40)),
                  5, cv::Scalar(0, 255, 0), -1);
    }
}

std::string cleanCommand(const std::string& cmd) {
    // Remove any non-alphanumeric characters except spaces and ?
    std::string clean;
    for(char c : cmd) {
        if (isalnum(c) || c == ' ' || c == '?' || c == '-') {
            clean += c;
        }
    }
    return clean;
}

int main() {
    KeyboardInput keyboard;
    
    ctello::Tello tello;
    if (!tello.Bind()) {
        std::cerr << "Failed to bind to Tello!" << std::endl;
        return 1;
    }

    // Create recordings directory if it doesn't exist
    std::filesystem::create_directories(RECORDING_DIR);
    
    // Generate timestamp for this session
    std::string timestamp = getTimestamp();

    // Start video stream
    tello.SendCommand("streamon");
    while (!tello.ReceiveResponse());

    // Initialize video capture
    cv::VideoCapture capture{TELLO_STREAM_URL, cv::CAP_FFMPEG};
    if (!capture.isOpened()) {
        std::cerr << "Failed to open video stream" << std::endl;
        return 1;
    }

    // Set up video writer with timestamp
    std::string videoFile = RECORDING_DIR + "tello_" + timestamp + ".avi";
    cv::VideoWriter videoWriter(videoFile, 
                              cv::VideoWriter::fourcc('M','J','P','G'),
                              30, cv::Size(960, 720));

    // Set up telemetry log file
    std::string telemetryFile = RECORDING_DIR + "telemetry_" + timestamp + ".csv";
    std::ofstream telemetryLog(telemetryFile);
    telemetryLog << "timestamp,state\n";

    std::cout << "Controls:\n";
    std::cout << "T: Takeoff\n";
    std::cout << "L: Land\n";
    std::cout << "I/K/J/H: Forward/Back/Left/Right\n";
    std::cout << "W/S: Up/Down\n";
    std::cout << "A/D: Rotate Left/Right\n";
    std::cout << "Space: Hover/Stop\n";
    std::cout << "Q: Quit\n\n";

    // Initialize control state
    DroneState drone_state = DroneState::IDLE;
    bool busy = false;
    std::string current_command;
    int current_lr = 0, current_fb = 0, current_ud = 0, current_yaw = 0;
    const int speed = 50;

    while (true) {
        // Handle response if available
        if (auto response = tello.ReceiveResponse()) {
            std::cout << "Tello: " << *response << std::endl;
            busy = false;
        }

        // Get current timestamp
        uint64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // Get and process video frame
        cv::Mat frame;
        if (capture.read(frame)) {
            if (!frame.empty()) {
                // Log telemetry
                if (auto state = tello.GetState()) {
                    telemetryLog << current_time << "," << *state << "\n";
                    telemetryLog.flush();
                }

                // Save frame
                videoWriter.write(frame);

                // Draw overlays
                DrawFlightData(frame, current_command, busy);
                DrawMovementIndicator(frame, current_lr, current_fb, current_ud, current_yaw);
                
                // Add timestamp to frame
                cv::putText(frame, 
                           "Frame Time: " + std::to_string(current_time),
                           cv::Point(10, 90), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.75, 
                           cv::Scalar(0, 255, 0), 2);

                // Display frame
                cv::resize(frame, frame, cv::Size(), 0.75, 0.75);
                cv::imshow("Tello", frame);
                cv::waitKey(1);
            }
        }

        // Handle keyboard input
        char key = keyboard.getKey();
        if (key != -1 && !busy) {
            switch (key) {
                case 't': // takeoff
                    if (drone_state == DroneState::IDLE) {
                        current_command = "takeoff";
                        tello.SendCommand(cleanCommand(current_command));
                        drone_state = DroneState::TAKING_OFF;
                        busy = true;
                    }
                    break;
                case 'l': // land
                    current_command = "land";
                    tello.SendCommand(cleanCommand(current_command));
                    drone_state = DroneState::LANDING;
                    busy = true;
                    break;
                case 'w': // up
                    current_ud = speed;
                    current_lr = current_fb = current_yaw = 0;
                    drone_state = DroneState::MOVING;
                    break;
                case 's': // down
                    current_ud = -speed;
                    current_lr = current_fb = current_yaw = 0;
                    drone_state = DroneState::MOVING;
                    break;
                case 'a': // rotate left
                    current_yaw = -speed;
                    current_lr = current_fb = current_ud = 0;
                    drone_state = DroneState::MOVING;
                    break;
                case 'd': // rotate right
                    current_yaw = speed;
                    current_lr = current_fb = current_ud = 0;
                    drone_state = DroneState::MOVING;
                    break;
                case 'i': // forward
                    current_fb = speed;
                    current_lr = current_ud = current_yaw = 0;
                    drone_state = DroneState::MOVING;
                    break;
                case 'k': // backward
                    current_fb = -speed;
                    current_lr = current_ud = current_yaw = 0;
                    drone_state = DroneState::MOVING;
                    break;
                case 'j': // left
                    current_lr = -speed;
                    current_fb = current_ud = current_yaw = 0;
                    drone_state = DroneState::MOVING;
                    break;
                case 'h': // right
                    current_lr = speed;
                    current_fb = current_ud = current_yaw = 0;
                    drone_state = DroneState::MOVING;
                    break;
                case ' ': // stop/hover
                    current_lr = current_fb = current_ud = current_yaw = 0;
                    current_command = "rc 0 0 0 0";
                    tello.SendCommand(cleanCommand(current_command));
                    drone_state = DroneState::IDLE;
                    break;
                case 'q': // quit
                    std::cout << "Landing and quitting..." << std::endl;
                    tello.SendCommand("land");
                    while (!tello.ReceiveResponse());
                    tello.SendCommand("streamoff");
                    goto cleanup;
            }
            
            // Send updated movement command if we're in moving state
            if (drone_state == DroneState::MOVING) {
                std::stringstream ss;
                ss << "rc " << current_lr << " " << current_fb << " " << current_ud << " " << current_yaw;
                current_command = ss.str();
                tello.SendCommand(cleanCommand(current_command));
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

cleanup:
    videoWriter.release();
    telemetryLog.close();
    cv::destroyAllWindows();
    return 0;
}