#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <TelloCpp.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class TelloController {
private:
    std::unique_ptr<TelloCpp> tello;
    cv::VideoWriter videoWriter;
    std::atomic<bool> is_flying{false};
    std::atomic<bool> should_quit{false};
    
    // Non-blocking keyboard input setup
    int kbhit() {
        struct termios oldt, newt;
        int ch;
        int oldf;
        
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
        
        ch = getchar();
        
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);
        
        if(ch != EOF) {
            ungetc(ch, stdin);
            return 1;
        }
        return 0;
    }

public:
    TelloController() : tello(std::make_unique<TelloCpp>()) {
        // Initialize video writer
        videoWriter.open("tello_recording.mp4", 
                        cv::VideoWriter::fourcc('m','p','4','v'),
                        30.0,
                        cv::Size(960, 720));
    }

    bool initialize() {
        if (!tello->connect()) {
            std::cerr << "Failed to connect to Tello\n";
            return false;
        }
        
        if (!tello->startVideo()) {
            std::cerr << "Failed to start video stream\n";
            return false;
        }

        std::cout << "Connected to Tello!\n";
        std::cout << "Battery: " << tello->getBattery() << "%\n";
        return true;
    }

    void run() {
        int speed = 50;
        int lr = 0, fb = 0, ud = 0, yv = 0;

        std::cout << "\nControls:\n";
        std::cout << "T: Takeoff | L: Land | Q: Quit\n";
        std::cout << "I/K/J/H: Forward/Back/Left/Right\n";
        std::cout << "W/S: Up/Down | A/D: Rotate\n\n";

        while (!should_quit) {
            // Get and record video frame
            auto frame = tello->getFrame();
            if (!frame.empty()) {
                videoWriter.write(frame);
                
                // Display battery and status
                std::string status = "Battery: " + std::to_string(tello->getBattery()) + "%";
                cv::putText(frame, status, cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                
                cv::imshow("Tello Feed", frame);
                cv::waitKey(1);
            }

            // Handle keyboard input
            if (kbhit()) {
                char key = getchar();
                
                // Reset movement values
                lr = fb = ud = yv = 0;

                switch (key) {
                    case 't':
                        if (!is_flying) {
                            std::cout << "Taking off...\n";
                            tello->takeoff();
                            is_flying = true;
                        }
                        break;
                    case 'l':
                        if (is_flying) {
                            std::cout << "Landing...\n";
                            tello->land();
                            is_flying = false;
                        }
                        break;
                    case 'w': ud = speed; break;
                    case 's': ud = -speed; break;
                    case 'a': yv = -speed; break;
                    case 'd': yv = speed; break;
                    case 'i': fb = speed; break;
                    case 'k': fb = -speed; break;
                    case 'j': lr = -speed; break;
                    case 'h': lr = speed; break;
                    case 'q':
                        should_quit = true;
                        break;
                }

                if (is_flying) {
                    tello->setRC(lr, fb, ud, yv);
                }
            }

            // Small sleep to prevent CPU overload
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        // Cleanup
        if (is_flying) {
            tello->land();
        }
    }

    ~TelloController() {
        videoWriter.release();
        cv::destroyAllWindows();
    }
};

int main() {
    TelloController controller;
    
    if (!controller.initialize()) {
        return -1;
    }
    
    controller.run();
    return 0;
}