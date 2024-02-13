#include "gazebo_interface.h"
#include <csignal>
#include <atomic>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

std::atomic<bool> running(true);

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        running = false;
    }
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    GazeboInterface gazeboInterface;
    gazeboInterface.init("/gazebo/solo12/robot/joint_control_cmd", "/gazebo/solo12/joint_states");

    gazeboInterface.onFeedbackReceived([](const std::string& feedback) {
        std::cout << "Feedback: " << feedback << std::endl;
    });

    std::string command;
    // Main loop to listen for stdin commands
    while (running && std::getline(std::cin, command)) {
        if (!command.empty()) {
            gazeboInterface.publishCommand(command);
            std::cout << "Command received and published: " << command << std::endl;
        }
    }

    return 0;
}
