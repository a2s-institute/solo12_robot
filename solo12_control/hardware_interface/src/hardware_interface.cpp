#include "hardware_interface.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <signal.h>
#include <stdexcept>
#include <thread>

// Constructor
HardwareInterface::HardwareInterface(const std::string& interface_name) 
    : if_name(interface_name), stop(false) {
    signal(SIGINT, HardwareInterface::StopSignalHandler);
}

// Destructor
HardwareInterface::~HardwareInterface() {
    robot_if.Stop(); // Ensure the interface is properly stopped
}

// Signal handler to gracefully stop the interface
void HardwareInterface::StopSignalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "Stopping Hardware Interface..." << std::endl;
        stop = true;
    }
}

// Initialization of the Hardware Interface
void HardwareInterface::Init() {
    robot_if.Init(if_name.c_str()); // Initialize the interface
    for (int i = 0; i < N_SLAVES_CONTROLED; i++) {
        // Motor initial setup
        robot_if.motor_drivers[i].motor1->SetCurrentReference(0);
        robot_if.motor_drivers[i].motor2->SetCurrentReference(0);
        robot_if.motor_drivers[i].motor1->Enable();
        robot_if.motor_drivers[i].motor2->Enable();

        // Configure PD controllers
        ConfigurePDController(i, kp, kd, iq_sat);

        robot_if.motor_drivers[i].EnablePositionRolloverError();
        robot_if.motor_drivers[i].SetTimeout(5);
        robot_if.motor_drivers[i].Enable();
    }
}

// Main loop to control the hardware
void HardwareInterface::Run() {
    while (!stop && !robot_if.IsTimeout()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Sleep to simulate control loop delay
        robot_if.ParseSensorData(); // Parse sensor data

        // Example control logic (implement your specific control logic here)
        ApplySinusoidalPositionControl();
        robot_if.SendCommand(); // Send commands to the motors
    }

    if (robot_if.IsTimeout()) {
        std::cerr << "Masterboard timeout detected. Check the connection." << std::endl;
    }
}

// Configure PD controllers for each motor
void HardwareInterface::ConfigurePDController(int motor_index, double kp, double kd, double iq_sat) {
    robot_if.motor_drivers[motor_index].motor1->set_kp(kp);
    robot_if.motor_drivers[motor_index].motor2->set_kp(kp);
    robot_if.motor_drivers[motor_index].motor1->set_kd(kd);
    robot_if.motor_drivers[motor_index].motor2->set_kd(kd);
    robot_if.motor_drivers[motor_index].motor1->set_current_sat(iq_sat);
    robot_if.motor_drivers[motor_index].motor2->set_current_sat(iq_sat);
}

// Example control logic: Apply a sinusoidal position control to the motors
void HardwareInterface::ApplySinusoidalPositionControl() {
    static auto start_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    double t = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time).count();

    for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++) {
        if (!robot_if.motor_drivers[i / 2].is_connected) continue; // Skip disconnected slaves

        double ref = init_pos[i] + amplitude * std::sin(2 * M_PI * freq * t);
        double v_ref = 2 * M_PI * freq * amplitude * std::cos(2 * M_PI * freq * t);
        
        robot_if.motors[i].SetCurrentReference(0.);
        robot_if.motors[i].SetPositionReference(ref);
        robot_if.motors[i].SetVelocityReference(v_ref);
    }
}
