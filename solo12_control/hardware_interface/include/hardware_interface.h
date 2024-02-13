#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include <string>
#include "master_board_sdk/master_board_interface.h"

class HardwareInterface {
public:
    // Constructor: Takes the name of the interface to connect to.
    explicit HardwareInterface(const std::string& interface_name);

    // Destructor: Ensures proper disconnection and cleanup.
    ~HardwareInterface();

    // Initialize the hardware interface and motor drivers.
    void Init();

    // Apply control inputs to the motors.
    // Parameters are example control inputs; adjust according to your needs.
    void ApplyControl(double position, double velocity, double torque);

private:
    MasterBoardInterface robot_if; // Interface to the master board
    std::string if_name;           // Name of the interface to connect to
};

#endif // HARDWARE_INTERFACE_H
