#include <cstdlib>
#include <iostream>

int main() {
    // Execute the combined script
    system("bash ~/sdp_ws/src/solo12_robot/script/setup_and_launch.sh");

    std::cout << "Setup and launch process initiated." << std::endl;
    return 0;
}

