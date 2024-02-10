### Hardware Interfaces

**An Overview**
- state_interfaces are read only data(handles only sensory readings)
- command_interfaces are read and write data handles that harware commands like setting a joint velocity reference.

**Code blocks required for writing a new hardware interface**
- on_init: ros2_control initialization, hardware setup, dynamic memory allocation. 
	- The SystemInterface::on_init(info) calls out object with specifics from the URDF. Example info object has fields for joints,sensors,gpios and more.
- export_state_interfaces: It returns a vector from the stateInterface(**describing the state_interface** for each joint) These are *read_only* data handles  
- export_command_interfaces: It returns the vector of the command interface for each joint
- read: It is responsible for the updating the data_Values of the state_interfaces
- write: It is also responsible for updating the values for the command_interfaces, but this sends the corresponding signals to the hardware.


