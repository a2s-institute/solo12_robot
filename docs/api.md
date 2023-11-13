# API Specification (solodev)

**Status**: Draft
**Version**: 0.0.0

This API provides the methods to interact with SOLO 12 robot physically or in a simulation environment.

## `Solo` Object

- `Solo(interface)`: Intantize an object of type Solo, which can be the physical robot or the robot in simulation.
    - `interface` this can be a hardware interface (in case of the physical robot) or an IP interface (for simulation)

## Methods

- `SendCommand(set_of_commands)`: Send control commands (TBD, define the type of command)
- `GetVersion()`: Returns API Version
- `GetAccelerometer()`: Returns the last accelerometer reading
- `GetGyroscope()`: Returns the last Gyroscope reading
- `GetAlttitude()`: Returns the last Alttitude reading
- `GetCurrent()`: Returns the voltage drop
- `GetVoltage()`: Returns the current board voltage
- `GetPower()`: Returns the current power consumption.
