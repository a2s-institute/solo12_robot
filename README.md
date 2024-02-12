# solo12_robot

![build status main](https://github.com/a2s-institute/solo12_robot/actions/workflows/main.yaml/badge.svg)

Base code for Nala robot (SOLO12 robot from Open Dynamic Robot Initiative)

## Installation

Install the required dependencies by runing the following command the in ROOT directory of your workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Usage

To launch the system use:

```bash
ros2 launch solo12_bringup robot.launch.py
```

to get more details of the availables launch arguments see [solo12_bringup](./solo12_bringup/README.md) package.

## Packages

For information about packages, please refer the `README.md` file of each package.
