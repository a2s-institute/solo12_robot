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

Launch arguments:
- `stand`: Spawn robot on stand (default false).
- `use_rviz`: Run rviz (default true).

Example:

```
ros2 launch solo12_bringup robot.launch.py stand:=true use_rviz:=false
```

## Packages

For information about packages, please refer the `README.md` file of each package.
