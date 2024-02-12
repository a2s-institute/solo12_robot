# `solo12_bringup`

This is the main package to launch all the components.

## Usage: `robot.launch.py`

To launch the full system is `robot.launch.py`, example:

```bash
ros2 launch solo12_bringup robot.launch.py
```

Launch arguments:
- `use_sim`: Start Gazebo (default true), otherwise start hardware.
- `start_rviz`: Run rviz (default true).
- `rviz_config_file`: Custom rviz config file (default `solo12_description/rviz/solo12.rviz`)
- `robot_name`: Robot name (default "solo12"). For now just set the gazebo entity name, but this can be used later to spawn multiples robot.
- `stand`: Spawn robot on stand (default true).

Example:

```
ros2 launch solo12_bringup robot.launch.py stand:=true use_rviz:=false
```

## `state_publisher.launch.py`

This launch file is the **main and only** source for the robot_description. If you want to make use of the `robot_description`, please subscribe to the topic `/robot_description`.

Launch arguments:
- `use_sim_time`: Sync ros and gazebo time (default true)

## `controllers.launch.py`

Launch all the components related with `ros2_control`.

Launch arguments:
- `use_sim_time`: Sync ros and gazebo time (default true)

## `sim_gazebo.launch.py`

Start the gazebo simulation environment. Spawn the robot and the stand.

Launch arguments:
- `stand`: spawn stand model into gazebo (default true).
- `robot_name`: robot entity name in gazebo (default "solo12")

## `start_rviz.launch.py`

Start rviz.

Launch arguments:
- `rviz_config_file`: Custom cofiguration file to start rviz (no default value)
