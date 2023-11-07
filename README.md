# solo12_robot

Base code for Nala robot (SOLO12 robot from Open Dynamic Robot Initiative)

## Launch Gazebo

To launch the robot in gazebo (prototyping purposes):

```
ros2 launch solo12_bringup robot.launch.py
```

## Manual Installation

Requeriments:
- curl
- git

1. Git clone `master-board`

```
git clone https://github.com/a2s-institute/master-board.git
```

2. Install the master-board SDK

```
cd master-board/sdk/master_board_sdk
mkdir build && cd build
cmake -DBUILD_PYTHON_INTERFACE=ON -DCMAKE_BUILD_TYPE=RELEASE ..
make
sudo make install
```

3. Install Gazebo 11 (Classic)

```
curl -sSL http://get.gazebosim.org | sh
```

this one-liner come from [Install Gazebo using Ubuntu packages](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
