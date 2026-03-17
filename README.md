# joystick
Driver code for DXS joystick

## System Dependencies

1. [glog](https://github.com/google/glog)
2. [gflags](https://github.com/gflags/gflags)
3. [Lua5.1](https://www.lua.org/)

## ROS2 Dependencies

1. [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
2. [config_reader](https://github.com/ut-amrl/config-reader)
3. [amrl_shared_lib](https://github.com/ut-amrl/amrl_shared_lib)

## Build

1. Clone the repository to your ROS2 workspace
2. Build the dependencies and `joystick` package
```bash
colcon build --packages-select config_reader amrl_shared_lib joystick
```
3. Source the workspace
```bash
source install/setup.bash
```

## Run

Run the node with the config file (default: `joystick/config/joystick.lua`) in a ROS2 compatible terminal.

```bash
ros2 run joystick joystick_node --config=joystick/config/joystick.lua
```

## Controller Mapping Definitions [TODO]

