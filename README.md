# joystick
Driver code for DXS joystick

## System Dependencies

1. [glog](https://github.com/google/glog)
2. [gflags](https://github.com/gflags/gflags)
3. [Lua5.2](https://www.lua.org/)

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

## Recording

Run `ros2 launch av_recorder recorder.launch.py` alongside the joystick (the Memo deployment tmux layout
does this). Recording commands are published by `joystick/src/joystick.cc` on `/record` (`std_msgs/Bool`):
true starts a bag, false stops and flushes it. `/recording` (`std_msgs/Int32`, 0/1) synchronizes toggle state.

The default DXS A button toggles recording once per press. With `Ps4Mapping`, hold the left bumper and
press triangle to start or circle to stop. Separate start/stop buttons issue idempotent commands;
holding a button does not repeat it. Raw `/joystick` buttons are published unchanged.

The recorder owns storage, QoS and topic selection through `src/av_recorder/config/recorder.yaml`.
The legacy Lua `record_cmd` is not executed. Default recordings go to `/data/arthur/bags` as MCAP,
including all published topics and hidden topics, with discovery for late-starting nodes.

