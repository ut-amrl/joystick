-- record_cmd = "mkdir -p ~/cotnav_ws/data/bags; ros2 bag record -o ~/cotnav_ws/data/bags/$(date +%Y-%m-%d_%H-%M-%S) "..
--     "/status "..
--     "/autonomy_arbiter/enabled "..
--     "/velodyne_points "..
--     "/imu "..
--     "/odometry "..
--     "/tf "..
--     "/tf_static "..
--     "/cmd_vel "..
--     "/joint_states "..
--     "/joystick "..
--     "/joy "..
--     "/odom "..
--     "/vectornav/imu "..
--     "/vectornav/gps "..
--     "/vectornav/magnetic "..
--     "/rgb/image_raw "..
--     "/rgb/camera_info "..
--     "/depth_to_rgb/image_raw "..
--     "/depth_to_rgb/camera_info "..
--     "/depth/frontright/image/compressed "..
--     "/depth/frontleft/image/compressed "..
--     "/depth_registered/back/camera_info "..
--     "/depth_registered/back/image "..
--     "/depth_registered/frontleft/camera_info "..
--     "/depth_registered/frontleft/image "..
--     "/depth_registered/frontright/camera_info "..
--     "/depth_registered/frontright/image "..
--     "/depth_registered/left/camera_info "..
--     "/depth_registered/left/image "..
--     "/depth_registered/right/camera_info "..
--     "/depth_registered/right/image "..
--     "/spot/camera/back/camera_info > /tmp/joystick_rosbag.log 2>&1 &";

record_cmd = "mkdir -p /home/ros/cotnav_ws/data/bags; ros2 bag record -o /home/ros/cotnav_ws/data/bags/$(date +%Y-%m-%d_%H-%M-%S) "..
    "/status "..
    "/autonomy_arbiter/enabled "..
    "/velodyne_points "..
    "/imu "..
    "/odometry "..
    "/tf "..
    "/tf_static "..
    "/cmd_vel "..
    "/joint_states "..
    "/joystick "..
    "/joy "..
    "/odom "..
    "/vectornav/imu "..
    "/vectornav/gps "..
    "/vectornav/magnetic "..
    "/rgb/image_raw "..
    "/rgb/camera_info "..
    "/depth_to_rgb/image_raw "..
    "/depth_to_rgb/camera_info > /tmp/joystick_rosbag.log 2>&1 &";

Ps4Mapping = {
    manual_button = 4;
    autonomous_button = 5;
    manual_autonomous_axis = -1;

    sit_button = 3;
    stand_button = 0;
    sit_stand_axis = -1;

    x_axis = 4;
    y_axis = 3;
    r_axis = 0;
    axis_scale = -1;

    left_bumper = 4;
    record_start_button = 2;
    record_stop_button = 1;
};

SpektrumDxsMapping = {
--[[
Left Side:
    A button: button#0, also turns axis#5 from positive to negative
        axis#5 does not appear to map to anything else
    B switch: axis#4
        0 -> positive value
        1 -> zero value
        2 -> negative value
    D switch: axis#6
        0 -> positive value
        1 -> zero value
        2 -> negative value, also toggles button#1
    Left Joystick
        Left-Right: axis#0
            left -> positive
            right -> negative
        Up-Down: axis#1, only active if the H switch is set to 0
            up -> positive
            down -> negative

Right Side:
    H switch: enable/disable axis#1 joystick
        0 -> enable
        1 -> disable, arbitrary negative value reported
    F switch: reduce joystick values
        0 -> full range (-23638 to +23637 at neutral zero point)
        1 -> reduced range (about 70%)
    Right Joystick:
        Left-Right: axis #2
            left -> positive
            right -> negative
        Up-Down: axis#3
            up -> positive
            down -> negative

Four Trim Sliders: adjust zero point of joysticks
    Higher-pitch chirp at neutral zero point.
--]]
    manual_button = -1;
    autonomous_button = -1;
    manual_autonomous_axis = 6;

    sit_button = -1;
    stand_button = -1;
    sit_stand_axis = 4;

    x_axis = 3;
    y_axis = 2;
    r_axis = 0;
    axis_scale = 32768 / 23638;

    left_bumper = 0;
    record_start_button = 0;
    record_stop_button = 0;
};

Mapping = SpektrumDxsMapping;