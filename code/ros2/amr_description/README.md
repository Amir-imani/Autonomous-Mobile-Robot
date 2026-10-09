# AMR Description

`amr_description` contains the measured Xacro model, TF topology, and RViz2
visualization configuration for the differential-drive AMR.

## Frame architecture

The runtime localization authority publishes `odom -> base_footprint` and the
live IMU tilt transform `base_footprint -> base_link`. `robot_state_publisher`
owns the remaining transforms below `base_link`:

```text
odom
└── base_footprint
    └── base_link
        ├── upper_deck_link
        ├── left_wheel_link
        ├── right_wheel_link
        ├── front_caster_link
        ├── rear_caster_link
        ├── imu_link
        ├── laser_mount_link
        │   ├── laser_link
        │   └── laser_servo_link
        │       └── laser_sensor_link
        └── camera_tilt_mount_link
            └── camera_link
                └── camera_optical_frame
```

`laser_link` is the fixed frame for complete 180-degree `LaserScan` messages.
`laser_servo_link` models the instantaneous physical servo angle and must not be
used as the frame of the assembled scan. This separation prevents double rotation.

The primary Xacro defaults `base_footprint_joint` to `fixed` for offline display.
The production launch selects `base_joint_type:=floating`; robot state publisher
then leaves that transform to `differential_odometry_node`, preventing duplicate
TF authorities while allowing live roll and pitch.

## Build and visualize

Deploy this directory to `/home/amirali/robot_ws/src/amr_description`, then run:

```bash
cd /home/amirali/robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select amr_description --symlink-install
source install/setup.bash
ros2 launch amr_description display.launch.py
```

The camera tilt joint permits 20 degrees upward and 70 degrees downward. The
LiDAR servo joint permits a 180-degree sweep centered on the forward direction.

Component masses, caster dimensions, and sensor housing dimensions have not been
provided. The model therefore does not invent inertial or housing geometry for
those components. Add measured values before using this description for dynamics
simulation or collision-sensitive planning.
