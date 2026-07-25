# Livox Mid-360 for ROS 2 + Gazebo Sim

This is the ROS 2 Humble / Gazebo Harmonic (`gz-sim8`) port of the original
Gazebo Classic plugin. Build it from the workspace root:

```bash
colcon build --packages-select livox_laser_simulation
source install/setup.bash
ros2 launch livox_laser_simulation mid360_gz.launch.py
rviz2 -d "$(ros2 pkg prefix livox_laser_simulation)/share/livox_laser_simulation/rviz/test_pattern.rviz"
```

The plugin is a Gazebo Sim system plugin (`LivoxGzSystem`), not a Classic
sensor plugin. It subscribes to the `gpu_lidar` Gazebo Transport topic and
publishes directly to ROS 2, so `ros_gz_bridge` is not required.

The Gazebo topic `/livox/mid360/scan` is only the plugin input. The ROS 2
output is `/livox/lidar`; after launch, verify it with `ros2 topic list` and
`ros2 topic echo /livox/lidar --once`.

The Mid-360 pattern remains driven by `scan_mode/mid360-real-centr.csv`. Since
Gazebo Sim's stock LiDAR supports a rectangular angular grid rather than
arbitrary per-ray directions, the plugin selects the nearest source-grid ray
for each CSV direction. The model's `horizontal` and `vertical` sample counts
control this angular approximation and GPU cost. The defaults preserve the
legacy 36000-ray source grid and publish 18000 Mid-360 ordered points at 10 Hz.

`publish_pointcloud_type` keeps the old numeric contract:

| Value | ROS 2 output |
| --- | --- |
| 0 | `sensor_msgs/msg/PointCloud` |
| 1 | `sensor_msgs/msg/PointCloud2` (`x`, `y`, `z`) |
| 2 | `sensor_msgs/msg/PointCloud2` (`x`, `y`, `z`, `intensity`, `tag`, `line`, `timestamp`) |
| 3 | `livox_laser_simulation/msg/CustomMsg` |

For a custom world, include `models/Mid360`, load the standard Gazebo Sim
`Physics`, `Sensors` (with `ogre2`), and `SceneBroadcaster` systems, and expose
both the installed package's `models` directory in `GZ_SIM_RESOURCE_PATH` and
its `lib` directory in `GZ_SIM_SYSTEM_PLUGIN_PATH`.
