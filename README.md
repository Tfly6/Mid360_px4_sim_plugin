# Livox Mid-360 for ROS 2 + Gazebo Sim

ROS 2 Humble / Gazebo Harmonic port of the Livox Mid-360 Gazebo Classic plugin.
The Gazebo Sim system plugin subscribes to `/livox/mid360/scan` and publishes
the ROS 2 point cloud on `/livox/lidar` directly; `ros_gz_bridge` is not needed.


## Build
Download:
```bash
mkdir ~/ros2_ws/src -p
cd ~/ros2_ws/src
git clone https://github.com/Tfly6/Mid360_px4_sim_plugin.git
cd Mid360_px4_sim_plugin/
git checkout ros2
```
Build from the workspace root:

```bash
cd ~/ros2_ws/
colcon build --packages-select livox_laser_simulation
source install/setup.bash
```

## Mid-360 world

```bash
cd ~/ros2_ws/
source install/setup.bash
ros2 launch livox_laser_simulation mid360_gz.launch.py
rviz2 -d "$(ros2 pkg prefix livox_laser_simulation)/share/livox_laser_simulation/rviz/test.rviz"
```

`publish_pointcloud_type` keeps the legacy numeric contract:

| Value | ROS 2 output |
| --- | --- |
| 0 | `sensor_msgs/msg/PointCloud` |
| 1 | `sensor_msgs/msg/PointCloud2` (`x`, `y`, `z`) |
| 2 | `sensor_msgs/msg/PointCloud2` (`x`, `y`, `z`, `intensity`, `tag`, `line`, `timestamp`) |
| 3 | `livox_laser_simulation/msg/CustomMsg` |

## PX4 SITL target overlay

PX4 generates `make px4_sitl gz_<model>` targets from airframes. The supplied
script force-replaces only the two model directories (`Mid360` and
`x500_mid360`), the Livox plugin, and the `4022_gz_x500_mid360` airframe. It
adds the CMake airframe entry only when absent:

```bash
./src/Mid360_px4_sim_plugin/livox_laser_simulation/px4/install_px4_x500_mid360.sh /path/to/PX4-Autopilot
```

launching through PX4:

```bash
cd /path/to/PX4-Autopilot
make px4_sitl gz_x500_mid360
```
