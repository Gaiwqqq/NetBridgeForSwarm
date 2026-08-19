# Utility scripts

- `bridge_test_publisher.py`: publish all topic types from `config/default.yaml` at
  a fixed rate. Each cloud contains 10,000 random points on a sphere and the image
  is a 640x480 RGB color-bar pattern.
- `build_zenoh_1_9_debs.sh`: build native Zenoh 1.9.0 Debian packages on Ubuntu 20.04.
- `create_vitrul_interface.sh`: create virtual interfaces for local multi-host simulation.
- `delete_virtul_interface.sh`: remove the virtual interfaces created for simulation.

Run the test publisher after sourcing the workspace:

```bash
rosrun swarm_ros_bridge bridge_test_publisher.py _rate:=10.0
```

The default topic names match `config/default.yaml`. ROS private parameters can
override the rate, topic names, frames, point count, sphere radius, inflation
offset, and random seed; for example:

```bash
rosrun swarm_ros_bridge bridge_test_publisher.py \
  _rate:=20.0 _sphere_radius:=8.0 _random_seed:=7
```
