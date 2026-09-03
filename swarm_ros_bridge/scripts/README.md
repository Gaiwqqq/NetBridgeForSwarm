# Utility scripts

- `bridge_test_publisher.py`: publish all topic types from `config/default.yaml` at
  a fixed rate. Each cloud contains 10,000 deterministic random sphere points
  with per-frame orbit, rotation, and breathing motion. The 640x480 RGB color-bar
  image scrolls horizontally on every frame.
- `topic_routing_matrix_test.sh`: start four isolated ROS masters and verify
  topic-level many-to-one, one-to-many, and 2x2 many-to-many routing, schema
  consistency, conflict quarantine, and non-destination isolation.
- `build_zenoh_1_9_debs.sh`: build native Zenoh 1.9.0 Debian packages on Ubuntu 20.04.
- `create_vitrul_interface.sh`: create virtual interfaces for local multi-host simulation.
- `delete_virtul_interface.sh`: remove the virtual interfaces created for simulation.

Run the test publisher after sourcing the workspace:

```bash
rosrun swarm_ros_bridge bridge_test_publisher.py _rate:=10.0
```

The default topic names match `config/default.yaml`. ROS private parameters can
override the rate, topic names, frames, point count, sphere radius, inflation
offset, random seed, cloud motion, and image scroll speed; for example:

```bash
rosrun swarm_ros_bridge bridge_test_publisher.py \
  _rate:=20.0 _sphere_radius:=8.0 _random_seed:=7 \
  _cloud_motion_radius:=2.0 _cloud_motion_speed:=0.7 \
  _image_scroll_speed:=120.0
```
