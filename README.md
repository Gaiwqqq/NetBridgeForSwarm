# NetBridgeForSwarm V1.1 BETA PREVIEW [中文](README-zh.md)

## 0. New In V1.1

- Support `sensor_msgs/PointCloud2` compression to reduce bandwidth usage.
- Support `sensor_msgs/PointCloud2` downsampling before transmission.
- Support Draco point cloud codec.
- Support adaptive JPEG quality control for image streams.
- Add terminal UI (`bridge_tui`) for bridge diagnostics.

## 1. Introduction

ROS1 multi-machine communication is still inconvenient in many real projects. Existing solutions are often tightly coupled to a specific system and are not flexible enough for mixed drone / station deployments.

NetBridgeForSwarm is a ROS1 multi-machine communication middleware that forwards:

- ROS topics
- ROS services
- image streams (`sensor_msgs/Image`)
- custom message and service types

This project is inspired by Peixuan Shu's open-source project [swarm_ros_bridge](https://github.com/shupx/swarm_ros_bridge), and extends it with a more flexible configuration model, image transport improvements, point cloud transport, and newer diagnostics support.

### 1.1 Main Features

- One shared YAML configuration for all nodes in the swarm.
- Custom topic and service type support through `include/msgs_macro.hpp`.
- Image transport over UDP with resize and adaptive JPEG quality settings.
- Topic and service forwarding over TCP.
- Point cloud compression, downsampling, and Draco codec support.
- Optional TUI monitor for host/topic/log visibility.

### 1.2 Dependencies

```bash
sudo apt-get install libzmqpp-dev ros-noetic-topic-tools
```

The package also depends on:

- ROS Noetic / catkin
- `cv_bridge_noetic_fit_version`
- OpenCV
- PCL
- JPEG
- bundled third-party libraries: `FTXUI` and `draco`

If you use a custom OpenCV build, recompile `cv_bridge` against that version and replace `cv_bridge_noetic_fit_version` accordingly.

Example:

```cmake
find_package(OpenCV 4.5.3 REQUIRED)
catkin_package(CATKIN_DEPENDS cv_bridge_noetic_fit_version)
```

## 2. Configuration

Main config files are under [`swarm_ros_bridge/config`](./swarm_ros_bridge/config):

- `default.yaml`: real deployment example
- `default_sim.yaml`: local simulation example
- `ip_real.yaml`: real network host map
- `ip_sim.yaml`: simulated network host map

### 2.1 IP Configuration

Each bridge node identifies itself through the ROS param `hostname`. That hostname must exist in the `IP` map.

```yaml
IP:
  all: '*'
  all_drone: 'all_drone'
  groundStation0: 172.16.0.200
  drone0: 172.16.0.100
  drone1: 172.16.0.101
  drone2: 172.16.0.102
  drone3: 172.16.0.103
```

Notes:

- `all` and `all_drone` are reserved keywords and must not be removed.
- Drone hostnames must use the `drone0`, `drone1`, ... format.
- Names like `drone_1` are not parsed as drone ids.

Global config example:

```yaml
config:
  debug: false
  odom_convert: true
  monitor_node: true
  warn_threshold: 3
  monitor_rate_hz: 500
```

### 2.2 Topic Configuration

Example:

```yaml
topics:
  - topic_name: /ekf_quat/ekf_odom
    msg_type: nav_msgs/Odometry
    srcIP:
      - drone1
    srcPort: 3004
    max_freq: 30
    dstIP:
      - groundStation0
    prefix: false
    same_prefix: false
```

Important fields:

- `topic_name`: ROS topic to forward
- `msg_type`: ROS message type in `package/Msg` format
- `srcIP`: sender hostnames from the `IP` map
- `dstIP`: receiver hostnames from the `IP` map
- `srcPort`: unique port for this forwarding rule
- `max_freq`: max transmit rate in Hz, `-1` means unlimited
- `prefix`: add source hostname as namespace on receiver side
- `same_prefix`: when enabled, merge under `/bridge/...`

Special topic options:

```yaml
  - topic_name: /camera/color/image_raw
    msg_type: sensor_msgs/Image
    imgResizeRate: 0.5
    imgJpegQuality: 85
    imgAdaptiveQuality: true
    imgMinJpegQuality: 40
    imgMaxJpegQuality: 90
    imgTargetBandwidthKbps: 800
    imgQualityStep: 5
    imgAdaptCooldownFrames: 6
```

- `imgResizeRate`: resize ratio before encoding
- `imgJpegQuality`: initial JPEG quality
- `imgAdaptiveQuality`: enable adaptive quality control
- `imgMinJpegQuality` / `imgMaxJpegQuality`: adaptive quality bounds
- `imgTargetBandwidthKbps`: target bandwidth for adaptive control
- `imgQualityStep`: quality step per adjustment
- `imgAdaptCooldownFrames`: cooldown before the next adaptation

Point cloud options:

```yaml
  - topic_name: /drone_0_ego_planner_node/grid_map/occupancy
    msg_type: sensor_msgs/PointCloud2
    cloudCompress: true
    cloudDownsample: -1.0
    cloudCodec: draco
```

- `cloudCompress`: enable point cloud compression
- `cloudDownsample`: `-1.0` disables downsampling
- `cloudCodec`: current example uses `draco`

Wildcard source example:

```yaml
  - topic_name: /drone_{id}_ego_planner_node/optimal_list
    msg_type: visualization_msgs/Marker
    srcIP:
      - all_drone
    srcPort: 3002
    max_freq: 30
    dstIP:
      - groundStation0
    prefix: false
    same_prefix: false
```

When `{id}` is used in the topic name, the bridge replaces it with the parsed drone id. For example, `groundStation0` can receive `/drone_0_ego_planner_node/optimal_list`, `/drone_1_ego_planner_node/optimal_list`, and so on.

### 2.3 Service Configuration

Services support one server and multiple clients.

```yaml
services:
  - srv_name: /add_two_ints
    srv_type: swarm_ros_bridge/AddTwoInts
    serverIp: drone1
    clientIp:
      - groundStation0
      - drone2
    srcPort: 2000
    prefix: true
```

If `prefix` is `true`, clients call the service with the server hostname namespace. For the example above, the client calls `/drone1/add_two_ints`.

### 2.4 Custom Message Types

To add a custom topic or service type, edit [`swarm_ros_bridge/include/msgs_macro.hpp`](./swarm_ros_bridge/include/msgs_macro.hpp):

1. add the required message / service header
2. append the type to `MSGS_MACRO` or `SRVS_MACRO`
3. use the exact same `msg_type` or `srv_type` string in YAML

Example:

```cpp
#include <your_pkg/YourMsg.h>

#define MSGS_MACRO \
  X("sensor_msgs/Image", sensor_msgs::Image) \
  X("your_pkg/YourMsg", your_pkg::YourMsg)
```

## 3. Build And Run

### 3.1 Build

Build it in a catkin workspace after dependencies are installed.

### 3.2 Local Simulation

For single-PC simulation, create virtual interfaces first:

```bash
sh swarm_ros_bridge/scripts/create_vitrul_interface.sh 4
```

Delete them when no longer needed:

```bash
sh swarm_ros_bridge/scripts/delete_virtul_interface.sh 4
```

Then launch with the simulation config files, for example:

```xml
<group ns="bridge">
  <node pkg="swarm_ros_bridge" type="bridge_new" name="swarm_bridge_node" output="screen">
    <param name="hostname" type="string" value="drone1"/>
    <rosparam command="load" file="$(find swarm_ros_bridge)/config/default_sim.yaml" />
    <rosparam command="load" file="$(find swarm_ros_bridge)/config/ip_sim.yaml" />
  </node>
</group>
```

### 3.3 Real Deployment

For real machines, update `ip_real.yaml` and use the provided launch files:

- `launch/example_bridge_drone.launch`
- `launch/example_bridge_station.launch`
- `launch/bridge_with_tui_drone.launch`
- `launch/bridge_with_tui_station.launch`

Example drone launch:

```xml
<launch>
  <group ns="bridge">
    <node pkg="swarm_ros_bridge" type="bridge_new" name="swarm_bridge_node" output="screen">
      <param name="hostname" type="string" value="drone$(env DRONE_ID)"/>
      <rosparam command="load" file="$(find swarm_ros_bridge)/config/default.yaml" />
      <rosparam command="load" file="$(find swarm_ros_bridge)/config/ip_real.yaml" />
    </node>
  </group>
</launch>
```

Example station launch:

```xml
<launch>
  <node pkg="swarm_ros_bridge" type="bridge_new" name="swarm_bridge_station_node" output="screen">
    <param name="hostname" type="string" value="groundStation0"/>
    <rosparam command="load" file="$(find swarm_ros_bridge)/config/default.yaml" />
    <rosparam command="load" file="$(find swarm_ros_bridge)/config/ip_real.yaml" />
  </node>
</launch>
```

## 4. Contributors

- Weiqi Gai 2025.01
- KengHou Hoi 2024.08

## Special Thanks

- BestAnHongjun and the [PicSocket](https://github.com/BestAnHongjun/PicSocket) project for helping the image transport implementation.
