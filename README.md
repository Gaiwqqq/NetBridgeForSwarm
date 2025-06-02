# NetBridgeForSwarm V1.1 BETA PREVIEW [中文](README-zh.md)

## 0. [version 1.1] New Features

- Support PointCloud2 message compression transmission. The bandwidth consumption is greatly reduced.
- Support PointCloud2 message downsampling transmission.

## 1. Introduction
ROS1 has always been a challenge for multi-machine communication support. There are many existing solutions that require the project to be bound, which are not flexible enough to meet the needs of users.
  Therefore, we hope to develop a multi-machine communication middleware that can forward **ros topic**, **ros service**, and **image** (as a subtype of ros topic) messages to other machines, and support custom message types.
  
The inspiration for this project comes from the open-source project of Peixuan Shu, [swarm_ros_bridge](https://github.com/shupx/swarm_ros_bridge). We hope to refactor it and expand its functionality, making it more flexible and user-friendly.

This is a multi-machine communication middleware that can forward messages from multiple ROS nodes to other machines, supporting custom message types, supporting ROS video stream transmission, and supporting ROS message TCP transmission.
  Thanks to zmqpp and my own encapsulation of the communication layer, users can get a high-performance and stable multi-machine communication experience without having to worry about the underlying network communication details.

### 1.1 Main Features

- All swarm agents use the same configuration file, which can flexibly configure the topics, services, and images (as a subtype of ros topic) to be forwarded, reducing the configuration complexity.
- Support custom message types, which can be added to `include/msgs_macro.hpp` and specified in the yaml file.
- Support ros video stream (sensor_msgs/Image) UDP transmission (custom compression ratio), ros topic TCP transmission, and ros service TCP transmission.
- Completely restructured code framework, making it more user-friendly, flexible, and extensible.

### 1.2 Future Work

- More custom configuration options for video stream transmission, such as bit rate, compression format, etc.
- Add ros topic UDP transmission.
- Add ros service UDP transmission.

### 1.3 Dependencies

```shell
#zmqpp 
sudo apt-get install libzmqpp-dev ros-noetic-topic-tools 
```
Note that since the cv_bridge depends on opencv, and the default compilation version is the one provided by ros, if you want to use a custom opencv, you need to recompile cv_bridge and replace the cv_bridge_noetic_fit_version package with your own custom opencv version.
For example, if the ros version is noetic and the custom opencv version is 4.5.3, you need to compile cv_bridge_noetic_fit_version.

```shell
# opencv
find_package(OpenCV 4.5.3 REQUIRED)
catkin_package(CATKIN_DEPENDS cv_bridge_noetic_fit_version)
```

## 2. Configuration

### 2.1 Ip configuration
The IP configuration file should be set according to the actual network configuration. The `ip_sim.yaml` 
file is an example of basic configuration. The `IP` field in the configuration file should be set to the 
corresponding IP address of the machine.

```yaml
IP:
  all: '*'                          # '*' stands for all IPs
  all_drone: 'all_drone'            # all drone, not include station
  groundStation0: 172.16.0.200      # laptop (grond station)
  drone0: 172.16.0.100              # drone 0 zld-4
  drone1: 172.16.0.101              # drone 1 zld-4
  drone2: 172.16.0.102              # drone 2 zld-4
  drone3: 172.16.0.103              # drone 3 zld-4
  drone4: 172.16.0.104              # drone 4 zld-4
```

The `srcIP` and `dstIP` fields in the topic configuration file should be set to the corresponding hostname in the IP configuration file. 
The `all` and `all_drone` keywords cannot be deleted. The `drone` field can only be set to `drone0`, `drone5` in the form of `drone_1`, 
such as `drone_1` will not be recognized as an id.


```yaml
config:
  debug: false                    # When true, you can receive messages you sent; when false, they are excluded
  odom_convert: true              # When true, odom-type topics will be automatically converted to PoseStamped before sending and converted back on the receiving end
  monitor_node: true              # When true, enables monitor_node to track the frequency and bandwidth of messages received by the bridge (currently deprecated)
  warn_threshold: 3               # Frequency threshold for monitor_node (currently deprecated)
  monitor_rate_hz: 500            # Detection frequency of monitor_node; must be higher than the maximum frequency of all to
```

### 2.2 Topic configuration
The `default.yaml` file defines the configuration information related to topic service. 
The specific configuration method is as follows:

```yaml
topics:
  - topic_name: /ekf_quat/ekf_odom  # send the messages of this ROS topic
    msg_type: nav_msgs/Odometry     # ROS message type (rosmsg style)
    imgResizeRate: 0.5              # only for image topic, resize rate, default: 1.0(raw image) [only used for image topic]                         
    cloudCompress: true             # only for [sensors_msgs/PointCloud2], default = false
    cloudDownsample: 0.1            # only for [sensors_msgs/PointCloud2], range [1e-4, 1e4] (default = -1.0, disable), value more, point less
    srcIP:
      - all_drone                     # send devices, all_drone means drone0, drone1, drone2....
      - drone1
    srcPort: 3001                   # ports of send_topics should be different
    max_freq: -1                    # max send frequent(hz), default: 10, unlimited: -1
    dstIP:
      - groundStation0                # recv devices
      - groundStation1
    prefix: true                    # add namespace prefix, default: true
    same_prefix: false              # prefix namespace with same name, default: false (multi adress to one topic)
```

The `all_drone` keyword represents all drones, `srcIP` and `dstIP` should correspond to the hostname in the IP field. 
When `prefix` is true, the namespace will be automatically added to the topic name on the receiving side (dstIP), 
the namespace is the hostname of the source (srcIP), to prevent multiple-to-many situations where the dstIP receives all 
topics from the same source.

For example, in the above example, srcIP is `all_drone`, groundStation0 will receive `/drone0/ekf_quat/ekf_odom`, `/drone1/ekf_quat/ekf_odom`, etc., 
corresponding to the source of drone0 and drone1. When `same_prefix` is true, all received topics are `/bridge/ekf_quat/ekf_odom`.

```yaml
- topic_name: /drone_{id}_ego_planner_node/optimal_list  # keyword {id} will be replaced by drone id
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
The format `/drone_{id}` can be added to the beginning of the topic_name, and the program will automatically parse the id. 
For example, in the above example, groundStation0 will receive `/drone_0_ego_planner_node/optimal_list`, `/drone_1_ego_planner_node/optimal_list`, etc.

### 2.3 Service configuration
The service supports multiple clients, one server, and the `prefix` only indicates whether the client needs to add a namespace. 
For example, the server is `drone0`, the client is `groundStation0`, then the server's service name is `/add_two_ints`, and the client needs to call the service name `/drone0/add_two_ints`.
```yaml
services:
  # --------- Services ---------- #
  - srv_name: /add_two_ints
    srv_type: swarm_ros_bridge/AddTwoInts
    serverIp: drone1
    clientIp:
      - drone2
    srcPort: 2000
    prefix: true
```

## 2.4 Customize message type
To add a new custom message, you need to modify `include/msgs_macro.hpp`, add the custom message above 
the include line, and fill in the format in `MSGS_MACRO`. The first parameter of the X macro should match the `msg_type` in the yaml file.
```c++
#ifndef __MSGS_MACRO__
#define __MSGS_MACRO__
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>

#include <std_srvs/Empty.h>
#include <swarm_ros_bridge/AddTwoInts.h>
// include your msg type here

#define INFO_MSG(str) do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str) do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str) do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str) do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)

// Use X macro
#define MSGS_MACRO \
  X("sensor_msgs/Image", sensor_msgs::Image)                           \
  X("std_msgs/String", std_msgs::String)                               \
  X("nav_msgs/Odometry", nav_msgs::Odometry)                           \
  X("geometry_msgs/PoseStamped", geometry_msgs::PoseStamped)           \
  X("sensor_msgs/PointCloud2", sensor_msgs::PointCloud2)               \
  X("visualization_msgs/Marker", visualization_msgs::Marker)           \
  X("visualization_msgs/MarkerArray", visualization_msgs::MarkerArray)

#define SRVS_MACRO \
  X("std_srvs/Empty", std_srvs::Empty) \
  X("swarm_ros_bridge/AddTwoInts", swarm_ros_bridge::AddTwoInts)

#endif

```

## 3. Run

### 3.1 Single PC simulation
To run the simulation in a single PC, you need to create multiple virtual network cards. You can use the `sh scripts/create_virtual_interface.sh 4` 
command to create virtual network cards, the parameter is the number of drones. The default IP address of the ground 
station is `172.16.0.200`, and the IP address of drone0 is `172.16.0.100`, and so on. If you don't need the network card, 
you can use the `scripts/delete_virtual_interface.sh` command to delete the network card, the parameter is the number of drones.

```yaml
IP:
  all: '*'                         # '*' stands for all IPs
  all_drone: 'all_drone'           # all drone, not include station
  groundStation0: 172.16.0.200     # laptop (grond station)
  drone0: 172.16.0.100             
  drone1: 172.16.0.101
  drone2: 172.16.0.102
  drone3: 172.16.0.103
```
```xml
    <group ns="bridge">
    <node pkg="swarm_ros_bridge" type="bridge_new" name="swarm_bridge_node" output="screen" >
      <param name="hostname" type="string" value="drone1"/>
      <rosparam command="load" file="$(find swarm_ros_bridge)/config/default_sim.yaml" />
      <rosparam command="load" file="$(find swarm_ros_bridge)/config/ip_real.yaml" />
    </node>
</group>
```

### 3.2 Realworld Run
To run in realworld, you need to modify the `ip_real.yaml` file. The launch file is as follows, you need to modify the path of the loaded yaml file and the name of the self-recognition.

```xml
  <group ns="bridge">
    <node pkg="swarm_ros_bridge" type="bridge_new" name="swarm_bridge_node" output="screen" >
      <param name="hostname" type="string" value="drone1"/>
      <rosparam command="load" file="$(find swarm_ros_bridge)/config/default_sim.yaml" />
      <rosparam command="load" file="$(find swarm_ros_bridge)/config/ip_real.yaml" />
    </node>
  </group>
```

## 4. Contributor

- Weiqi Gai 2025.01 
- KengHou Hoi 2024.08

## Special Thanks

-  BestAnHongjun (an.hongjun@foxmail.com) for his github project [PicSocket](https://github.com/BestAnHongjun/PicSocket)
which helps me a lot in the development of image transmission.
