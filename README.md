# NetBridgeForSwarm V1.0 BETA PREVIEW

## 1. Introduction
#### [English]
- ROS1 has always been a challenge for multi-machine communication support. There are many existing solutions that require the project to be bound, which are not flexible enough to meet the needs of users.
  Therefore, we hope to develop a multi-machine communication middleware that can forward **ros topic**, **ros service**, and **image** (as a subtype of ros topic) messages to other machines, and support custom message types.

- The inspiration for this project comes from the open-source project of Peixuan Shu, [swarm_ros_bridge](https://github.com/shupx/swarm_ros_bridge). We hope to refactor it and expand its functionality, making it more flexible and user-friendly.

- This is a multi-machine communication middleware that can forward messages from multiple ROS nodes to other machines, supporting custom message types, supporting ROS video stream transmission, and supporting ROS message TCP transmission.
  Thanks to zmqpp and my own encapsulation of the communication layer, users can get a high-performance and stable multi-machine communication experience without having to worry about the underlying network communication details.

#### [中文]
- ROS1 对多机通讯的支持一直是个难题，现存的解决方案大多需要与项目绑定，难以定制化的满足使用需求。
因此，希望开发一种多机通讯中间件，可以将**ros topic**、**ros service**、**image**等多种消息类型转发到其他机器，并支持自定义消息类型。

- 灵感来源于Peixuan Shu博士的开源项目[swarm_ros_bridge](https://github.com/shupx/swarm_ros_bridge)，我们希望能将其重构，并进行功能扩展，使其更加灵活、易用。

- 这是一个ros多机通讯中间件，可以将多个ros节点的消息转发到其他机器，支持自定义消息/服务类型，支持ros视频流传输，支持ros消息TCP传输。
同时得益于zmqpp以及本人在通讯层的封装使得用户无需关心底层网络通信细节便可获得良好的高性能稳定多机通讯体验。
### 1.1 Main Features

#### [English]
- 1. All swarm agents use the same configuration file, which can flexibly configure the topics, services, and images (as a subtype of ros topic) to be forwarded, reducing the configuration complexity.
- 2. Support custom message types, which can be added to `include/msgs_macro.hpp` and specified in the yaml file.
- 3. Support ros video stream (sensor_msgs/Image) UDP transmission (custom compression ratio), ros topic TCP transmission, and ros service TCP transmission.
- 4. Completely restructured code framework, making it more user-friendly, flexible, and extensible.

#### [中文]
- 1. 所有智能体使用 `同一个配置文件`，可灵活配置需要转发的topic，service, image(作为ros topic的子类型)， 免去繁琐的配置过程
- 2. 支持**自定义消息类型**，只需在`include/msgs_macro.hpp`中添加自定义topic/service类型，并在yaml文件中指定消息类型即可
- 3. 支持ros视频流(sensor_msgs/Image)UDP传输（可自定义压缩比）， ros topic TCP传输， ros service TCP传输
- 4. 完全重构的代码框架，使其更加易用、灵活且易于扩展

### 1.2 Future Work

#### [English]
- More custom configuration options for video stream transmission, such as bit rate, compression format, etc.
- Add ros topic UDP transmission.
- Add ros service UDP transmission.

#### [中文]
- 视频流传输的更多自定义配置，例如码率，压缩格式等
- 增加ros topic UDP传输
- 增加ros service UDP传输

### 1.3 Dependencies

```shell
#zmqpp 
sudo apt-get install libzmqpp-dev
```
#### [English]
- Note that since the cv_bridge depends on opencv, and the default compilation version is the one provided by ros, if you want to use a custom opencv, you need to recompile cv_bridge and replace the cv_bridge_noetic_fit_version package with your own custom opencv version.
For example, if the ros version is noetic and the custom opencv version is 4.5.3, you need to compile cv_bridge_noetic_fit_version.

#### [中文]
- 注意，由于ros的cv_bridge依赖于opencv，且默认编译版本为ros自带，因此若要使用自定义的opencv，需自行重新编译cv_bridge并参照
项目中cv_bridge_noetic_fit_version包进行替换以适配自定义opencv版本
例如：ros版本为noetic，自定义opencv版本为4.5.3，则需编译cv_bridge_noetic_fit_version
```shell
# opencv
find_package(OpenCV 4.5.3 REQUIRED)
catkin_package(CATKIN_DEPENDS cv_bridge_noetic_fit_version)
```

```shell
# topic-tools 
sudo apt-get install ros-noetic-topic-tools 
```

## 2. Configuration

### 2.1 Ip configuration

- [中文]需要转发的topic在config文件夹内定义，`config/ip_sim.yaml`介绍了基本的配置方法。
- [English] The IP configuration file should be set according to the actual network configuration. The `ip_sim.yaml` 
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

- [中文] 设置hostname对应的ip，其中`all`和`all_drone`为关键字，不能删除。`drone`只能设置成`drone0`，`drone5`的形式，例如`drone_1`将无法识别id
- [English] The `srcIP` and `dstIP` fields in the topic configuration file should be set to the corresponding hostname in the IP configuration file. 
The `all` and `all_drone` keywords cannot be deleted. The `drone` field can only be set to `drone0`, `drone5` in the form of `drone_1`, 
such as `drone_1` will not be recognized as an id.


```yaml
config:
  debug: false                    # true时可以收到自己发出的消息，false时排除
  odom_convert: true              # true时odom类型的topic将自动转为posestamped发出，收方会再转回odom
  monitor_node: true              # true时启动monitor_node，用于检测bridge收到的消息的频率、带宽等信息, 目前弃用
  warn_threshold: 3               # monitor_node的频率阈值，目前弃用
  monitor_rate_hz: 500            # monitor_node的检测频率，必须大于所有topic的最大频率, 目前弃用
```

### 2.2 Topic configuration
- [中文] `default.yaml` 定义了 topic service 相关的配置信息，具体配置方法如下：
- [English] The `default.yaml` file defines the configuration information related to topic service. 
The specific configuration method is as follows:
  
```yaml
topics:
- topic_name: /ekf_quat/ekf_odom  # send the messages of this ROS topic
  msg_type: nav_msgs/Odometry     # ROS message type (rosmsg style)
  imgResizeRate: 0.5              # only for image topic, resize rate, default: 1.0(raw image) [only used for image topic]                         
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

#### [中文] 

可用关键字`all_drone`代表所有飞机，`srcIP`和`dstIP`都要对应IP字段内填写的hostname，
`prefix`为true的情况下，收方（dstIP）的会自动在topic名字前添加namespace，namespace为来源（srcIP）的hostname，防止多对多时收方把
所有来源对应到同一个topic名上。 

例如上方的例子srcIP为`all_drone`，groundStation0将会收到`/drone0/ekf_quat/ekf_odom`、`/drone1/ekf_quat/ekf_odom`等，分别对应
来源于drone0和drone1。`same_prefix`为true时，所有收到的topic均为`/bridge/ekf_quat/ekf_odom`

#### [English]

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

- [中文] 可在topic_name最前面加上`/drone_{id}`的格式，这种格式程序会自动解析id，例如上述例子的groudStation0将会收到，
`/drone_0_ego_planner_node/optimal_list`、`/drone_1_ego_planner_node/optimal_list`等。
- [English] The format `/drone_{id}` can be added to the beginning of the topic_name, and the program will automatically parse the id. 
For example, in the above example, groundStation0 will receive `/drone_0_ego_planner_node/optimal_list`, `/drone_1_ego_planner_node/optimal_list`, etc.

### 2.3 Service configuration

- [中文] service 支持多客户端，一服务端，与前面不同的是，这里的`prefix` 仅表示客户端是否需要添加namespace，服务端无需添加。
例如，服务端为`drone0`，客户端为`groundStation0`，则服务端的服务名为`/add_two_ints`，客户端需要call的服务名为`/drone0/add_two_ints`
- [English] The service supports multiple clients, one server, and the `prefix` only indicates whether the client needs to add a namespace. 
For example, the server is `drone0`, the client is `groundStation0`, then the server's service name is `/add_two_ints`, and the client needs to call the service name `/drone0/add_two_ints`.
- 
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


- [中文] 需添加新的自定义消息，需要修改`include/msgs_macro.hpp`，在上方include自定义消息，在`MSGS_MACRO`里按照格式填写，X宏的第一个参数需和yaml文件内的`msg_type`能够对应上
- [English] To add a new custom message, you need to modify `include/msgs_macro.hpp`, add the custom message above 
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

- [中文] 需创建多个虚拟网卡，可用`sh scripts/create_virtual_interface.sh 4`命令创建，参数为无人机的个数，groundStation默认设置
为`172.16.0.200`，drone0为`172.16.0.100`，drone1为`172.16.0.101`，以此类推。不需要网卡时可用`scripts/delete_virtual_interface.sh`删除网卡，参数同样为无人机的个数
- [English] To run the simulation in a single PC, you need to create multiple virtual network cards. You can use the `sh scripts/create_virtual_interface.sh 4` 
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

- [中文] 需修改`ip_real.yaml`文件， launch文件的编写方法如下，需要修改载入的yaml文件路径以及自身的识别名
- [English] To run in realworld, you need to modify the `ip_real.yaml` file. The launch file is as follows, you need to modify the path of the loaded yaml file and the name of the self-recognition.

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