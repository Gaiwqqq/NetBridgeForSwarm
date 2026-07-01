# NetBridgeForSwarm V1.1 BETA PREVIEW [English](README.md)

## 0. V1.1 新增内容

- 支持 `sensor_msgs/PointCloud2` 压缩传输，显著降低带宽占用。
- 支持 `sensor_msgs/PointCloud2` 发送前降采样。
- 支持点云 `Draco` 编解码。
- 支持图像流自适应 JPEG 质量控制。
- 增加终端诊断界面 `bridge_tui`。

## 1. 项目介绍

ROS1 在多机通信场景里一直不够友好，很多现有方案与具体工程耦合较深，不方便在无人机集群和地面站混合部署时复用。

NetBridgeForSwarm 是一个面向 ROS1 的多机通信中间件，支持转发：

- ROS topic
- ROS service
- 图像流 `sensor_msgs/Image`
- 自定义消息与服务类型

本项目受到 Peixuan Shu 的开源项目 [swarm_ros_bridge](https://github.com/shupx/swarm_ros_bridge) 启发，并在此基础上扩展了更灵活的配置方式、图像传输能力、点云传输能力以及更完整的诊断支持。

### 1.1 主要功能

- 所有节点共用一套 YAML 配置。
- 通过 `include/msgs_macro.hpp` 扩展自定义 topic / service 类型。
- 图像通过 UDP 传输，支持缩放和自适应 JPEG 质量调节。
- topic 和 service 通过 TCP 转发。
- 点云支持压缩、降采样与 Draco codec。
- 可选 TUI 界面查看主机、topic 和日志状态。

### 1.2 依赖

```bash
sudo apt-get install libzmqpp-dev ros-noetic-topic-tools
```

此外还依赖：

- ROS Noetic / catkin
- `cv_bridge_noetic_fit_version`
- OpenCV
- PCL
- JPEG
- 仓库内置第三方库：`FTXUI`、`draco`

如果你使用自定义 OpenCV，需要重新编译 `cv_bridge`，并替换为适配版本的 `cv_bridge_noetic_fit_version`。

示例：

```cmake
find_package(OpenCV 4.5.3 REQUIRED)
catkin_package(CATKIN_DEPENDS cv_bridge_noetic_fit_version)
```

## 2. 配置说明

主要配置文件位于 [`swarm_ros_bridge/config`](./swarm_ros_bridge/config)：

- `default.yaml`：实机示例配置
- `default_sim.yaml`：单机模拟示例配置
- `ip_real.yaml`：实机 IP 映射
- `ip_sim.yaml`：模拟 IP 映射

### 2.1 IP 配置

每个 bridge 节点都通过 ROS 参数 `hostname` 标识自己，这个值必须出现在 `IP` 映射表中。

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

注意：

- `all` 和 `all_drone` 是保留关键字，不能删除。
- 无人机命名必须使用 `drone0`、`drone1` 这种格式。
- `drone_1` 这类名字不会被解析成 drone id。

全局配置示例：

```yaml
config:
  debug: false
  odom_convert: true
  monitor_node: true
  warn_threshold: 3
  monitor_rate_hz: 500
```

### 2.2 Topic 配置

示例：

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

主要字段说明：

- `topic_name`：要转发的 ROS topic
- `msg_type`：ROS 消息类型，格式为 `package/Msg`
- `srcIP`：发送端主机名，必须来自 `IP` 表
- `dstIP`：接收端主机名，必须来自 `IP` 表
- `srcPort`：该转发规则独占的端口
- `max_freq`：最大发送频率，单位 Hz，`-1` 表示不限制
- `prefix`：接收端是否自动加来源主机名前缀
- `same_prefix`：是否统一映射到 `/bridge/...`

图像流额外配置：

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

- `imgResizeRate`：编码前缩放比例
- `imgJpegQuality`：初始 JPEG 质量
- `imgAdaptiveQuality`：是否开启自适应质量控制
- `imgMinJpegQuality` / `imgMaxJpegQuality`：自适应质量上下限
- `imgTargetBandwidthKbps`：目标带宽
- `imgQualityStep`：每次调整的质量步长
- `imgAdaptCooldownFrames`：两次调节之间的冷却帧数

点云额外配置：

```yaml
  - topic_name: /drone_0_ego_planner_node/grid_map/occupancy
    msg_type: sensor_msgs/PointCloud2
    cloudCompress: true
    cloudDownsample: -1.0
    cloudCodec: draco
```

- `cloudCompress`：是否压缩点云
- `cloudDownsample`：点云降采样参数，`-1.0` 表示关闭
- `cloudCodec`：当前示例使用 `draco`

多机通配示例：

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

当 `topic_name` 中包含 `{id}` 时，bridge 会自动替换为对应的无人机编号。比如地面站会收到 `/drone_0_ego_planner_node/optimal_list`、`/drone_1_ego_planner_node/optimal_list` 等。

### 2.3 Service 配置

service 支持单服务端、多客户端。

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

当 `prefix: true` 时，客户端调用时需要带服务端主机名前缀。以上例中客户端调用路径应为 `/drone1/add_two_ints`。

### 2.4 自定义消息类型

如果要增加自定义 topic 或 service 类型，需要修改 [`swarm_ros_bridge/include/msgs_macro.hpp`](./swarm_ros_bridge/include/msgs_macro.hpp)：

1. 引入对应的头文件
2. 在 `MSGS_MACRO` 或 `SRVS_MACRO` 中追加类型
3. 在 YAML 中使用完全一致的 `msg_type` 或 `srv_type`

示例：

```cpp
#include <your_pkg/YourMsg.h>

#define MSGS_MACRO \
  X("sensor_msgs/Image", sensor_msgs::Image) \
  X("your_pkg/YourMsg", your_pkg::YourMsg)
```

## 3. 编译与运行

### 3.1 编译

在安装好依赖后，放到 catkin 工作空间中进行编译即可。

### 3.2 单机模拟

单机模拟时，需要先创建虚拟网卡：

```bash
sh swarm_ros_bridge/scripts/create_vitrul_interface.sh 4
```

不再需要时删除：

```bash
sh swarm_ros_bridge/scripts/delete_virtul_interface.sh 4
```

随后加载模拟配置，例如：

```xml
<group ns="bridge">
  <node pkg="swarm_ros_bridge" type="bridge_new" name="swarm_bridge_node" output="screen">
    <param name="hostname" type="string" value="drone1"/>
    <rosparam command="load" file="$(find swarm_ros_bridge)/config/default_sim.yaml" />
    <rosparam command="load" file="$(find swarm_ros_bridge)/config/ip_sim.yaml" />
  </node>
</group>
```

### 3.3 实机部署

实机运行时，先修改 `ip_real.yaml`，然后使用仓库内提供的 launch：

- `launch/example_bridge_drone.launch`
- `launch/example_bridge_station.launch`
- `launch/bridge_with_tui_drone.launch`
- `launch/bridge_with_tui_station.launch`

无人机端示例：

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

地面站示例：

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

- BestAnHongjun 和 [PicSocket](https://github.com/BestAnHongjun/PicSocket) 项目，对图像传输部分的实现帮助很大。
