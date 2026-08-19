# NetBridgeForSwarm（Zenoh 1.9） [English](README.md)

NetBridgeForSwarm 是面向 ROS1 Noetic 多机系统的通信 bridge。当前版本使用一个进程级共享 Zenoh session 统一承载 topic、图像、点云与 service，同时保留 ROS 序列化、JPEG 自适应质量、Draco/PCL 点云压缩、前缀规则及 `to_drone_ids` 定向路由。

主要变化：

- 移除 ZeroMQ、每 topic 独立端口和自研 UDP 图像分片运行时依赖。
- topic 使用 Zenoh pub/sub，service 使用 query/reply。
- 使用版本化 `NBZ1` envelope，携带来源、序号、源时间、ROS type/MD5、header 与 payload 类型。
- Zenoh 回调只写入有界队列；ROS 解码和发布由可停止、可 join 的 worker 执行。
- `state`/`bulk` 队列丢旧保新，避免图像或点云积压；`command`/service 使用阻塞拥塞策略。
- 图像通过 Zenoh 传输 JPEG，并恢复原始 `seq/stamp/frame_id`。

## 依赖与编译

基础依赖包括 ROS Noetic/catkin、OpenCV、PCL、JPEG、仓库内置 FTXUI 与 Draco，以及固定版本的 `zenoh-c`/`zenoh-cpp` 1.9.0。

官方 GNU Zenoh 1.9.0 二进制不能直接运行于 Ubuntu 20.04（GLIBC 2.31）。请分别在 x86_64 和 ARM64 的 Ubuntu 20.04 构建机上生成原生 Debian 包：

```bash
sudo apt-get install build-essential cmake git curl dpkg-dev ros-noetic-topic-tools
```

使用官方 [`rustup`](https://rust-lang.org/tools/install.html) 安装并固定 Rust 1.93 工具链。安装完成后必须加载 Cargo 环境；新开的 shell 会自动加载：

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
  sh -s -- -y --profile minimal --default-toolchain 1.93.0
source "$HOME/.cargo/env"
rustup default 1.93.0
rustc --version
cargo --version
```

随后构建 Zenoh Debian 包：

```bash
./swarm_ros_bridge/scripts/build_zenoh_1_9_debs.sh \
  --work-dir /tmp/zenoh-1.9-build \
  --output-dir ./zenoh-debs
sudo dpkg -i ./zenoh-debs/*.deb
```

脚本固定 tag `1.9.0`，启用 Zenoh reliability API，关闭 shared-memory，并输出 `libzenohc`、开发头文件及 `zenoh-cpp` 包。不要把其他发行版构建出的包复制到 Ubuntu 20.04。

随后在工作空间根目录编译：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## 配置

主机目录不再包含端口或必需 IP：

```yaml
hosts:
  - groundStation0
  - drone1
  - drone2
```

旧 `IP` map 仍可作为临时 host/seed 来源，但新部署应使用 `hosts`。`all` 与 `all_drone` 只在路由数组中作为选择器使用。

Zenoh 全局配置：

```yaml
zenoh:
  mode: peer
  multicast_scouting: true
  gossip_scouting: true
  compression_enabled: false
  listen_endpoints: []
  connect_endpoints: []
  seed_from_ip_table: false
  seed_port: 7447
  service_timeout_ms: 1000
  service_worker_threads: 2
  service_queue_capacity: 64
```

组播被机载网络过滤时，在 `connect_endpoints` 中加入一个或多个静态 peer，例如 `tcp/192.168.123.6:7447`。TCP 与 QUIC 的对照配置见 [`zenoh_quic_example.yaml`](swarm_ros_bridge/config/zenoh_quic_example.yaml)。QUIC mixed reliability 只有在本项目构建脚本启用的 unstable reliability API 下才会把 `Reliable` 与 `BestEffort` 分别映射到 stream/datagram；官方配置语法见 [Zenoh QUIC 文档](https://zenoh.io/docs/manual/quic/)。

每条 topic 必须提供 `qos_class`：

```yaml
topics:
  - topic_name: /ekf_quat/ekf_odom
    msg_type: nav_msgs/Odometry
    qos_class: state
    srcIP: [drone1]
    dstIP: [groundStation0]
    max_freq: 30
    prefix: false
    same_prefix: false
```

| 类别 | Zenoh 设置 | 用途 |
|---|---|---|
| `command` | Reliable / RealTime / Block / Express | 一次性控制、目标与模式切换 |
| `state` | BestEffort / DataHigh / Drop / Express | Odometry 等连续状态，接收队列只留最新 |
| `bulk` | BestEffort / DataLow / Drop | Image、PointCloud2、MarkerArray，接收队列只留最新 |
| service | Reliable / InteractiveHigh / Block / Express | 固定由 bridge 使用，默认超时 1000 ms |

不再支持 `srcPort`。`srcIP`/`dstIP` 字段名为兼容现有配置宏暂时保留，但其中的值是 host 名，不是 IP。

图像仍支持 `imgResizeRate`、`imgJpegQuality`、`imgAdaptiveQuality`、质量上下限、目标带宽和调节步长。点云仍支持 `cloudCompress`、`cloudDownsample` 以及 `draco`/`pcl_octree` codec。Zenoh 通用压缩默认关闭，避免重复压缩 JPEG/Draco。

Draco 会自动识别标准 `float32 x/y/z` 和带打包 `rgb`/`rgba`（`FLOAT32` 或 `UINT32`）或独立 `uint8 r/g/b[/a]` 的 XYZRGB 点云。传输会保留完整 header、字段描述、宽高、字节序、`point_step`、`row_step`、`is_dense`、行填充和 intensity/ring 等附加字段；XYZ 坐标仍采用有损 14-bit 量化。显式启用 `cloudDownsample` 时，宽高对应降采样后的点云结构。

service 配置不再需要端口：

```yaml
services:
  - srv_name: /add_two_ints
    srv_type: swarm_ros_bridge/AddTwoInts
    serverIp: drone1
    clientIp: [groundStation0, drone2]
    prefix: true
```

客户端调用 `/drone1/add_two_ints`。服务端还会根据 envelope 的来源 host、service type 与 MD5 拒绝未授权或不匹配的请求。

## Key 空间与诊断

- fanout topic：`netbridge/v1/topic/<source>/fanout/<ros-topic>`
- 定向 topic：`netbridge/v1/topic/<source>/dst/<hostname>/<ros-topic>`
- service：`netbridge/v1/service/<server>/<ros-service>`
- liveliness：`netbridge/v1/alive/<hostname>`

`/swarm_bridge/diagnostics` 现在包含 transport、QoS、session/link 状态、peer/router 数、估算重连次数、接收队列丢弃、service 超时、envelope 解码错误，以及每个 Zenoh 节点的 liveliness 状态。

`bridge_tui` 的 Overview 页面会显示在线节点数；Nodes 页面会把配置中的节点和 Zenoh 动态发现的节点合并展示：`ONLINE` 表示当前持有 liveliness token，`OFFLINE` 表示曾经在线但 token 已消失，`UNKNOWN` 表示本机 bridge 启动后尚未观察到该节点，或本机诊断已超过 2.5 s 未更新。页面右上角同时显示诊断流为 `LIVE`、`STALE` 或 `WAIT`，避免把缓存中的旧状态误认为在线。节点详情还会显示当前状态持续时间和上线次数。TUI 每 500 ms 刷新，bridge 每 1 s 发布一次诊断；Zenoh 已产生 liveliness 事件后，TUI 通常会在约 1–2 s 内更新，非正常断网的发现时间还取决于 Zenoh transport 的失联检测时间。

TUI 会在运行时根据终端尺寸自动重排：窄屏使用顶部导航和紧凑列表，中屏上下分区，宽屏使用侧栏与并排详情；直接调整终端大小即可切换，无需重启。最小验证尺寸为 `40x12`，日常使用建议至少 `80x24`。

## 测试与部署

本地测试：

```bash
./devel/lib/swarm_ros_bridge/test_bridge_transport
./devel/lib/swarm_ros_bridge/test_zenoh_transport_smoke
./devel/lib/swarm_ros_bridge/test_tui_layout
```

第二项会在 loopback 上建立两个真实 Zenoh peer，验证 pub/sub、query/reply，以及节点上线和离线事件。完整实机验收、TCP/QUIC 交替测试、回滚门槛与记录表见 [`ZENOH_VALIDATION.md`](swarm_ros_bridge/docs/ZENOH_VALIDATION.md)。

启动方式保持不变，例如：

```bash
roslaunch swarm_ros_bridge example_bridge_drone.launch
roslaunch swarm_ros_bridge example_bridge_station.launch
# 同时启动 bridge 和 TUI
roslaunch swarm_ros_bridge bridge_with_tui_drone.launch
roslaunch swarm_ros_bridge bridge_with_tui_station.launch
```

新增自定义消息或 service 时，继续修改 [`msgs_macro.hpp`](swarm_ros_bridge/include/msgs_macro.hpp)，并保证所有节点使用相同构建产物与配置版本。

## Contributors

- Weiqi Gai 2025.01
- KengHou Hoi 2024.08
