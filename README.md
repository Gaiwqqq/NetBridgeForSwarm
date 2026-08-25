# NetBridgeForSwarm (Zenoh 1.9) [中文](README-zh.md)

NetBridgeForSwarm is a ROS1 Noetic bridge for multi-robot systems. This version uses three Zenoh sessions: UDP for images, a dedicated TCP session for PointCloud2, and a TCP control session for other topics, services, and liveliness. ROS serialization, adaptive JPEG quality, Draco/PCL point-cloud compression, prefix rules, and directed `to_drone_ids` routing are preserved.

Key changes:

- ZeroMQ, per-topic ports, and the custom UDP image-fragmentation runtime are removed.
- Topics use Zenoh pub/sub; services use Zenoh query/reply.
- Control, image, and cloud traffic use TCP, UDP, and an isolated TCP session respectively. The cloud session uses a separate scouting group; image/cloud sessions do not publish duplicate liveliness tokens.
- The versioned `NBZ1` envelope carries source, sequence, source time, ROS type/MD5, ROS header metadata, payload kind, and payload.
- Zenoh callbacks only enqueue work into bounded queues. Joinable bridge workers perform ROS deserialization and publication.
- `state` and `bulk` queues keep only the newest sample; `command` and service traffic use blocking congestion control.
- JPEG image transport now preserves `seq`, `stamp`, and `frame_id`.

## Dependencies and build

The project requires ROS Noetic/catkin, OpenCV, PCL, JPEG, the bundled FTXUI and Draco sources, and exactly `zenoh-c`/`zenoh-cpp` 1.9.0.

The official Zenoh 1.9.0 GNU binaries are not compatible with Ubuntu 20.04's GLIBC 2.31. This repository includes native Ubuntu 20.04 packages for amd64 and arm64; install the set matching the current machine:

```bash
sudo dpkg -i ./zenoh-debs/"$(dpkg --print-architecture)"/*.deb
```

To rebuild the packages, install the build dependencies first:

```bash
sudo apt-get install build-essential cmake git curl dpkg-dev ros-noetic-topic-tools
```

Install and pin Rust 1.93 with the official [`rustup`](https://rust-lang.org/tools/install.html). Load Cargo's environment in the current shell after installation; new shells load it automatically:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
  sh -s -- -y --profile minimal --default-toolchain 1.93.0
source "$HOME/.cargo/env"
rustup default 1.93.0
rustc --version
cargo --version
```

Then build the Zenoh Debian packages:

```bash
./swarm_ros_bridge/scripts/build_zenoh_1_9_debs.sh \
  --work-dir /tmp/zenoh-1.9-build \
  --output-dir ./zenoh-debs/"$(dpkg --print-architecture)"
sudo dpkg -i ./zenoh-debs/"$(dpkg --print-architecture)"/*.deb
```

The script pins tag `1.9.0`, enables the Zenoh reliability API, disables shared memory, and builds packages for the native architecture. Do not copy packages built on a newer Linux distribution to Ubuntu 20.04.

Build the ROS workspace normally after installing the packages:

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Configuration

The host inventory no longer contains ports or mandatory IP addresses:

```yaml
hosts:
  - groundStation0
  - drone1
  - drone2
```

The legacy `IP` map can temporarily supply host names and optional TCP seed addresses. New deployments should use `hosts`. `all` and `all_drone` remain routing selectors, not host entries.

Global Zenoh configuration:

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
  image_session:
    enabled: true
    multicast_scouting: true
    gossip_scouting: true
    compression_enabled: false
    listen_endpoints:
      - "udp/0.0.0.0:0?rel=1;mixed_rel=1;multistream=1"
    connect_endpoints: []
    seed_from_ip_table: false
    seed_port: 7448
  cloud_session:
    enabled: true
    multicast_scouting: true
    gossip_scouting: true
    multicast_address: "224.0.0.225:7446"
    compression_enabled: false
    listen_endpoints: ["tcp/0.0.0.0:0"]
    connect_endpoints: []
    seed_from_ip_table: false
    seed_port: 7449
```

Root endpoints belong to the TCP control session, `image_session` endpoints to UDP images, and `cloud_session` endpoints to the dedicated PointCloud2 TCP session. Image and cloud topics must remain `qos_class: bulk`. The cloud scouting address separates the two TCP discovery domains.

When multicast is unavailable, configure all three `connect_endpoints` sets, conventionally using ports 7447, 7448, and 7449. See [`zenoh_three_session_example.yaml`](swarm_ros_bridge/config/zenoh_three_session_example.yaml). UDP endpoints are unencrypted and should only be used on a trusted, isolated robot LAN.

Every topic must define `qos_class`:

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

| Class | Zenoh policy | Intended use |
|---|---|---|
| `command` | Reliable / RealTime / Block / Express | One-shot control, goals, and mode changes |
| `state` | BestEffort / DataHigh / Drop / Express | Continuous state such as Odometry; receiver keeps latest |
| `bulk` | BestEffort / DataLow / Drop | Image, PointCloud2, and MarkerArray; receiver keeps latest |
| service | Reliable / InteractiveHigh / Block / Express | Bridge-managed class; 1000 ms default timeout |

`srcPort` is no longer supported. The field names `srcIP` and `dstIP` remain for configuration compatibility, but their values are host names rather than IP addresses.

Image options (`imgResizeRate`, JPEG quality, adaptive quality bounds, target bandwidth, and cooldown) remain supported. Point clouds retain downsampling and Draco/PCL codecs. Generic Zenoh compression is disabled by default to avoid recompressing JPEG and Draco payloads.

Draco automatically recognizes standard `float32 x/y/z` clouds and XYZRGB clouds with packed `rgb`/`rgba` (`FLOAT32` or `UINT32`) or separate `uint8 r/g/b[/a]` fields. The bridge preserves the complete header, field schema, dimensions, endianness, `point_step`, `row_step`, `is_dense`, row padding, and additional fields such as intensity or ring; XYZ coordinates remain subject to lossy 14-bit quantization. When `cloudDownsample` is enabled explicitly, dimensions describe the downsampled cloud layout.

Services no longer have a port:

```yaml
services:
  - srv_name: /add_two_ints
    srv_type: swarm_ros_bridge/AddTwoInts
    serverIp: drone1
    clientIp: [groundStation0, drone2]
    prefix: true
```

Clients call `/drone1/add_two_ints`. The server rejects requests whose envelope source is unauthorized or whose service type/MD5 is incompatible.

## Key space and diagnostics

- Fanout topic: `netbridge/v1/topic/<source>/fanout/<ros-topic>`
- Directed topic: `netbridge/v1/topic/<source>/dst/<hostname>/<ros-topic>`
- Service: `netbridge/v1/service/<server>/<ros-service>`
- Liveliness: `netbridge/v1/alive/<hostname>`

`/swarm_bridge/diagnostics` reports separate control, image, and cloud session rows. Image receiver rows include sequence-gap loss, complete-frame success, and the rolling three-second effective decoded JPEG bandwidth. These are end-to-end bridge-frame metrics, not raw UDP datagram loss. Liveliness remains on the control session only.

The TUI reflows at runtime: narrow terminals use top navigation and compact lists, medium terminals stack list/detail panels, and wide terminals use a sidebar with side-by-side inspectors. Resize the terminal without restarting. The validated minimum is `40x12`; `80x24` or larger is recommended for routine use.

## Tests and deployment

Run the local tests after building:

```bash
./devel/lib/swarm_ros_bridge/test_bridge_transport
./devel/lib/swarm_ros_bridge/test_zenoh_transport_smoke
./devel/lib/swarm_ros_bridge/test_zenoh_three_session_smoke
./devel/lib/swarm_ros_bridge/test_topic_metrics
./devel/lib/swarm_ros_bridge/test_tui_layout
```

The three-session smoke test verifies isolated control TCP, image UDP, and cloud TCP paths. Run `./swarm_ros_bridge/scripts/three_node_transport_test.sh` for a repeatable three-ROS-master, two-drone/one-station many-to-one and one-to-many test covering Odometry, Image, and Draco PointCloud2. See [`ZENOH_VALIDATION.md`](swarm_ros_bridge/docs/ZENOH_VALIDATION.md) for physical-link acceptance and rollback criteria.

Launch commands remain unchanged:

```bash
roslaunch swarm_ros_bridge example_bridge_drone.launch
roslaunch swarm_ros_bridge example_bridge_station.launch
```

To add a custom ROS topic or service type, update [`msgs_macro.hpp`](swarm_ros_bridge/include/msgs_macro.hpp). All nodes must use the same build artifact and configuration version.

## Contributors

- Weiqi Gai 2025.01
- KengHou Hoi 2024.08
