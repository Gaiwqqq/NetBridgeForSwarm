# NetBridgeForSwarm (Zenoh 1.9) [中文](README-zh.md)

NetBridgeForSwarm is a ROS1 Noetic bridge for multi-robot systems. This version uses one process-wide Zenoh session for topics, images, point clouds, and services while preserving ROS serialization, adaptive JPEG quality, Draco/PCL point-cloud compression, prefix rules, and directed `to_drone_ids` routing.

Key changes:

- ZeroMQ, per-topic ports, and the custom UDP image-fragmentation runtime are removed.
- Topics use Zenoh pub/sub; services use Zenoh query/reply.
- The versioned `NBZ1` envelope carries source, sequence, source time, ROS type/MD5, ROS header metadata, payload kind, and payload.
- Zenoh callbacks only enqueue work into bounded queues. Joinable bridge workers perform ROS deserialization and publication.
- `state` and `bulk` queues keep only the newest sample; `command` and service traffic use blocking congestion control.
- JPEG image transport now preserves `seq`, `stamp`, and `frame_id`.

## Dependencies and build

The project requires ROS Noetic/catkin, OpenCV, PCL, JPEG, the bundled FTXUI and Draco sources, and exactly `zenoh-c`/`zenoh-cpp` 1.9.0.

The official Zenoh 1.9.0 GNU binaries are not compatible with Ubuntu 20.04's GLIBC 2.31. Build native Debian packages on an Ubuntu 20.04 x86_64 builder and again on an Ubuntu 20.04 ARM64 builder:

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
  --output-dir ./zenoh-debs
sudo dpkg -i ./zenoh-debs/*.deb
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
```

If the airborne network blocks multicast, put one or more static peers such as `tcp/192.168.123.6:7447` in `connect_endpoints`. See [`zenoh_quic_example.yaml`](swarm_ros_bridge/config/zenoh_quic_example.yaml) for the controlled-network TCP/QUIC comparison. QUIC mixed reliability maps `Reliable` traffic to streams and `BestEffort` traffic to datagrams when built by the supplied script. See the [Zenoh QUIC documentation](https://zenoh.io/docs/manual/quic/) for endpoint details.

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

`/swarm_bridge/diagnostics` now reports transport and QoS class, session/link state, peer/router counts, estimated reconnections, bounded-queue drops, service timeouts, and envelope decode failures. `bridge_tui` continues to show topic rate, bandwidth, latency, jitter, and JPEG state.

The TUI reflows at runtime: narrow terminals use top navigation and compact lists, medium terminals stack list/detail panels, and wide terminals use a sidebar with side-by-side inspectors. Resize the terminal without restarting. The validated minimum is `40x12`; `80x24` or larger is recommended for routine use.

## Tests and deployment

Run the local tests after building:

```bash
./devel/lib/swarm_ros_bridge/test_bridge_transport
./devel/lib/swarm_ros_bridge/test_zenoh_transport_smoke
./devel/lib/swarm_ros_bridge/test_tui_layout
```

The smoke test starts two real loopback Zenoh peers and verifies both pub/sub and query/reply. See [`ZENOH_VALIDATION.md`](swarm_ros_bridge/docs/ZENOH_VALIDATION.md) for the physical-link baseline, alternating TCP/QUIC trials, acceptance criteria, and rollback gate.

Launch commands remain unchanged:

```bash
roslaunch swarm_ros_bridge example_bridge_drone.launch
roslaunch swarm_ros_bridge example_bridge_station.launch
```

To add a custom ROS topic or service type, update [`msgs_macro.hpp`](swarm_ros_bridge/include/msgs_macro.hpp). All nodes must use the same build artifact and configuration version.

## Contributors

- Weiqi Gai 2025.01
- KengHou Hoi 2024.08
