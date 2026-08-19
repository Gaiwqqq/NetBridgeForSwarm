# Zenoh 迁移实机验证规程

本文档用于在 4–10 架无人机的真实无线链路上比较 Legacy ZeroMQ/UDP、Zenoh TCP 和 Zenoh QUIC mixed reliability。不要使用 `netem` 代替实机链路测试。

## 1. 测试前冻结项

每轮测试使用相同的：

- 飞行路线、飞行速度、机体与无线设备；
- ROS topic/service 配置、消息频率、JPEG/Draco 参数；
- 软件 Release 构建、CPU governor 和日志等级；
- 地面站位置、天线方向及信道配置。

记录每个节点的软件 commit、配置文件校验和、Zenoh Debian 包版本与架构。提前保存已经验证可用的 Legacy 二进制和完整配置作为回滚包；本分支不提供 Zenoh/Legacy 运行时双栈。

## 2. 采集指标

至少按 1 秒周期记录：

| 分类 | 指标 |
|---|---|
| 无线 | RSSI、SNR、PHY/link rate、丢包或重传统计 |
| 主机 | bridge CPU、RSS、整机 CPU、温度 |
| topic | 发送/接收频率、有效 payload kbps、P50/P95/P99 延迟、抖动、最新消息 age、队列丢弃 |
| service | 请求数、正确回复数、超时数、P50/P95/P99 RTT |
| transport | session/link、peer/router 数、重连数、解码错误 |
| 任务 | 离网时间、恢复时间、任务是否完成、人工异常记录 |

端到端延迟依赖各节点时钟同步。测试前验证 chrony/PTP 偏差，并把最大时钟偏差写入记录；否则只比较 RTT 与消息新鲜度。

建议按现场链路分布预先定义 RSSI/SNR 桶，例如 `strong`、`medium`、`weak`，并在同一桶内比较，不把不同无线质量的数据直接合并。

## 3. Legacy 基线

在安装 Zenoh 版本前运行已经验证的 ZeroMQ/UDP 回滚包。每个 RSSI/SNR 桶至少完成三次完整路线，包含：

1. 常规 topic 负载；
2. 图像和点云同时满负载；
3. service 周期请求；
4. 单节点离网、保持至少 10 秒、重新入网。

保存 rosbag、bridge 诊断、无线驱动统计和系统资源采样。基线原始数据只追加、不覆盖。

## 4. Zenoh TCP/QUIC 对照

采用 `Legacy → Zenoh TCP → Legacy → Zenoh QUIC` 的交替顺序，并在下一组反转 Zenoh 次序，以减小天气和无线环境随时间变化造成的偏差。

### TCP peer

每个节点使用唯一监听地址，例如：

```yaml
zenoh:
  mode: peer
  multicast_scouting: true
  gossip_scouting: true
  compression_enabled: false
  listen_endpoints: ["tcp/0.0.0.0:7447"]
  connect_endpoints: ["tcp/192.168.123.6:7447"]
```

若 multicast 工作正常，可清空 `connect_endpoints`；仍建议保留至少一个明确的 seed-fallback 测试轮次。

### QUIC mixed reliability

受控、隔离且可信的机群网可以使用无加密 UDP/QUIC endpoint：

```yaml
zenoh:
  mode: peer
  compression_enabled: false
  listen_endpoints:
    - "udp/0.0.0.0:7447?rel=1;multistream=1;mixed_rel=1"
  connect_endpoints:
    - "udp/192.168.123.6:7447?rel=1;multistream=1;mixed_rel=1"
```

公网或非可信网络必须改用带证书配置的 `quic/` endpoint。不要把上例视为通用安全配置。为每个监听节点分配不冲突的地址/端口，不能把示例地址原样部署到整组设备。

每种 transport 重复 Legacy 基线的四类负载。重点检查点云/图像满载时 odometry 和 service 是否被饿死，以及单 session TCP 是否出现跨流队头阻塞。

## 5. 功能矩阵

每个候选 transport 均需通过：

- 固定 source/destination、`all`、`all_drone`；
- 运行时 `to_drone_ids` 定向发送，且非目标节点不发布；
- `prefix`、`same_prefix` 和 `{id}` topic 展开；
- Image 的 `seq/stamp/frame_id`、JPEG 解码和自适应质量；
- PointCloud2 原始、下采样、Draco/PCL codec；
- service 正常回复、错误回复、未授权来源、type/MD5 不匹配和 1000 ms 超时；
- 节点退出/恢复、multicast 禁用后的静态 seed fallback；
- 所有接收及 service 队列保持有界，退出时 worker 可正常 join。

## 6. 结果表

每个 RSSI/SNR 桶分别填写，不同桶不能仅用总平均值比较：

| 架构 | 场景 | 关键流成功率 | 关键流 P95/P99 | bulk 新鲜吞吐 | service 成功率/P95 | 恢复时间 | CPU/RSS | 严重异常 |
|---|---|---:|---:|---:|---:|---:|---:|---|
| Legacy | | | | | | | | |
| Zenoh TCP | | | | | | | | |
| Zenoh QUIC | | | | | | | | |

传输选择顺序为：

1. 关键流和 service 成功率最高；
2. 关键流 P95/P99 延迟最低；
3. bulk 最新有效吞吐最高；
4. 指标接近时选择配置和运维更简单的 TCP。

## 7. 硬性验收与切换

候选方案必须同时满足：

- 无崩溃、payload 损坏、错误反序列化或无限增长队列；
- 已完成的 service 回复内容 100% 正确，整体成功率不低于 Legacy；
- 同一无线质量区间内，关键流 P95 延迟和 bulk 有效吞吐相对 Legacy 的恶化均不超过 10%；
- 断连恢复、关键流成功率或 bulk 新鲜度至少一项明显优于 Legacy；
- bridge CPU 与 RSS 相对 Legacy 的增幅均不超过 15%。

测试集群通过后一次性切换 4–10 架整组设备。旧 `pic_sockets` 源码已从当前仓库版本移除；需要回滚时应使用迁移前的 Git 提交、Legacy 二进制和配置，不要在当前 Zenoh bridge 中恢复或混用旧 UDP 图像链路。

若任一硬性条件失败，立即整组恢复 Legacy 包，保留失败轮次数据，并按失败指标决定调整 QoS、发送频率、JPEG/Draco 参数或 Zenoh endpoint；不要在同一任务中混用两种 bridge。
