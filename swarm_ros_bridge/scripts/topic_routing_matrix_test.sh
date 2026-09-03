#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
SETUP_FILE="${WORKSPACE_DIR}/devel/setup.bash"

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "Missing ${SETUP_FILE}; run catkin_make first." >&2
  exit 1
fi

# shellcheck disable=SC1090
source "${SETUP_FILE}"

TEST_LOG_DIR="$(mktemp -d /tmp/netbridge-topic-matrix.XXXXXX)"
PIDS=()
LAST_PID=""

cleanup() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]:-}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  echo "Logs: ${TEST_LOG_DIR}"
}
trap cleanup EXIT

start_process() {
  local name="$1"
  local master_uri="$2"
  shift 2
  ROS_MASTER_URI="${master_uri}" "$@" \
    >"${TEST_LOG_DIR}/${name}.log" 2>&1 &
  LAST_PID="$!"
  PIDS+=("${LAST_PID}")
}

stop_process() {
  local pid="$1"
  kill -INT "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

wait_for_master() {
  local master_uri="$1"
  local attempt
  for attempt in $(seq 1 60); do
    if ROS_MASTER_URI="${master_uri}" rosparam get /rosversion \
        >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  echo "ROS master did not start: ${master_uri}" >&2
  return 1
}

expect_field() {
  local master_uri="$1"
  local topic_field="$2"
  local expected="$3"
  local output
  output="$(ROS_MASTER_URI="${master_uri}" timeout 20 \
    rostopic echo -n 1 "${topic_field}")"
  if [[ "${output}" != *"${expected}"* ]]; then
    echo "Unexpected ${topic_field}: expected ${expected}, got ${output}" >&2
    return 1
  fi
}

expect_payload_set() {
  local master_uri="$1"
  local topic_prefix="$2"
  local identity="$3"
  local group="$4"
  # The compatibility Odometry codec normalizes header.frame_id to "world";
  # child_frame_id preserves the unique source identity.
  expect_field "${master_uri}" "${topic_prefix}/odom/child_frame_id" \
    "${identity}_${group}_base"
  expect_field "${master_uri}" "${topic_prefix}/image/header/frame_id" \
    "${identity}_${group}_camera"
  expect_field "${master_uri}" "${topic_prefix}/cloud/header/frame_id" \
    "${identity}_${group}_world"
  expect_field "${master_uri}" "${topic_prefix}/generic/name" \
    "${group}_${identity}"
}

expect_topic_type() {
  local master_uri="$1"
  local topic="$2"
  local expected_type="$3"
  local actual_type
  actual_type="$(ROS_MASTER_URI="${master_uri}" timeout 10 rostopic type "${topic}")"
  if [[ "${actual_type}" != "${expected_type}" ]]; then
    echo "Unexpected type for ${topic}: ${actual_type}" >&2
    return 1
  fi
}

expect_topic_absent() {
  local master_uri="$1"
  local topic="$2"
  local attempt
  for attempt in $(seq 1 30); do
    if ! ROS_MASTER_URI="${master_uri}" rostopic list 2>/dev/null | \
        grep -Fqx -- "${topic}"; then
      return 0
    fi
    sleep 0.1
  done
  echo "Unauthorized or quarantined topic is present: ${topic} (${master_uri})" >&2
  return 1
}

capture_diagnostics() {
  local master_uri="$1"
  local output_file="$2"
  ROS_MASTER_URI="${master_uri}" timeout 20 rostopic echo -n 1 \
    /swarm_bridge/diagnostics >"${output_file}"
}

diagnostic_has_state() {
  local master_uri="$1"
  local topic="$2"
  local direction="$3"
  local state="$4"
  local output_file="${TEST_LOG_DIR}/diagnostic-state.yaml"
  if ! ROS_MASTER_URI="${master_uri}" timeout 3 rostopic echo -n 1 \
      /swarm_bridge/diagnostics >"${output_file}" 2>/dev/null; then
    return 1
  fi
  python3 - "${output_file}" "${topic}" "${direction}" "${state}" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    document = next(doc for doc in yaml.safe_load_all(stream) if doc)
matches = [row for row in document.get("info", [])
           if row.get("name") == sys.argv[2]
           and row.get("direction") == sys.argv[3]
           and row.get("schema_state") == sys.argv[4]]
raise SystemExit(0 if len(matches) == 1 else 1)
PY
}

wait_for_diagnostic_state() {
  local master_uri="$1"
  local topic="$2"
  local direction="$3"
  local state="$4"
  local attempt
  for attempt in $(seq 1 30); do
    if diagnostic_has_state "${master_uri}" "${topic}" "${direction}" "${state}"; then
      return 0
    fi
    sleep 0.2
  done
  echo "Diagnostic state not reached: ${topic} ${direction} ${state}" >&2
  return 1
}

start_test_publisher() {
  local name="$1"
  local master_uri="$2"
  local group="$3"
  local identity="$4"
  local seed="$5"
  start_process "${name}" "${master_uri}" rosrun swarm_ros_bridge \
    bridge_test_publisher.py "__name:=${name}" _rate:=2 _point_count:=500 \
    "_random_seed:=${seed}" "_frame_id:=${identity}_${group}_world" \
    "_child_frame_id:=${identity}_${group}_base" \
    "_camera_frame_id:=${identity}_${group}_camera" \
    "_odom_topic:=/test/${group}/odom" \
    "_image_topic:=/test/${group}/image" \
    "_cloud_topic:=/test/${group}/cloud" \
    "_inflated_cloud_topic:=/unused/${identity}/${group}/cloud2" \
    "_marker_topic:=/unused/${identity}/${group}/markers"
}

STATION0_MASTER="http://127.0.0.1:11611"
STATION1_MASTER="http://127.0.0.1:11612"
DRONE1_MASTER="http://127.0.0.1:11613"
DRONE2_MASTER="http://127.0.0.1:11614"

start_process roscore_station0 "${STATION0_MASTER}" roscore -p 11611
start_process roscore_station1 "${STATION1_MASTER}" roscore -p 11612
start_process roscore_drone1 "${DRONE1_MASTER}" roscore -p 11613
start_process roscore_drone2 "${DRONE2_MASTER}" roscore -p 11614
wait_for_master "${STATION0_MASTER}"
wait_for_master "${STATION1_MASTER}"
wait_for_master "${DRONE1_MASTER}"
wait_for_master "${DRONE2_MASTER}"

UDP_OPTIONS='?rel=1;mixed_rel=1;multistream=1'
start_process bridge_station0 "${STATION0_MASTER}" roslaunch swarm_ros_bridge \
  topic_routing_matrix_test.launch hostname:=groundStation0 \
  control_listen_port:=19640 image_listen_port:=19641 cloud_listen_port:=19642
start_process bridge_station1 "${STATION1_MASTER}" roslaunch swarm_ros_bridge \
  topic_routing_matrix_test.launch hostname:=groundStation1 \
  control_listen_port:=19650 image_listen_port:=19651 cloud_listen_port:=19652

CONTROL_TARGETS='["tcp/127.0.0.1:19640","tcp/127.0.0.1:19650"]'
IMAGE_TARGETS="[\"udp/127.0.0.1:19641${UDP_OPTIONS}\",\"udp/127.0.0.1:19651${UDP_OPTIONS}\"]"
CLOUD_TARGETS='["tcp/127.0.0.1:19642","tcp/127.0.0.1:19652"]'
start_process bridge_drone1 "${DRONE1_MASTER}" roslaunch swarm_ros_bridge \
  topic_routing_matrix_test.launch hostname:=drone1 \
  control_listen_port:=19660 image_listen_port:=19661 cloud_listen_port:=19662 \
  "control_connect_endpoints:=${CONTROL_TARGETS}" \
  "image_connect_endpoints:=${IMAGE_TARGETS}" \
  "cloud_connect_endpoints:=${CLOUD_TARGETS}"
start_process bridge_drone2 "${DRONE2_MASTER}" roslaunch swarm_ros_bridge \
  topic_routing_matrix_test.launch hostname:=drone2 \
  control_listen_port:=19670 image_listen_port:=19671 cloud_listen_port:=19672 \
  "control_connect_endpoints:=${CONTROL_TARGETS}" \
  "image_connect_endpoints:=${IMAGE_TARGETS}" \
  "cloud_connect_endpoints:=${CLOUD_TARGETS}"

sleep 3

start_test_publisher station0_o2m "${STATION0_MASTER}" o2m station0 40
start_test_publisher drone1_m2o "${DRONE1_MASTER}" m2o drone1 41
start_test_publisher drone2_m2o "${DRONE2_MASTER}" m2o drone2 42
start_test_publisher drone1_m2m "${DRONE1_MASTER}" m2m drone1 43
start_test_publisher drone2_m2m "${DRONE2_MASTER}" m2m drone2 44

# NetworkInfo is intentionally not in MSGS_MACRO and exercises raw ROS1 bytes.
start_process generic_station0_o2m "${STATION0_MASTER}" rostopic pub -r 2 \
  /test/o2m/generic swarm_ros_bridge/NetworkInfo '{name: o2m_station0}'
start_process generic_drone1_m2o "${DRONE1_MASTER}" rostopic pub -r 2 \
  /test/m2o/generic swarm_ros_bridge/NetworkInfo '{name: m2o_drone1}'
start_process generic_drone2_m2o "${DRONE2_MASTER}" rostopic pub -r 2 \
  /test/m2o/generic swarm_ros_bridge/NetworkInfo '{name: m2o_drone2}'
start_process generic_drone1_m2m "${DRONE1_MASTER}" rostopic pub -r 2 \
  /test/m2m/generic swarm_ros_bridge/NetworkInfo '{name: m2m_drone1}'
start_process generic_drone2_m2m "${DRONE2_MASTER}" rostopic pub -r 2 \
  /test/m2m/generic swarm_ros_bridge/NetworkInfo '{name: m2m_drone2}'

# Multi-source conflict: the same rule is String on drone1 and UInt32 on drone2.
start_process conflict_drone1 "${DRONE1_MASTER}" rostopic pub -r 2 \
  /test/conflict/sources std_msgs/String '{data: drone1_schema}'
start_process conflict_drone2 "${DRONE2_MASTER}" rostopic pub -r 2 \
  /test/conflict/sources std_msgs/UInt32 '{data: 2}'

# Runtime conflict starts valid and is rebound after its first message arrives.
start_process runtime_string "${DRONE1_MASTER}" rostopic pub -r 2 \
  /test/conflict/runtime std_msgs/String '{data: initial_schema}'
RUNTIME_STRING_PID="${LAST_PID}"

# Many-to-one: station0 receives both source identities for every topic class.
for source in drone1 drone2; do
  expect_payload_set "${STATION0_MASTER}" "/${source}/test/m2o" \
    "${source}" m2o
done

# One-to-many: both destinations receive all four topics from station0.
for master_uri in "${DRONE1_MASTER}" "${DRONE2_MASTER}"; do
  expect_payload_set "${master_uri}" /test/o2m station0 o2m
done

# Many-to-many: both destinations receive both sources for every topic class.
for master_uri in "${STATION0_MASTER}" "${STATION1_MASTER}"; do
  for source in drone1 drone2; do
    expect_payload_set "${master_uri}" "/${source}/test/m2m" \
      "${source}" m2m
  done
done

expect_field "${STATION0_MASTER}" \
  /drone1/test/conflict/runtime/data initial_schema
expect_topic_type "${STATION0_MASTER}" \
  /drone1/test/conflict/runtime std_msgs/String

stop_process "${RUNTIME_STRING_PID}"
start_process runtime_uint32 "${DRONE1_MASTER}" rostopic pub -r 2 \
  /test/conflict/runtime std_msgs/UInt32 '{data: 99}'
wait_for_diagnostic_state "${DRONE1_MASTER}" \
  /test/conflict/runtime send conflict
wait_for_diagnostic_state "${STATION0_MASTER}" \
  /drone1/test/conflict/sources recv conflict

# A quarantined multi-source rule must not leave any ROS publisher behind.
expect_topic_absent "${STATION0_MASTER}" /drone1/test/conflict/sources
expect_topic_absent "${STATION0_MASTER}" /drone2/test/conflict/sources

# Every non-destination must stay free of bridge-created output topics.
for source in drone1 drone2; do
  for kind in odom image cloud generic; do
    expect_topic_absent "${STATION1_MASTER}" "/${source}/test/m2o/${kind}"
  done
done
for kind in odom image cloud generic; do
  expect_topic_absent "${STATION1_MASTER}" "/test/o2m/${kind}"
  expect_topic_absent "${DRONE1_MASTER}" "/drone2/test/m2o/${kind}"
  expect_topic_absent "${DRONE2_MASTER}" "/drone1/test/m2o/${kind}"
  expect_topic_absent "${DRONE1_MASTER}" "/drone2/test/m2m/${kind}"
  expect_topic_absent "${DRONE2_MASTER}" "/drone1/test/m2m/${kind}"
done
expect_topic_absent "${STATION1_MASTER}" /drone1/test/conflict/sources
expect_topic_absent "${STATION1_MASTER}" /drone2/test/conflict/sources

STATION0_DIAGNOSTICS="${TEST_LOG_DIR}/station0_diagnostics.yaml"
STATION1_DIAGNOSTICS="${TEST_LOG_DIR}/station1_diagnostics.yaml"
DRONE1_DIAGNOSTICS="${TEST_LOG_DIR}/drone1_diagnostics.yaml"
DRONE2_DIAGNOSTICS="${TEST_LOG_DIR}/drone2_diagnostics.yaml"
capture_diagnostics "${STATION0_MASTER}" "${STATION0_DIAGNOSTICS}"
capture_diagnostics "${STATION1_MASTER}" "${STATION1_DIAGNOSTICS}"
capture_diagnostics "${DRONE1_MASTER}" "${DRONE1_DIAGNOSTICS}"
capture_diagnostics "${DRONE2_MASTER}" "${DRONE2_DIAGNOSTICS}"

python3 - "${STATION0_DIAGNOSTICS}" "${STATION1_DIAGNOSTICS}" \
  "${DRONE1_DIAGNOSTICS}" "${DRONE2_DIAGNOSTICS}" <<'PY'
import re
import sys
import yaml


def load(path):
    with open(path, encoding="utf-8") as stream:
        return next(doc for doc in yaml.safe_load_all(stream) if doc)


documents = {
    "station0": load(sys.argv[1]),
    "station1": load(sys.argv[2]),
    "drone1": load(sys.argv[3]),
    "drone2": load(sys.argv[4]),
}

specs = {
    "odom": ("nav_msgs/Odometry", "odom_pose", "zenoh-tcp"),
    "image": ("sensor_msgs/Image", "jpeg", "zenoh-udp"),
    "cloud": (
        "sensor_msgs/PointCloud2",
        "pointcloud_draco",
        "zenoh-cloud-tcp",
    ),
    "generic": ("swarm_ros_bridge/NetworkInfo", "ros1", "zenoh-tcp"),
}
md5_by_type = {}
image_rows = []


def find_row(host, name, direction):
    matches = [row for row in documents[host].get("info", [])
               if row.get("name") == name
               and row.get("direction") == direction]
    if len(matches) != 1:
        raise SystemExit(
            f"{host}: expected one {direction} diagnostic for {name}, "
            f"got {len(matches)}"
        )
    return matches[0]


def assert_ready(host, name, kind):
    row = find_row(host, name, "recv")
    expected_type, expected_codec, expected_transport = specs[kind]
    actual = (row.get("msg_type"), row.get("codec"), row.get("transport"))
    expected = (expected_type, expected_codec, expected_transport)
    if actual != expected:
        raise SystemExit(f"{host}:{name}: expected {expected}, got {actual}")
    if row.get("schema_state") != "ready" or row.get("schema_error"):
        raise SystemExit(f"{host}:{name}: schema is not cleanly ready")
    md5 = str(row.get("schema_md5", ""))
    if not re.fullmatch(r"[0-9a-f]{32}", md5):
        raise SystemExit(f"{host}:{name}: invalid schema MD5 {md5!r}")
    previous = md5_by_type.setdefault(expected_type, md5)
    if previous != md5:
        raise SystemExit(
            f"{expected_type}: inconsistent MD5 {previous} versus {md5}"
        )
    if int(row.get("total_messages", 0)) <= 0:
        raise SystemExit(f"{host}:{name}: no decoded messages in diagnostics")
    if kind == "image":
        image_rows.append((host, name, row))


for source in ("drone1", "drone2"):
    for kind in specs:
        assert_ready("station0", f"/{source}/test/m2o/{kind}", kind)
        assert_ready("station0", f"/{source}/test/m2m/{kind}", kind)
        assert_ready("station1", f"/{source}/test/m2m/{kind}", kind)

for host in ("drone1", "drone2"):
    for kind in specs:
        assert_ready(host, f"/test/o2m/{kind}", kind)

runtime = find_row(
    "station0", "/drone1/test/conflict/runtime", "recv"
)
if runtime.get("msg_type") != "std_msgs/String" or \
        runtime.get("schema_state") != "ready":
    raise SystemExit("receiver accepted or exposed the runtime UInt32 schema")

for source in ("drone1", "drone2"):
    row = find_row(
        "station0", f"/{source}/test/conflict/sources", "recv"
    )
    if row.get("schema_state") != "conflict" or not row.get("schema_error"):
        raise SystemExit(f"multi-source schema conflict missing for {source}")

runtime_sender = find_row(
    "drone1", "/test/conflict/runtime", "send"
)
if runtime_sender.get("schema_state") != "conflict" or \
        not runtime_sender.get("schema_error"):
    raise SystemExit("runtime schema change did not quarantine the sender")

for host, name, row in image_rows:
    expected = int(row.get("expected_frames", 0))
    complete = int(row.get("transport_complete_frames", 0))
    decoded = int(row.get("decoded_frames", 0))
    bandwidth = float(row.get("effective_recv_bandwidth_kbps", 0.0))
    loss = float(row.get("image_loss_rate_pct", 100.0))
    if expected <= 0 or complete != expected or decoded != expected:
        raise SystemExit(
            f"{host}:{name}: incomplete image path "
            f"expected={expected}, complete={complete}, decoded={decoded}"
        )
    if loss > 0.01 or bandwidth <= 0.0:
        raise SystemExit(
            f"{host}:{name}: loss={loss}, bandwidth={bandwidth}"
        )

for host, document in documents.items():
    sessions = {(row.get("name"), row.get("transport"))
                for row in document.get("info", [])
                if str(row.get("name", "")).startswith("@zenoh/session/")}
    required = {
        ("@zenoh/session/control", "zenoh-tcp"),
        ("@zenoh/session/image", "zenoh-udp"),
        ("@zenoh/session/cloud", "zenoh-cloud-tcp"),
    }
    if not required.issubset(sessions):
        raise SystemExit(f"{host}: missing transport sessions: {required - sessions}")
PY

echo "PASS: topic-level many-to-one, one-to-many, and 2x2 many-to-many"
echo "PASS: Odometry, Image, Draco PointCloud2, and unregistered custom message"
echo "PASS: schema MD5/codec/transport consistency and image diagnostics"
echo "PASS: non-destination isolation and multi-source/runtime quarantine"
