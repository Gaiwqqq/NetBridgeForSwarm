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

TEST_LOG_DIR="$(mktemp -d /tmp/netbridge-three-node.XXXXXX)"
PIDS=()

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
  PIDS+=("$!")
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

expect_message() {
  local master_uri="$1"
  local topic="$2"
  ROS_MASTER_URI="${master_uri}" timeout 20 \
    rostopic echo -n 1 "${topic}" >/dev/null
}

STATION_MASTER="http://127.0.0.1:11511"
DRONE1_MASTER="http://127.0.0.1:11512"
DRONE2_MASTER="http://127.0.0.1:11513"

start_process roscore_station "${STATION_MASTER}" roscore -p 11511
start_process roscore_drone1 "${DRONE1_MASTER}" roscore -p 11512
start_process roscore_drone2 "${DRONE2_MASTER}" roscore -p 11513
wait_for_master "${STATION_MASTER}"
wait_for_master "${DRONE1_MASTER}"
wait_for_master "${DRONE2_MASTER}"

UDP_OPTIONS='?rel=1;mixed_rel=1;multistream=1'
start_process bridge_station "${STATION_MASTER}" roslaunch swarm_ros_bridge \
  three_node_bridge_test.launch hostname:=groundStation0 \
  control_listen_port:=19447 image_listen_port:=19448 cloud_listen_port:=19449
start_process bridge_drone1 "${DRONE1_MASTER}" roslaunch swarm_ros_bridge \
  three_node_bridge_test.launch hostname:=drone1 \
  control_listen_port:=19441 image_listen_port:=19451 cloud_listen_port:=19461 \
  'control_connect_endpoints:=["tcp/127.0.0.1:19447"]' \
  "image_connect_endpoints:=[\"udp/127.0.0.1:19448${UDP_OPTIONS}\"]" \
  'cloud_connect_endpoints:=["tcp/127.0.0.1:19449"]'
start_process bridge_drone2 "${DRONE2_MASTER}" roslaunch swarm_ros_bridge \
  three_node_bridge_test.launch hostname:=drone2 \
  control_listen_port:=19442 image_listen_port:=19452 cloud_listen_port:=19462 \
  'control_connect_endpoints:=["tcp/127.0.0.1:19447"]' \
  "image_connect_endpoints:=[\"udp/127.0.0.1:19448${UDP_OPTIONS}\"]" \
  'cloud_connect_endpoints:=["tcp/127.0.0.1:19449"]'

sleep 3

start_process publisher_station "${STATION_MASTER}" rosrun swarm_ros_bridge \
  bridge_test_publisher.py __name:=station_test_publisher _rate:=3 \
  _point_count:=1000 _random_seed:=30 _frame_id:=station_world \
  _camera_frame_id:=station_camera _odom_topic:=/test/o2m/odom \
  _image_topic:=/test/o2m/image _cloud_topic:=/test/o2m/cloud \
  _inflated_cloud_topic:=/unused/station/cloud2 \
  _marker_topic:=/unused/station/markers
start_process publisher_drone1 "${DRONE1_MASTER}" rosrun swarm_ros_bridge \
  bridge_test_publisher.py __name:=drone1_test_publisher _rate:=3 \
  _point_count:=1000 _random_seed:=31 _frame_id:=drone1_world \
  _camera_frame_id:=drone1_camera _odom_topic:=/test/m2o/odom \
  _image_topic:=/test/m2o/image _cloud_topic:=/test/m2o/cloud \
  _inflated_cloud_topic:=/unused/drone1/cloud2 \
  _marker_topic:=/unused/drone1/markers
start_process publisher_drone2 "${DRONE2_MASTER}" rosrun swarm_ros_bridge \
  bridge_test_publisher.py __name:=drone2_test_publisher _rate:=3 \
  _point_count:=1000 _random_seed:=32 _frame_id:=drone2_world \
  _camera_frame_id:=drone2_camera _odom_topic:=/test/m2o/odom \
  _image_topic:=/test/m2o/image _cloud_topic:=/test/m2o/cloud \
  _inflated_cloud_topic:=/unused/drone2/cloud2 \
  _marker_topic:=/unused/drone2/markers

# many-to-one: station must receive and distinguish both drones.
expect_field "${STATION_MASTER}" /drone1/test/m2o/image/header/frame_id drone1_camera
expect_field "${STATION_MASTER}" /drone2/test/m2o/image/header/frame_id drone2_camera
expect_field "${STATION_MASTER}" /drone1/test/m2o/cloud/header/frame_id drone1_world
expect_field "${STATION_MASTER}" /drone2/test/m2o/cloud/header/frame_id drone2_world
expect_message "${STATION_MASTER}" /drone1/test/m2o/odom
expect_message "${STATION_MASTER}" /drone2/test/m2o/odom

# one-to-many: both drones must receive every transport class from the station.
for master_uri in "${DRONE1_MASTER}" "${DRONE2_MASTER}"; do
  expect_field "${master_uri}" /test/o2m/image/header/frame_id station_camera
  expect_field "${master_uri}" /test/o2m/cloud/header/frame_id station_world
  expect_message "${master_uri}" /test/o2m/odom
done

DIAGNOSTICS_FILE="${TEST_LOG_DIR}/station_diagnostics.yaml"
ROS_MASTER_URI="${STATION_MASTER}" timeout 20 rostopic echo -n 1 \
  /swarm_bridge/diagnostics >"${DIAGNOSTICS_FILE}"
python3 - "${DIAGNOSTICS_FILE}" <<'PY'
import re
import sys

text = open(sys.argv[1], encoding="utf-8").read()
if "@zenoh/session/cloud" not in text or "zenoh-cloud-tcp" not in text:
    raise SystemExit("cloud TCP session is missing from diagnostics")

image_receivers = [block for block in re.split(r"\n  - \n", text)
                   if 'msg_type: "sensor_msgs/Image"' in block
                   and 'direction: "recv"' in block]
if len(image_receivers) != 2:
    raise SystemExit(f"expected two station image receivers, got {len(image_receivers)}")

def value(block, field):
    match = re.search(rf"{field}: ([0-9.]+)", block)
    if not match:
        raise SystemExit(f"missing {field}")
    return float(match.group(1))

for block in image_receivers:
    expected = value(block, "expected_frames")
    decoded = value(block, "decoded_frames")
    loss = value(block, "image_loss_rate_pct")
    success = value(block, "complete_frame_success_rate_pct")
    bandwidth = value(block, "effective_recv_bandwidth_kbps")
    if expected <= 0 or decoded != expected:
        raise SystemExit(f"incomplete image frames: {decoded}/{expected}")
    if loss > 0.01 or success < 99.99 or bandwidth <= 0:
        raise SystemExit(
            f"bad image metrics: loss={loss}, success={success}, bw={bandwidth}")
PY

echo "PASS: 2 drones + 1 station, many-to-one and one-to-many"
echo "PASS: control TCP + image UDP + cloud TCP"
echo "PASS: image loss/success/effective-bandwidth diagnostics"
