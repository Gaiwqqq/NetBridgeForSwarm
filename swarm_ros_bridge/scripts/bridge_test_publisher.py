#!/usr/bin/env python3

import math
import os

import rospy
import yaml
from geometry_msgs.msg import Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

# ===== Test Knobs =====
# Set to None to follow the rate configured in default.yaml.
GLOBAL_RATE_OVERRIDE_HZ = None

TOPIC_RATE_OVERRIDES_HZ = {
    "/drone_0_ego_planner_node/rc_ctrl_marker": 20.0,
    "/drone_0_ego_planner_node/grid_map/occupancy_inflate": 5.0,
    "/drone_0_ego_planner_node/grid_map/occupancy": 5.0,
    "/ekf_quat/ekf_odom": 30.0,
    "/camera/color/image_raw": 24.0,
}

POINTS_PER_CLOUD = 10000
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 360


class PublisherBase(object):
    def __init__(self, topic_cfg):
        self.topic_cfg = topic_cfg
        self.topic_name = topic_cfg["topic_name"]
        self.max_freq = self.resolve_rate(topic_cfg)
        self.frame_id = topic_cfg.get("frame_id", "world")
        self.publisher = None
        self.sequence = 0

    def resolve_rate(self, topic_cfg):
        if GLOBAL_RATE_OVERRIDE_HZ is not None:
            return float(GLOBAL_RATE_OVERRIDE_HZ)
        if self.topic_name in TOPIC_RATE_OVERRIDES_HZ:
            return float(TOPIC_RATE_OVERRIDES_HZ[self.topic_name])
        return float(topic_cfg.get("max_freq", 10.0))

    def period(self):
        if self.max_freq <= 0.0:
            return 1.0
        return 1.0 / self.max_freq

    def build_header(self):
        header = Header()
        header.seq = self.sequence
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        self.sequence += 1
        return header

    def publish(self, _event):
        raise NotImplementedError


class MarkerPublisher(PublisherBase):
    def __init__(self, topic_cfg):
        super(MarkerPublisher, self).__init__(topic_cfg)
        self.publisher = rospy.Publisher(self.topic_name, MarkerArray, queue_size=2)

    def publish(self, _event):
        now = rospy.Time.now().to_sec()
        header = self.build_header()
        markers = MarkerArray()

        arrow = Marker()
        arrow.header = header
        arrow.ns = "bridge_test"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = 2.0 * math.cos(now * 0.4)
        arrow.pose.position.y = 2.0 * math.sin(now * 0.4)
        arrow.pose.position.z = 1.0 + 0.3 * math.sin(now * 0.8)
        arrow.pose.orientation.w = 1.0
        arrow.scale = Vector3(0.9, 0.18, 0.18)
        arrow.color = ColorRGBA(0.15, 0.8, 1.0, 0.95)
        markers.markers.append(arrow)

        for index in range(12):
          marker = Marker()
          marker.header = header
          marker.ns = "bridge_test_ring"
          marker.id = 100 + index
          marker.type = Marker.SPHERE
          marker.action = Marker.ADD
          angle = now * 0.6 + index * 2.0 * math.pi / 12.0
          marker.pose.position.x = 3.0 * math.cos(angle)
          marker.pose.position.y = 3.0 * math.sin(angle)
          marker.pose.position.z = 1.5 + 0.25 * math.sin(now + index * 0.4)
          marker.pose.orientation.w = 1.0
          marker.scale = Vector3(0.22, 0.22, 0.22)
          marker.color = ColorRGBA(1.0, 0.45 + 0.04 * index, 0.1, 0.9)
          markers.markers.append(marker)

        self.publisher.publish(markers)


class PointCloudPublisher(PublisherBase):
    def __init__(self, topic_cfg, inflated):
        super(PointCloudPublisher, self).__init__(topic_cfg)
        self.inflated = inflated
        self.publisher = rospy.Publisher(self.topic_name, PointCloud2, queue_size=1)
        self.target_points = int(topic_cfg.get("test_points", POINTS_PER_CLOUD))

    def publish(self, _event):
        now = rospy.Time.now().to_sec()
        header = self.build_header()
        points = []
        ring_count = 100 if self.inflated else 80
        phase_bias = 0.35 if self.inflated else 0.0
        for index in range(self.target_points):
            ratio = float(index) / float(max(1, self.target_points - 1))
            ring = index % ring_count
            spoke = index // ring_count
            angle = ratio * 18.0 * math.pi + now * (0.45 + phase_bias)
            radius = 0.8 + 5.4 * ratio
            swirl = 0.18 * math.sin(spoke * 0.11 + now * 1.4)
            x = (radius + swirl) * math.cos(angle) + 0.04 * math.sin(ring * 0.3 + now)
            y = (radius - swirl) * math.sin(angle) + 0.04 * math.cos(ring * 0.25 - now * 0.7)
            z = 0.9 * math.sin(radius * 1.6 - now * 1.2) + 0.35 * math.cos(angle * 0.5)
            if self.inflated:
                z += 0.25 * math.sin(spoke * 0.08 + now * 1.8)
            else:
                z += 0.12 * math.cos(spoke * 0.06 - now * 1.1)
            points.append((x, y, z))

        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud)


class OdomPublisher(PublisherBase):
    def __init__(self, topic_cfg):
        super(OdomPublisher, self).__init__(topic_cfg)
        self.publisher = rospy.Publisher(self.topic_name, Odometry, queue_size=5)

    def publish(self, _event):
        now = rospy.Time.now().to_sec()
        msg = Odometry()
        msg.header = self.build_header()
        msg.child_frame_id = "base_link"
        radius = 8.0
        angular_speed = 0.25
        yaw = now * angular_speed

        pose = Pose()
        pose.position.x = radius * math.cos(yaw)
        pose.position.y = radius * math.sin(yaw)
        pose.position.z = 1.5 + 0.4 * math.sin(now * 0.5)
        pose.orientation = Quaternion(
            0.0,
            0.0,
            math.sin(yaw * 0.5),
            math.cos(yaw * 0.5),
        )
        msg.pose.pose = pose

        msg.twist.twist.linear.x = -radius * angular_speed * math.sin(yaw)
        msg.twist.twist.linear.y = radius * angular_speed * math.cos(yaw)
        msg.twist.twist.linear.z = 0.2 * math.cos(now * 0.5)
        msg.twist.twist.angular.z = angular_speed
        self.publisher.publish(msg)


class ImagePublisher(PublisherBase):
    def __init__(self, topic_cfg):
        super(ImagePublisher, self).__init__(topic_cfg)
        self.publisher = rospy.Publisher(self.topic_name, Image, queue_size=1)
        self.width = int(topic_cfg.get("test_width", IMAGE_WIDTH))
        self.height = int(topic_cfg.get("test_height", IMAGE_HEIGHT))

    def publish(self, _event):
        now = rospy.Time.now().to_sec()
        image = Image()
        image.header = self.build_header()
        image.width = self.width
        image.height = self.height
        image.encoding = "rgb8"
        image.is_bigendian = 0
        image.step = self.width * 3

        raw = bytearray(image.step * image.height)
        phase = int(now * 40.0) % 255
        for y in range(self.height):
            for x in range(self.width):
                offset = (y * self.width + x) * 3
                raw[offset + 0] = (x + phase) % 255
                raw[offset + 1] = (2 * y + phase) % 255
                raw[offset + 2] = int(
                    127.5 * (1.0 + math.sin(x * 0.03 + y * 0.02 + now * 1.5))
                ) % 255

        image.data = bytes(raw)
        self.publisher.publish(image)


def make_publishers(config_path):
    with open(config_path, "r") as handle:
        config = yaml.safe_load(handle)

    publishers = []
    for topic_cfg in config.get("topics", []):
        msg_type = topic_cfg.get("msg_type", "")
        topic_name = topic_cfg.get("topic_name", "")

        if msg_type == "visualization_msgs/MarkerArray":
            publishers.append(MarkerPublisher(topic_cfg))
        elif msg_type == "sensor_msgs/PointCloud2":
            publishers.append(PointCloudPublisher(topic_cfg, "inflate" in topic_name))
        elif msg_type == "nav_msgs/Odometry":
            publishers.append(OdomPublisher(topic_cfg))
        elif msg_type == "sensor_msgs/Image":
            publishers.append(ImagePublisher(topic_cfg))
        else:
            rospy.logwarn("Skip unsupported test topic: %s (%s)", topic_name, msg_type)
    return publishers


def main():
    rospy.init_node("bridge_test_publisher", anonymous=False)
    default_config = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        "config",
        "default.yaml",
    )
    config_path = rospy.get_param("~config_file", default_config)
    rospy.loginfo("Loading bridge test topics from %s", config_path)

    publishers = make_publishers(config_path)
    timers = []
    for publisher in publishers:
        timers.append(rospy.Timer(rospy.Duration(publisher.period()), publisher.publish))
        rospy.loginfo(
            "Test publisher armed: %-48s type=%s rate=%.2fHz",
            publisher.topic_name,
            publisher.topic_cfg.get("msg_type", "unknown"),
            publisher.max_freq,
        )

    if not publishers:
        rospy.logwarn("No supported topics found in %s", config_path)
    rospy.spin()


if __name__ == "__main__":
    main()
