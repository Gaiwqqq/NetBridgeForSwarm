#!/usr/bin/env python3
"""Publish deterministic test data for every topic in config/default.yaml."""

import math
import random

import rospy
import sensor_msgs.point_cloud2 as point_cloud2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_MARKER_TOPIC = "/drone_0_ego_planner_node/rc_ctrl_marker"
DEFAULT_INFLATED_CLOUD_TOPIC = (
    "/drone_0_ego_planner_node/grid_map/occupancy_inflate"
)
DEFAULT_CLOUD_TOPIC = "/drone_0_ego_planner_node/grid_map/occupancy"
DEFAULT_ODOM_TOPIC = "/ekf_quat/ekf_odom"
DEFAULT_IMAGE_TOPIC = "/camera/color/image_raw"


def make_sphere_points(count, radius, seed):
    """Return points uniformly distributed on a sphere surface."""
    generator = random.Random(seed)
    points = []
    for _ in range(count):
        z_unit = generator.uniform(-1.0, 1.0)
        azimuth = generator.uniform(0.0, 2.0 * math.pi)
        xy_unit = math.sqrt(max(0.0, 1.0 - z_unit * z_unit))
        points.append(
            (
                radius * xy_unit * math.cos(azimuth),
                radius * xy_unit * math.sin(azimuth),
                radius * z_unit,
            )
        )
    return points


def make_color_bars(width, height, frame_id):
    """Build an RGB8 image containing eight vertical color bars."""
    colors = (
        (255, 255, 255),
        (255, 255, 0),
        (0, 255, 255),
        (0, 255, 0),
        (255, 0, 255),
        (255, 0, 0),
        (0, 0, 255),
        (0, 0, 0),
    )

    row = bytearray()
    for x in range(width):
        color_index = min(len(colors) - 1, x * len(colors) // width)
        row.extend(colors[color_index])

    image = Image()
    image.header.frame_id = frame_id
    image.height = height
    image.width = width
    image.encoding = "rgb8"
    image.is_bigendian = 0
    image.step = width * 3
    image.data = bytes(row) * height
    return image


class BridgeTestPublisher:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.point_count = int(rospy.get_param("~point_count", 10000))
        self.sphere_radius = float(rospy.get_param("~sphere_radius", 5.0))
        self.inflate_offset = float(rospy.get_param("~inflate_offset", 0.35))
        self.random_seed = int(rospy.get_param("~random_seed", 42))
        self.frame_id = str(rospy.get_param("~frame_id", "world"))
        self.child_frame_id = str(rospy.get_param("~child_frame_id", "base_link"))
        self.camera_frame_id = str(
            rospy.get_param("~camera_frame_id", "camera_color_optical_frame")
        )

        if self.rate_hz <= 0.0:
            raise ValueError("~rate must be greater than zero")
        if self.point_count <= 0:
            raise ValueError("~point_count must be greater than zero")
        if self.sphere_radius <= 0.0:
            raise ValueError("~sphere_radius must be greater than zero")
        if self.sphere_radius + self.inflate_offset <= 0.0:
            raise ValueError("~sphere_radius + ~inflate_offset must be greater than zero")

        self.marker_topic = str(
            rospy.get_param("~marker_topic", DEFAULT_MARKER_TOPIC)
        )
        self.inflated_cloud_topic = str(
            rospy.get_param(
                "~inflated_cloud_topic", DEFAULT_INFLATED_CLOUD_TOPIC
            )
        )
        self.cloud_topic = str(rospy.get_param("~cloud_topic", DEFAULT_CLOUD_TOPIC))
        self.odom_topic = str(rospy.get_param("~odom_topic", DEFAULT_ODOM_TOPIC))
        self.image_topic = str(rospy.get_param("~image_topic", DEFAULT_IMAGE_TOPIC))

        self.marker_publisher = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1
        )
        self.inflated_cloud_publisher = rospy.Publisher(
            self.inflated_cloud_topic, PointCloud2, queue_size=1
        )
        self.cloud_publisher = rospy.Publisher(
            self.cloud_topic, PointCloud2, queue_size=1
        )
        self.odom_publisher = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)
        self.image_publisher = rospy.Publisher(self.image_topic, Image, queue_size=1)

        # Large, static payloads are generated once. Only their headers change at runtime.
        self.cloud_message = self._make_cloud(False)
        self.inflated_cloud_message = self._make_cloud(True)
        self.image_message = make_color_bars(640, 480, self.camera_frame_id)

        self.sequence = 0
        self.started_at = rospy.Time.now()

        rospy.loginfo(
            "Bridge test publisher: %.3f Hz, %d points/cloud, 640x480 RGB bars",
            self.rate_hz,
            self.point_count,
        )
        rospy.loginfo(
            "Topics: marker=%s cloud=%s inflated_cloud=%s odom=%s image=%s",
            self.marker_topic,
            self.cloud_topic,
            self.inflated_cloud_topic,
            self.odom_topic,
            self.image_topic,
        )

    def _make_cloud(self, inflated):
        radius = self.sphere_radius + self.inflate_offset if inflated else self.sphere_radius
        seed = self.random_seed + 1 if inflated else self.random_seed
        points = make_sphere_points(self.point_count, radius, seed)
        return point_cloud2.create_cloud_xyz32(
            Header(frame_id=self.frame_id), points
        )

    def _make_markers(self, stamp, elapsed):
        marker_array = MarkerArray()
        for marker_id in range(8):
            angle = elapsed * 0.4 + marker_id * math.pi / 4.0
            marker = Marker()
            marker.header.seq = self.sequence
            marker.header.stamp = stamp
            marker.header.frame_id = self.frame_id
            marker.ns = "netbridge_test"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = 2.0 * math.cos(angle)
            marker.pose.position.y = 2.0 * math.sin(angle)
            marker.pose.position.z = 1.0 + 0.25 * math.sin(angle * 2.0)
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.r = (marker_id % 3) / 2.0
            marker.color.g = ((marker_id + 1) % 3) / 2.0
            marker.color.b = ((marker_id + 2) % 3) / 2.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0.0)
            marker_array.markers.append(marker)
        return marker_array

    def _make_odometry(self, stamp, elapsed):
        radius = 2.0
        angular_speed = 0.25
        angle = angular_speed * elapsed

        odometry = Odometry()
        odometry.header.seq = self.sequence
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.frame_id
        odometry.child_frame_id = self.child_frame_id
        odometry.pose.pose.position.x = radius * math.cos(angle)
        odometry.pose.pose.position.y = radius * math.sin(angle)
        odometry.pose.pose.position.z = 1.0
        odometry.pose.pose.orientation.z = math.sin((angle + math.pi / 2.0) / 2.0)
        odometry.pose.pose.orientation.w = math.cos((angle + math.pi / 2.0) / 2.0)
        odometry.twist.twist.linear.x = -radius * angular_speed * math.sin(angle)
        odometry.twist.twist.linear.y = radius * angular_speed * math.cos(angle)
        odometry.twist.twist.angular.z = angular_speed
        return odometry

    def publish_once(self):
        stamp = rospy.Time.now()
        elapsed = (stamp - self.started_at).to_sec()

        self.cloud_message.header.seq = self.sequence
        self.cloud_message.header.stamp = stamp
        self.inflated_cloud_message.header.seq = self.sequence
        self.inflated_cloud_message.header.stamp = stamp
        self.image_message.header.seq = self.sequence
        self.image_message.header.stamp = stamp

        self.marker_publisher.publish(self._make_markers(stamp, elapsed))
        self.cloud_publisher.publish(self.cloud_message)
        self.inflated_cloud_publisher.publish(self.inflated_cloud_message)
        self.odom_publisher.publish(self._make_odometry(stamp, elapsed))
        self.image_publisher.publish(self.image_message)

        self.sequence = (self.sequence + 1) & 0xFFFFFFFF

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.publish_once()
            rate.sleep()


def main():
    rospy.init_node("bridge_test_publisher")
    try:
        BridgeTestPublisher().run()
    except (ValueError, rospy.ROSInterruptException) as error:
        if not rospy.is_shutdown():
            rospy.logfatal("Bridge test publisher stopped: %s", error)
            raise


if __name__ == "__main__":
    main()
