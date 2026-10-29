#!/usr/bin/env python3
"""Forwards ROS 2 PointCloud2 topics to the Point Cloud Streamer plugin over local TCP.

Each frame is a u8 source index, a little-endian u32 point count and then x, y, z as float32 per
point. With `target_frame` and several topics, the bridge merges the newest sweep of every topic
into one cloud, source 0, whenever the first topic delivers a sweep. Each sweep is moved to where
the target frame is at the newest sweep's stamp, through `fixed_frame`.
"""

from collections import namedtuple
import socket
import struct

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

STREAMER_ADDRESS = ("127.0.0.1", 29571)
SEND_TIMEOUT_S = 1.0
MAX_SOURCES = 256
NANOSECONDS_PER_SECOND = 1_000_000_000

Sweep = namedtuple("Sweep", "frame_id stamp xyz")


def rotation_matrix(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)


class PointCloudBridge(Node):
    def __init__(self):
        super().__init__("point_cloud_bridge")
        topics = self.declare_parameter("topics", ["/lidar_points"]).value
        self.target_frame = self.declare_parameter("target_frame", "").value
        self.fixed_frame = self.declare_parameter("fixed_frame", "odom").value
        self.max_sweep_age_ns = int(
            self.declare_parameter("max_sweep_age_s", 0.5).value * NANOSECONDS_PER_SECOND
        )
        if len(topics) > MAX_SOURCES:
            raise ValueError(f"At most {MAX_SOURCES} topics")

        self.connection = None
        self.is_merging = bool(self.target_frame) and len(topics) > 1
        self.newest_sweeps = {}
        if self.target_frame:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        newest_only = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for source, topic in enumerate(topics):
            self.create_subscription(
                PointCloud2,
                topic,
                lambda cloud, source=source: self.forward(source, cloud),
                newest_only,
            )

    def newest_pose_stamp(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.target_frame, Time()
            )
        except TransformException:
            return None
        return Time.from_msg(transform.header.stamp)

    def transform(self, sweep, target_stamp):
        lookups = [
            lambda: self.tf_buffer.lookup_transform(self.target_frame, sweep.frame_id, sweep.stamp),
            lambda: self.tf_buffer.lookup_transform(self.target_frame, sweep.frame_id, Time()),
        ]
        pose_stamp = self.newest_pose_stamp()
        if pose_stamp is not None:
            lookups.insert(0, lambda: self.tf_buffer.lookup_transform_full(
                self.target_frame,
                min(target_stamp, pose_stamp),
                sweep.frame_id,
                min(sweep.stamp, pose_stamp),
                self.fixed_frame,
            ))
        for lookup in lookups:
            try:
                return lookup()
            except TransformException as error:
                last_error = error
        self.get_logger().warning(
            f"No transform for {sweep.frame_id}: {last_error}", throttle_duration_sec=5.0
        )
        return None

    def to_target_frame(self, sweep, target_stamp):
        transform = self.transform(sweep, target_stamp)
        if transform is None:
            return None

        rotation = rotation_matrix(transform.transform.rotation)
        translation = transform.transform.translation
        offset = np.array([translation.x, translation.y, translation.z], dtype=np.float32)
        return sweep.xyz @ rotation.T + offset

    def forward(self, source, cloud):
        points = point_cloud2.read_points(cloud, field_names=("x", "y", "z"))
        x, y, z = points["x"], points["y"], points["z"]
        is_return = np.isfinite(x + y + z) & ((x != 0) | (y != 0) | (z != 0))
        xyz = np.empty((np.count_nonzero(is_return), 3), dtype="<f4")
        xyz[:, 0], xyz[:, 1], xyz[:, 2] = x[is_return], y[is_return], z[is_return]
        sweep = Sweep(cloud.header.frame_id, Time.from_msg(cloud.header.stamp), xyz)

        if self.is_merging:
            self.merge(source, sweep)
        elif self.target_frame:
            self.send_in_target_frame(source, [sweep], sweep.stamp)
        else:
            self.send(source, sweep.xyz)

    def merge(self, source, sweep):
        self.newest_sweeps[source] = sweep
        if source != 0:
            return

        merged_stamp = max(current.stamp for current in self.newest_sweeps.values())
        current_sweeps = [
            current
            for current in self.newest_sweeps.values()
            if (merged_stamp - current.stamp).nanoseconds <= self.max_sweep_age_ns
        ]
        self.send_in_target_frame(0, current_sweeps, merged_stamp)

    def send_in_target_frame(self, source, sweeps, target_stamp):
        parts = [self.to_target_frame(sweep, target_stamp) for sweep in sweeps]
        parts = [part for part in parts if part is not None]
        if parts:
            self.send(source, np.concatenate(parts))

    def send(self, source, xyz):
        frame = struct.pack("<BI", source, len(xyz)) + xyz.astype("<f4", copy=False).tobytes()

        try:
            if self.connection is None:
                self.connection = socket.create_connection(STREAMER_ADDRESS)
                self.connection.settimeout(SEND_TIMEOUT_S)
                self.get_logger().info("Connected to the Oden Streamer")
            self.connection.sendall(frame)
        except OSError as error:
            if self.connection is None:
                self.get_logger().warning(
                    f"Oden Streamer not reachable: {error}", throttle_duration_sec=5.0
                )
            else:
                self.get_logger().warning(f"Lost Oden Streamer: {error}")
                self.connection.close()
            self.connection = None


def main():
    rclpy.init()
    try:
        rclpy.spin(PointCloudBridge())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
