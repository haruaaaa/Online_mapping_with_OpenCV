#!/usr/bin/env python3
"""
High-performance Livox MID-360 PointCloud2 to 2D LaserScan converter & Kobuki Velocity Bridge.
Tailored for StarLine Hackathon 2026 (TurtleBot 2 + Livox MID-360) and compatible with HSL26:
- Slices 3D point cloud by Z to eliminate floor reflections and ceiling clutter.
- Automatically transforms points into base_link when TF is available (z: [0.15, 0.60]m).
- Masks out robot chassis blind zone (<0.18m).
- Bins 3D points into a continuous 360-degree 2D LaserScan for SLAM Toolbox and Nav2.
- Automatically relays /cmd_vel from Nav2 into Kobuki's /commands/velocity topic.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import PointCloud2, LaserScan
from tf2_ros import Buffer, TransformListener


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),       2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w),       2.0 * (y * z + x * w),       1.0 - 2.0 * (x * x + y * y)]
    ])


class LivoxToLaserScan(Node):
    def __init__(self):
        super().__init__('livox_to_laserscan')

        # Parameters
        self.declare_parameter('cloud_topic', '/livox/lidar')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('target_frame', 'livox')
        self.declare_parameter('min_height_base', 0.15)  # In base_link: above floor (floor is z=0)
        self.declare_parameter('max_height_base', 0.60)  # In base_link: below maze top
        self.declare_parameter('min_height_lidar', -0.12) # In lidar frame fallback
        self.declare_parameter('max_height_lidar', 0.35)  # In lidar frame fallback
        self.declare_parameter('min_range', 0.18)        # Clears TurtleBot 2 chassis (radius 0.177m)
        self.declare_parameter('max_range', 8.0)         # Max effective maze range
        self.declare_parameter('num_rays', 720)          # 0.5 deg angular resolution (360 deg)

        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.min_height_base = float(self.get_parameter('min_height_base').value)
        self.max_height_base = float(self.get_parameter('max_height_base').value)
        self.min_height_lidar = float(self.get_parameter('min_height_lidar').value)
        self.max_height_lidar = float(self.get_parameter('max_height_lidar').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.num_rays = int(self.get_parameter('num_rays').value)

        # Precompute scan angles and parameters
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = (2.0 * math.pi) / self.num_rays

        # TF Buffer for dynamic base_link transform lookup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lidar_rot = None
        self.lidar_shift = None

        # Publishers and Subscribers
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.pointcloud_callback,
            10
        )

        # Kobuki velocity bridge (/cmd_vel -> /commands/velocity)
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self.cmd_vel_cb, 10
        )
        self.commands_velocity_pub = self.create_publisher(
            Twist, '/commands/velocity', 10
        )

        self.get_logger().info(
            f" [LivoxToLaserScan] Initialized! Cloud: '{self.cloud_topic}' -> Scan: '{self.scan_topic}'\n"
            f"                     Z filter base: [{self.min_height_base:.2f}, {self.max_height_base:.2f}]m, "
            f"Bridge: /cmd_vel -> /commands/velocity"
        )

    def cmd_vel_cb(self, msg):
        twist_cmd = msg.twist if hasattr(msg, 'twist') else msg
        self.commands_velocity_pub.publish(twist_cmd)

    def pointcloud_callback(self, msg: PointCloud2):
        if msg.width == 0 or len(msg.data) == 0:
            return

        point_step = msg.point_step
        if point_step < 12:
            return

        # Vectorized parsing of X, Y, Z float32 coordinates from Livox PointCloud2
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, point_step)
        x = np.frombuffer(raw[:, 0:4].copy(), dtype=np.float32)
        y = np.frombuffer(raw[:, 4:8].copy(), dtype=np.float32)
        z = np.frombuffer(raw[:, 8:12].copy(), dtype=np.float32)

        # Try to resolve TF from lidar to base_link once (fixed mount)
        if self.lidar_rot is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
                t = tf.transform.translation
                rot = rotation_matrix(tf.transform.rotation)
                self.lidar_rot = rot
                self.lidar_shift = np.array([t.x, t.y, t.z], dtype=np.float32)
                self.get_logger().info(f" [LivoxToLaserScan] TF {self.base_frame} <- {msg.header.frame_id} locked!")
            except Exception:
                pass

        if self.lidar_rot is not None:
            # Transform all points to base_link frame
            pts = np.column_stack((x, y, z)) @ self.lidar_rot.T + self.lidar_shift
            xv = pts[:, 0]
            yv = pts[:, 1]
            zv = pts[:, 2]
            output_frame = self.base_frame
            z_min = self.min_height_base
            z_max = self.max_height_base
        else:
            # Fallback to local lidar frame
            xv = x
            yv = y
            zv = z
            output_frame = self.target_frame if self.target_frame else msg.header.frame_id
            z_min = self.min_height_lidar
            z_max = self.max_height_lidar

        # 1. Height filter: isolate maze walls & obstacles, filter out floor
        valid_z = (zv >= z_min) & (zv <= z_max)
        if not np.any(valid_z):
            return

        xv = xv[valid_z]
        yv = yv[valid_z]

        # 2. Distance filter: discard points within robot blind zone and beyond max range
        r = np.hypot(xv, yv)
        valid_r = (r >= self.min_range) & (r <= self.max_range)
        if not np.any(valid_r):
            return

        xv = xv[valid_r]
        yv = yv[valid_r]
        rv = r[valid_r]

        # 3. Calculate azimuth angle for each obstacle point
        angles = np.arctan2(yv, xv)

        # 4. Bin points into 2D LaserScan rays (min distance in each angular bin)
        bins = np.floor((angles - self.angle_min) / self.angle_increment).astype(np.int32)
        bins = np.clip(bins, 0, self.num_rays - 1)

        ranges = np.full(self.num_rays, np.inf, dtype=np.float32)
        np.minimum.at(ranges, bins, rv)

        # 5. Populate and publish LaserScan message
        scan_msg = LaserScan()
        scan_msg.header = msg.header
        scan_msg.header.frame_id = output_frame

        scan_msg.angle_min = float(self.angle_min)
        scan_msg.angle_max = float(self.angle_max)
        scan_msg.angle_increment = float(self.angle_increment)
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = float(self.min_range)
        scan_msg.range_max = float(self.max_range)
        scan_msg.ranges = [float(val) for val in ranges]

        self.scan_pub.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxToLaserScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
