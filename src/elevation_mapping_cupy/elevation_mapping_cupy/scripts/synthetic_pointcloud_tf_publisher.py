#!/usr/bin/env python3
"""
Synthetic PointCloud2 + moving TF publisher for deterministic elevation-mapping bring-up.

Publishes:
  - TF: map -> base_link (moving in a square trajectory, optional yaw)
  - PointCloud2: /camera/depth/points in frame base_link

The pointcloud represents a *static* world (in 'map') transformed into the moving sensor frame.
This is the key property that lets you visually verify that the elevation map stays fixed in the
world and shifts correctly as the robot moves (no axis swap).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros


def _quat_from_yaw(yaw_rad: float) -> Tuple[float, float, float, float]:
    """Quaternion (x,y,z,w) for a pure yaw rotation."""
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _rotmat_from_yaw(yaw_rad: float) -> np.ndarray:
    """3x3 rotation matrix for a pure yaw rotation."""
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)


def _make_pointcloud2(points_xyz: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Create a PointCloud2 with xyz float32 fields from an (N,3) numpy array."""
    if points_xyz.ndim != 2 or points_xyz.shape[1] != 3:
        raise ValueError(f"points_xyz must have shape (N,3), got {points_xyz.shape}")
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = int(points_xyz.shape[0])
    msg.is_bigendian = False
    msg.is_dense = True
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.data = np.asarray(points_xyz, dtype=np.float32).tobytes()
    return msg


@dataclass(frozen=True)
class _Trajectory:
    speed_mps: float
    segment_s: float
    enable_yaw: bool
    yaw_rate_rps: float

    def pose_at(self, t: float) -> Tuple[np.ndarray, float]:
        """
        Deterministic square trajectory in map frame.

        Returns:
          translation (x,y,z) in meters, yaw in radians.
        """
        seg = self.segment_s
        v = self.speed_mps
        loop_s = 4.0 * seg
        tt = t % loop_s
        i = int(tt // seg)
        tau = tt - i * seg
        d = v * seg

        if i == 0:
            x, y = v * tau, 0.0
        elif i == 1:
            x, y = d, v * tau
        elif i == 2:
            x, y = d - v * tau, d
        else:
            x, y = 0.0, d - v * tau

        yaw = (self.yaw_rate_rps * t) if self.enable_yaw else 0.0
        return np.array([x, y, 0.0], dtype=np.float32), float(yaw)


class SyntheticPointcloudTfPublisher(Node):
    def __init__(self):
        super().__init__("synthetic_pointcloud_tf_publisher")

        # Fail loudly: these are the supported parameters for the demo.
        self.declare_parameter("pointcloud_topic", "/camera/depth/points")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_range_m", 6.0)
        self.declare_parameter("front_only", True)
        self.declare_parameter("ground_resolution_m", 0.05)
        self.declare_parameter("trajectory_speed_mps", 0.25)
        self.declare_parameter("trajectory_segment_s", 5.0)
        self.declare_parameter("enable_yaw", True)
        self.declare_parameter("yaw_rate_rps", 0.15)
        self.declare_parameter("enable_moving_obstacle", False)
        self.declare_parameter("moving_obstacle_height_m", 0.35)
        self.declare_parameter("moving_obstacle_size_m", 0.4)
        self.declare_parameter("moving_obstacle_resolution_m", 0.05)
        self.declare_parameter("moving_obstacle_motion_axis", "x")
        self.declare_parameter("moving_obstacle_x_m", 1.2)
        self.declare_parameter("moving_obstacle_x_min_m", 0.8)
        self.declare_parameter("moving_obstacle_x_max_m", 2.4)
        self.declare_parameter("moving_obstacle_y_m", 0.6)
        self.declare_parameter("moving_obstacle_y_min_m", -0.8)
        self.declare_parameter("moving_obstacle_y_max_m", 0.8)
        self.declare_parameter("moving_obstacle_speed_mps", 0.25)

        self._topic = self.get_parameter("pointcloud_topic").value
        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._max_range = float(self.get_parameter("max_range_m").value)
        self._front_only = bool(self.get_parameter("front_only").value)
        self._ground_resolution = max(float(self.get_parameter("ground_resolution_m").value), 0.01)
        self._enable_moving_obstacle = bool(self.get_parameter("enable_moving_obstacle").value)
        self._moving_obstacle_height = max(
            float(self.get_parameter("moving_obstacle_height_m").value), 0.01
        )
        self._moving_obstacle_size = max(
            float(self.get_parameter("moving_obstacle_size_m").value), 0.05
        )
        self._moving_obstacle_resolution = max(
            float(self.get_parameter("moving_obstacle_resolution_m").value), 0.02
        )
        motion_axis = str(self.get_parameter("moving_obstacle_motion_axis").value).lower()
        if motion_axis in ("x", "forward", "longitudinal"):
            self._moving_obstacle_motion_axis = "x"
        elif motion_axis in ("y", "lateral", "sideways"):
            self._moving_obstacle_motion_axis = "y"
        else:
            self.get_logger().warn(
                f"Unknown moving_obstacle_motion_axis='{motion_axis}', using 'x'."
            )
            self._moving_obstacle_motion_axis = "x"
        self._moving_obstacle_x = float(self.get_parameter("moving_obstacle_x_m").value)
        self._moving_obstacle_x_min = float(self.get_parameter("moving_obstacle_x_min_m").value)
        self._moving_obstacle_x_max = float(self.get_parameter("moving_obstacle_x_max_m").value)
        if self._moving_obstacle_x_max <= self._moving_obstacle_x_min:
            self._moving_obstacle_x_max = self._moving_obstacle_x_min + 0.1
        self._moving_obstacle_y = float(self.get_parameter("moving_obstacle_y_m").value)
        self._moving_obstacle_y_min = float(self.get_parameter("moving_obstacle_y_min_m").value)
        self._moving_obstacle_y_max = float(self.get_parameter("moving_obstacle_y_max_m").value)
        if self._moving_obstacle_y_max <= self._moving_obstacle_y_min:
            self._moving_obstacle_y_max = self._moving_obstacle_y_min + 0.1
        self._moving_obstacle_speed = max(
            float(self.get_parameter("moving_obstacle_speed_mps").value), 0.0
        )

        self._traj = _Trajectory(
            speed_mps=float(self.get_parameter("trajectory_speed_mps").value),
            segment_s=float(self.get_parameter("trajectory_segment_s").value),
            enable_yaw=bool(self.get_parameter("enable_yaw").value),
            yaw_rate_rps=float(self.get_parameter("yaw_rate_rps").value),
        )

        # TF broadcaster for map -> base_link.
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Pointcloud publisher (sensor_data QoS).
        qos = QoSPresetProfiles.get_from_short_key("sensor_data")
        self._pc_pub = self.create_publisher(PointCloud2, self._topic, qos)

        # Build a small static world in map frame (ground plane + one bump landmark).
        self._world_points = self._make_static_world()

        self._t0 = self.get_clock().now()
        self._last_obstacle_log_t = -1.0
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"Publishing TF '{self._map_frame}' -> '{self._base_frame}' and PointCloud2 '{self._topic}' "
            f"in frame '{self._base_frame}' at {rate_hz:.1f} Hz. "
            f"ground_resolution={self._ground_resolution:.3f} m, "
            f"moving_obstacle={self._enable_moving_obstacle}."
        )

    def _make_static_world(self) -> np.ndarray:
        # Ground plane grid.
        n_ground = int(math.ceil(16.0 / self._ground_resolution)) + 1
        xs = np.linspace(-8.0, 8.0, n_ground, dtype=np.float32)
        ys = np.linspace(-8.0, 8.0, n_ground, dtype=np.float32)
        X, Y = np.meshgrid(xs, ys, indexing="xy")
        Z = np.zeros_like(X, dtype=np.float32)

        # Add a deterministic "bump" landmark (small hill) to visualize motion.
        cx, cy = 2.5, -1.0
        r2 = (X - cx) ** 2 + (Y - cy) ** 2
        bump = 0.4 * np.exp(-r2 / (2.0 * (0.6**2))).astype(np.float32)
        Z = Z + bump

        pts = np.stack([X.reshape(-1), Y.reshape(-1), Z.reshape(-1)], axis=1)
        return pts.astype(np.float32)

    def _ping_pong(self, start: float, end: float, speed: float, t: float) -> float:
        span = end - start
        if speed <= 0.0:
            return start

        period = 2.0 * span / speed
        phase = (t % period) / period
        if phase < 0.5:
            return start + 2.0 * span * phase
        return end - 2.0 * span * (phase - 0.5)

    def _moving_obstacle_center_at(self, t: float) -> Tuple[float, float]:
        if self._moving_obstacle_motion_axis == "y":
            y = self._ping_pong(
                self._moving_obstacle_y_min,
                self._moving_obstacle_y_max,
                self._moving_obstacle_speed,
                t,
            )
            return self._moving_obstacle_x, float(y)

        x = self._ping_pong(
            self._moving_obstacle_x_min,
            self._moving_obstacle_x_max,
            self._moving_obstacle_speed,
            t,
        )
        return float(x), self._moving_obstacle_y

    def _make_moving_obstacle(self, t: float) -> np.ndarray:
        cx, cy = self._moving_obstacle_center_at(t)
        half = 0.5 * self._moving_obstacle_size
        n_xy = max(2, int(math.ceil(self._moving_obstacle_size / self._moving_obstacle_resolution)) + 1)
        n_z = max(2, int(math.ceil(self._moving_obstacle_height / self._moving_obstacle_resolution)) + 1)

        xs = np.linspace(cx - half, cx + half, n_xy, dtype=np.float32)
        ys = np.linspace(cy - half, cy + half, n_xy, dtype=np.float32)
        zs = np.linspace(0.0, self._moving_obstacle_height, n_z, dtype=np.float32)

        X, Y = np.meshgrid(xs, ys, indexing="xy")
        top = np.stack(
            [
                X.reshape(-1),
                Y.reshape(-1),
                np.full(X.size, self._moving_obstacle_height, dtype=np.float32),
            ],
            axis=1,
        )

        side_points = []
        Y_side, Z_side = np.meshgrid(ys, zs, indexing="xy")
        for x in (cx - half, cx + half):
            side_points.append(
                np.stack(
                    [
                        np.full(Y_side.size, x, dtype=np.float32),
                        Y_side.reshape(-1),
                        Z_side.reshape(-1),
                    ],
                    axis=1,
                )
            )

        X_side, Z_side = np.meshgrid(xs, zs, indexing="xy")
        for y in (cy - half, cy + half):
            side_points.append(
                np.stack(
                    [
                        X_side.reshape(-1),
                        np.full(X_side.size, y, dtype=np.float32),
                        Z_side.reshape(-1),
                    ],
                    axis=1,
                )
            )

        return np.concatenate([top, *side_points], axis=0).astype(np.float32)

    def _publish_tf(self, trans: np.ndarray, yaw: float, stamp_msg) -> None:
        t = TransformStamped()
        t.header.stamp = stamp_msg
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = float(trans[0])
        t.transform.translation.y = float(trans[1])
        t.transform.translation.z = float(trans[2])
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)

    def _world_to_sensor_points(self, trans: np.ndarray, yaw: float, t: float) -> np.ndarray:
        # Sensor pose in map: p_map = R * p_sensor + t  => p_sensor = R^T * (p_map - t)
        R = _rotmat_from_yaw(yaw)
        world_points = self._world_points
        if self._enable_moving_obstacle:
            moving_obstacle = self._make_moving_obstacle(t)
            world_points = np.concatenate([world_points, moving_obstacle], axis=0)

        pts = world_points - trans[None, :]
        pts_sensor = (R.T @ pts.T).T

        # Simple range / FOV gating to keep message sizes reasonable.
        d = np.linalg.norm(pts_sensor[:, :2], axis=1)
        keep = d <= self._max_range
        if self._front_only:
            keep &= pts_sensor[:, 0] > 0.05  # "in front" of sensor
        pts_sensor = pts_sensor[keep]
        return pts_sensor.astype(np.float32)

    def _tick(self) -> None:
        now = self.get_clock().now()
        t = (now - self._t0).nanoseconds * 1e-9
        trans, yaw = self._traj.pose_at(t)
        stamp = now.to_msg()

        self._publish_tf(trans, yaw, stamp)
        if self._enable_moving_obstacle and (t - self._last_obstacle_log_t) >= 1.0:
            cx, cy = self._moving_obstacle_center_at(t)
            self.get_logger().info(
                f"Moving obstacle center in {self._map_frame}: x={cx:.2f}, y={cy:.2f}, "
                f"height={self._moving_obstacle_height:.2f}, axis={self._moving_obstacle_motion_axis}."
            )
            self._last_obstacle_log_t = t
        pts_sensor = self._world_to_sensor_points(trans, yaw, t)
        msg = _make_pointcloud2(pts_sensor, frame_id=self._base_frame, stamp=stamp)
        self._pc_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = SyntheticPointcloudTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # launch_testing / signal handlers can already have shut down the context.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
