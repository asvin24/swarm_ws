import math
import os
import signal
import threading
import time
from copy import deepcopy
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from mavros_msgs.msg import State, RCIn
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, PointCloud2, PointField
from tf2_ros import TransformBroadcaster


class ManualOverride(Exception):
    """Raised when a safety pilot takes manual control via RC sticks or mode switch."""


# =============================================================================
# GEOMETRY & MESSAGE UTILITIES
# =============================================================================
def quat_to_euler(q: Quaternion) -> tuple[float, float, float]:
    """Convert Quaternion to (roll, pitch, yaw) in radians."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_to_rot_matrix(q: Quaternion) -> np.ndarray:
    """Convert a Quaternion into a 3x3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n < 1e-8:
        return np.eye(3, dtype=np.float32)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z

    return np.array([
        [1.0 - (yy + zz), xy - wz,         xz + wy],
        [xy + wz,         1.0 - (xx + zz), yz - wx],
        [xz - wy,         yz + wx,         1.0 - (xx + yy)],
    ], dtype=np.float32)


def yaw_to_quat(yaw_rad: float) -> Quaternion:
    """Convert a yaw angle (radians) to a level Quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


def wrap_angle(angle_rad: float) -> float:
    """Wrap any angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def numpy_to_pointcloud2(points: np.ndarray, stamp, frame_id: str = "map") -> PointCloud2:
    """Convert an (N, 3) float32 NumPy array to a sensor_msgs/PointCloud2 message."""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.is_dense = True
    msg.is_bigendian = False
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.data = np.asarray(points, dtype=np.float32).tobytes()
    return msg


# =============================================================================
# SINGLE DRONE API NODE
# =============================================================================
class Drone(Node):
    """ROS 2 Drone controller with TF broadcasting, 3D PointCloud mapping, and RC Manual Override."""

    def __init__(
        self,
        name: str = "drone0",
        offset: tuple[float, float] = (0.0, 0.0),
        mapping: bool = True,
        voxel_size: float = 0.10,
        sim_time: bool | None = None,
        # Manual Override parameters
        monitor_rc: bool = True,
        monitor_mode: bool = True,
        rc_deadband: int = 50,
        rc_confirm_samples: int = 3,
        roll_idx: int = 0,
        pitch_idx: int = 1,
        allowed_modes: tuple[str, ...] = ("GUIDED", "LAND", "BRAKE"),
        override_modes: tuple[str, ...] = ("LOITER", "POSHOLD", "BRAKE"),
        only_when_armed: bool = True,
        exit_on_override: bool = False,
        on_override=None,
        service_timeout: float = 10.0,
    ):
        if sim_time is None:
            sim_time = os.environ.get("SWARM_REAL_HW", "0") != "1"

        super().__init__(
            f"{name}_node",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, bool(sim_time))],
        )
        self.name = name
        # How long to wait for MAVROS to answer a service call. A busy sim (several SITL
        # instances) can take several seconds to acknowledge commands.
        self.service_timeout = float(service_timeout)
        self.offset_x = float(offset[0])
        self.offset_y = float(offset[1])

        self.cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Telemetry State
        self.pos = [0.0, 0.0, 0.0]   # Local [x, y, z] (m)
        self.rpy = [0.0, 0.0, 0.0]   # [roll, pitch, yaw] (rad)
        self.state = State()
        self.pose_msg = PoseStamped()
        self.pose_msg.pose.orientation.w = 1.0
        self.imu = Imu()
        self.imu.orientation.w = 1.0
        self.has_imu = False
        self.gps = NavSatFix()
        self.scan: LaserScan | None = None
        self._last_tf_ns = -1

        # Local position (x, y, z) captured on the ground by takeoff(), just before arming.
        # Use it to fly relative to the takeoff point, e.g. target z = home[2] + alt.
        self.home: tuple[float, float, float] | None = None

        # Manual Override State
        self.monitor_rc = monitor_rc
        self.monitor_mode = monitor_mode
        self.rc_deadband = rc_deadband
        self.rc_confirm_samples = rc_confirm_samples
        self.roll_idx = roll_idx
        self.pitch_idx = pitch_idx
        self.only_when_armed = only_when_armed
        self.override_modes = tuple(m.upper() for m in override_modes)
        self.exit_on_override = exit_on_override
        self.on_override = on_override

        self.override_triggered = threading.Event()
        self.override_handled = threading.Event()
        self.override_reason: str | None = None
        self.rc_baseline: tuple[int, int] | None = None
        self._rc_moved_count = 0
        self._prev_mode: str | None = None
        self._FlightMode_ready = False  # Armed once mission connects/commands GUIDED
        self._allowed_modes = {m.upper() for m in allowed_modes}
        self._intended_mode: str | None = None

        # QoS Profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.prefix = f"/{name}/mavros"
        self._setup_interfaces(self.prefix)
        self.create_subscription(
            LaserScan, f"/{name}/scan", self.on_scan, self.sensor_qos, callback_group=self.cb_group
        )
        self.override_pub = self.create_publisher(
            Bool, f"/{name}/manual_override", self.latched_qos, callback_group=self.cb_group
        )

        # 3D Voxel PointCloud Database
        self.mapping = mapping
        self.voxel_size = float(voxel_size)
        self.voxel_db: dict[tuple[int, int, int], np.ndarray] = {}

        if self.mapping:
            self.cloud_pub = self.create_publisher(
                PointCloud2, f"/{name}/cloud", self.reliable_qos, callback_group=self.cb_group
            )
            self.create_timer(0.25, self.publish_cloud, callback_group=self.cb_group)

    def _setup_interfaces(self, prefix: str):
        self.prefix = prefix
        self.arm_client = self.create_client(CommandBool, f"{prefix}/cmd/arming", callback_group=self.cb_group)
        self.mode_client = self.create_client(SetMode, f"{prefix}/set_mode", callback_group=self.cb_group)
        self.takeoff_client = self.create_client(CommandTOL, f"{prefix}/cmd/takeoff", callback_group=self.cb_group)
        self.land_client = self.create_client(CommandTOL, f"{prefix}/cmd/land", callback_group=self.cb_group)
        self.stream_client = self.create_client(StreamRate, f"{prefix}/set_stream_rate", callback_group=self.cb_group)

        self.state_sub = self.create_subscription(State, f"{prefix}/state", self.on_state, self.reliable_qos, callback_group=self.cb_group)
        self.rc_sub = self.create_subscription(RCIn, f"{prefix}/rc/in", self.on_rc, self.sensor_qos, callback_group=self.cb_group)
        self.pose_sub = self.create_subscription(PoseStamped, f"{prefix}/local_position/pose", self.on_pose, self.sensor_qos, callback_group=self.cb_group)
        self.imu_sub = self.create_subscription(Imu, f"{prefix}/imu/data", self.on_imu, self.sensor_qos, callback_group=self.cb_group)
        self.gps_sub = self.create_subscription(NavSatFix, f"{prefix}/global_position/global", self.on_gps, self.sensor_qos, callback_group=self.cb_group)

        self.setpoint_pub = self.create_publisher(
            PoseStamped, f"{prefix}/setpoint_position/local", self.reliable_qos, callback_group=self.cb_group
        )

    # -------------------------------------------------------------------------
    # Manual Override Watchdog Logic
    # -------------------------------------------------------------------------
    def allow_mode(self, mode: str):
        """Whitelist a flight mode before your script intentionally switches to it."""
        with self._lock:
            self._allowed_modes.add(mode.upper())
            self._intended_mode = mode.upper()

    def check_override(self):
        """Raise ManualOverride if an override has been triggered."""
        if self.override_triggered.is_set():
            raise ManualOverride(f"[{self.name}] Manual override: {self.override_reason}")

    def is_overridden(self) -> bool:
        """Return True if manual override has been triggered."""
        return self.override_triggered.is_set()

    def reset_rc_baseline(self):
        """Re-center RC baseline (useful if sticks were trimmed before takeoff)."""
        with self._lock:
            self.rc_baseline = None
            self._rc_moved_count = 0

    def _trigger_override(self, reason: str, set_hold: bool):
        with self._lock:
            if self.override_triggered.is_set():
                return
            self.override_reason = reason
            self.override_triggered.set()

        self.get_logger().fatal(f"[{self.name}] MANUAL OVERRIDE TRIGGERED: {reason}")
        self.override_pub.publish(Bool(data=True))
        threading.Thread(target=self._handle_override, args=(set_hold,), daemon=True).start()

    def _handle_override(self, set_hold: bool):
        try:
            if set_hold:
                switched = False
                for mode in self.override_modes:
                    for _ in range(3):
                        with self._lock:
                            self._intended_mode = mode
                        if self._set_mode_confirmed(mode, timeout=2.5):
                            self.get_logger().warn(f"[{self.name}] Handed over control in {mode} mode.")
                            switched = True
                            break
                    if switched:
                        break
                if not switched:
                    self.get_logger().error(
                        f"[{self.name}] Could not switch to {self.override_modes}! Pilot must switch mode manually."
                    )

            if self.on_override is not None:
                try:
                    self.on_override(self.override_reason)
                except Exception as e:
                    self.get_logger().error(f"[{self.name}] on_override callback error: {e!r}")
        finally:
            self.override_handled.set()

        if self.exit_on_override:
            self.get_logger().warn(f"[{self.name}] Terminating host process (SIGINT)...")
            os.kill(os.getpid(), signal.SIGINT)
            time.sleep(3.0)
            os._exit(1)

    def _set_mode_confirmed(self, mode: str, timeout: float = 2.5) -> bool:
        req = SetMode.Request(custom_mode=mode.upper())
        res = self._call(self.mode_client, req, timeout=timeout)
        if not (res and res.mode_sent):
            return False
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self._lock:
                cur_mode = self.state.mode
            if cur_mode == mode.upper():
                return True
            time.sleep(0.05)
        return False

    # -------------------------------------------------------------------------
    # ROS 2 Callbacks & TF Broadcaster
    # -------------------------------------------------------------------------
    def _broadcast_base_tf(self, stamp):
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= self._last_tf_ns:
            return
        self._last_tf_ns = stamp_ns

        with self._lock:
            px, py, pz = self.pos
            q = deepcopy(self.imu.orientation if self.has_imu else self.pose_msg.pose.orientation)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = f"{self.name}/base_link"
        t.transform.translation.x = float(px + self.offset_x)
        t.transform.translation.y = float(py + self.offset_y)
        t.transform.translation.z = float(pz)
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

    def on_state(self, msg: State):
        with self._lock:
            self.state = msg
            new_mode = (msg.mode or "").upper()
            prev_mode = self._prev_mode
            self._prev_mode = new_mode
            watch_active = self.monitor_mode and self._FlightMode_ready
            expected = (new_mode in self._allowed_modes) or (new_mode == self._intended_mode)

        if not new_mode or prev_mode is None or new_mode == prev_mode:
            return

        if watch_active and not expected:
            self._trigger_override(
                f"Flight mode changed externally: {prev_mode} -> {new_mode}",
                set_hold=False,
            )

    def on_rc(self, msg: RCIn):
        if not self.monitor_rc or self.override_triggered.is_set():
            return
        ch = msg.channels
        if len(ch) <= max(self.roll_idx, self.pitch_idx):
            return

        roll, pitch = int(ch[self.roll_idx]), int(ch[self.pitch_idx])
        if roll == 0 or pitch == 0:  # 0 = no RC receiver signal
            return

        with self._lock:
            armed = self.state.armed
            if self.rc_baseline is None:
                self.rc_baseline = (roll, pitch)
                self.get_logger().info(f"[{self.name}] RC baseline locked: roll={roll}, pitch={pitch}")
                return

            if self.only_when_armed and not armed:
                self._rc_moved_count = 0
                return

            base_r, base_p = self.rc_baseline
            moved = abs(roll - base_r) > self.rc_deadband or abs(pitch - base_p) > self.rc_deadband
            self._rc_moved_count = (self._rc_moved_count + 1) if moved else 0
            count = self._rc_moved_count

        if count >= self.rc_confirm_samples:
            self._trigger_override(
                f"RC Stick input (roll={roll}, pitch={pitch}, baseline={self.rc_baseline})",
                set_hold=True,
            )

    def on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        r, pitch, y = quat_to_euler(msg.pose.orientation)
        with self._lock:
            self.pose_msg = deepcopy(msg)
            self.pos = [p.x, p.y, p.z]
            if not self.has_imu:
                self.rpy = [r, pitch, y]

    def on_imu(self, msg: Imu):
        r, pitch, y = quat_to_euler(msg.orientation)
        with self._lock:
            self.imu = deepcopy(msg)
            self.rpy = [r, pitch, y]
            self.has_imu = True

    def on_gps(self, msg: NavSatFix):
        with self._lock:
            self.gps = deepcopy(msg)

    def on_scan(self, msg: LaserScan):
        with self._lock:
            self.scan = msg
        self._broadcast_base_tf(stamp=msg.header.stamp)
        if self.mapping:
            self.register_scan_3d(msg)

    # -------------------------------------------------------------------------
    # 3D PointCloud Registration & Spatial Queries
    # -------------------------------------------------------------------------
    def scan_to_3d_points(self, scan: LaserScan, world: bool = True) -> np.ndarray:
        with self._lock:
            px, py, pz = self.pos
            q = deepcopy(self.imu.orientation if self.has_imu else self.pose_msg.pose.orientation)

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges), dtype=np.float32) * scan.angle_increment

        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges < scan.range_max - 0.05)
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float32)

        r = ranges[valid]
        a = angles[valid]

        pts_base = np.column_stack((r * np.cos(a), r * np.sin(a), np.full_like(r, 0.08)))
        rot_mat = quat_to_rot_matrix(q)
        pts_rotated = pts_base @ rot_mat.T

        tx = px + (self.offset_x if world else 0.0)
        ty = py + (self.offset_y if world else 0.0)
        pts_3d = pts_rotated + np.array([tx, ty, pz], dtype=np.float32)

        if pz > 0.35:
            pts_3d = pts_3d[pts_3d[:, 2] >= 0.05]
        return pts_3d

    def register_scan_3d(self, scan: LaserScan):
        pts_3d = self.scan_to_3d_points(scan, world=True)
        if pts_3d.shape[0] == 0:
            return
        voxel_keys = np.floor(pts_3d / self.voxel_size).astype(np.int32)
        with self._lock:
            for i in range(len(pts_3d)):
                key = (int(voxel_keys[i, 0]), int(voxel_keys[i, 1]), int(voxel_keys[i, 2]))
                self.voxel_db[key] = pts_3d[i]

    def publish_cloud(self):
        with self._lock:
            if self.scan is None or not self.voxel_db:
                return
            stamp = self.scan.header.stamp
            pts = np.array(list(self.voxel_db.values()), dtype=np.float32)
        self.cloud_pub.publish(numpy_to_pointcloud2(pts, stamp=stamp, frame_id="map"))

    def get_cloud(self) -> np.ndarray:
        with self._lock:
            if not self.voxel_db:
                return np.empty((0, 3), dtype=np.float32)
            return np.array(list(self.voxel_db.values()), dtype=np.float32)

    def get_local_cloud(self, radius: float = 5.0, z_min: float = 0.15, z_max: float = 2.2) -> np.ndarray:
        cloud = self.get_cloud()
        if cloud.shape[0] == 0:
            return cloud
        wx, wy, _ = self.get_pos(world=True)
        dx = cloud[:, 0] - wx
        dy = cloud[:, 1] - wy
        mask = ((dx * dx + dy * dy) <= radius * radius) & (cloud[:, 2] >= z_min) & (cloud[:, 2] <= z_max)
        return cloud[mask]

    def get_body_cloud(self, radius: float = 5.0, z_min: float = 0.15, z_max: float = 2.2) -> np.ndarray:
        local_pts = self.get_local_cloud(radius=radius, z_min=z_min, z_max=z_max)
        if local_pts.shape[0] == 0:
            return local_pts

        wx, wy, wz = self.get_pos(world=True)
        _, _, yaw = self.get_rpy(deg=False)
        c, s = math.cos(-yaw), math.sin(-yaw)

        rel = local_pts - np.array([wx, wy, wz], dtype=np.float32)
        bx = c * rel[:, 0] - s * rel[:, 1]
        by = s * rel[:, 0] + c * rel[:, 1]
        bz = rel[:, 2]
        return np.column_stack((bx, by, bz)).astype(np.float32)

    def is_box_clear(
        self,
        x_range: tuple[float, float] = (0.2, 1.4),
        y_range: tuple[float, float] = (-0.45, 0.45),
        z_range: tuple[float, float] = (-0.4, 0.4),
    ) -> bool:
        max_r = max(abs(x_range[0]), abs(x_range[1]), abs(y_range[0]), abs(y_range[1])) + 1.0
        body_pts = self.get_body_cloud(radius=max_r)
        if body_pts.shape[0] == 0:
            return True

        inside = (
            (body_pts[:, 0] >= x_range[0]) & (body_pts[:, 0] <= x_range[1]) &
            (body_pts[:, 1] >= y_range[0]) & (body_pts[:, 1] <= y_range[1]) &
            (body_pts[:, 2] >= z_range[0]) & (body_pts[:, 2] <= z_range[1])
        )
        return not bool(np.any(inside))

    def clear_cloud(self):
        with self._lock:
            self.voxel_db.clear()

    # -------------------------------------------------------------------------
    # Telemetry Getters
    # -------------------------------------------------------------------------
    def get_pos(self, world: bool = False) -> tuple[float, float, float]:
        with self._lock:
            x, y, z = self.pos
        return (x + self.offset_x, y + self.offset_y, z) if world else (x, y, z)

    def get_rpy(self, deg: bool = False) -> tuple[float, float, float]:
        with self._lock:
            r, p, y = self.rpy
        return (math.degrees(r), math.degrees(p), math.degrees(y)) if deg else (r, p, y)

    def get_imu(self) -> Imu:
        with self._lock:
            return deepcopy(self.imu)

    def get_scan(self) -> LaserScan | None:
        with self._lock:
            return deepcopy(self.scan)

    def min_dist(self, min_deg: float = -180.0, max_deg: float = 180.0) -> float:
        scan = self.get_scan()
        if scan is None:
            return float("inf")
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = np.degrees(scan.angle_min + np.arange(len(ranges), dtype=np.float32) * scan.angle_increment)
        valid = (
            np.isfinite(ranges)
            & (ranges >= scan.range_min)
            & (ranges <= scan.range_max)
            & (angles >= min_deg)
            & (angles <= max_deg)
        )
        return float(np.min(ranges[valid])) if np.any(valid) else float("inf")

    # -------------------------------------------------------------------------
    # Flight Control Actions
    # -------------------------------------------------------------------------
    def _call(self, client, req, timeout: float | None = None):
        timeout = self.service_timeout if timeout is None else timeout
        event = threading.Event()
        future = client.call_async(req)
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout=timeout):
            self.get_logger().warn(f"[{self.name}] Service {client.srv_name} timed out")
            return None
        return future.result()

    def set_stream_rate(self, rate: int = 20):
        if self.stream_client.wait_for_service(timeout_sec=5.0):
            req = StreamRate.Request(stream_id=0, message_rate=rate, on_off=True)
            self._call(self.stream_client, req)

    def connect(self):
        self.get_logger().info(f"[{self.name}] Connecting to MAVROS...")
        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            alt_prefix = f"/{self.name}"
            alt_client = self.create_client(CommandBool, f"{alt_prefix}/cmd/arming", callback_group=self.cb_group)
            if alt_client.wait_for_service(timeout_sec=2.0):
                self._setup_interfaces(alt_prefix)

        for srv in (self.arm_client, self.mode_client, self.takeoff_client, self.land_client, self.stream_client):
            while rclpy.ok() and not srv.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"[{self.name}]   waiting for {srv.srv_name}...")

        while rclpy.ok() and not self.state.connected:
            self.get_logger().info(f"[{self.name}]   waiting for FCU heartbeat...")
            time.sleep(0.5)

        self.set_stream_rate(rate=20)
        self.get_logger().info(
            f"[{self.name}] Connected! Manual Override watchdog active "
            f"(RC={self.monitor_rc}, Mode={self.monitor_mode}, Allowed={sorted(self._allowed_modes)})"
        )

    def set_mode(self, mode: str) -> bool:
        mode_up = mode.upper()
        self.allow_mode(mode_up)
        res = self._call(self.mode_client, SetMode.Request(custom_mode=mode_up))
        return bool(res and res.mode_sent)

    def arm(self) -> bool:
        res = self._call(self.arm_client, CommandBool.Request(value=True))
        return bool(res and res.success)

    def disarm(self) -> bool:
        res = self._call(self.arm_client, CommandBool.Request(value=False))
        return bool(res and res.success)

    def stop(self) -> bool:
        self.get_logger().warn(f"[{self.name}] Emergency BRAKE activated!")
        return self.set_mode("BRAKE")

    def takeoff(
        self,
        alt: float = 1.0,
        wait: bool = True,
        tol: float = 0.35,
        ready_timeout: float = 180.0,
        climb_timeout: float = 30.0,
    ) -> bool:
        """GUIDED + arm + climb to `alt` metres ABOVE THE TAKEOFF POINT.

        Records the ground position in `self.home` just before arming.

        Returns True when the drone is within `tol` of the target height (or, with
        wait=False, as soon as the takeoff command is accepted). Returns False if
        GUIDED/arming does not succeed within `ready_timeout` seconds, the takeoff
        command is rejected, or the height is not reached within `climb_timeout`.
        """
        self.check_override()  # never take control back from a pilot who already overrode
        self.set_stream_rate(rate=20)
        t0 = time.monotonic()

        # 1. GUIDED, confirmed by the FCU's reported mode (not just "request sent")
        self.allow_mode("GUIDED")
        while not self._set_mode_confirmed("GUIDED", timeout=2.5):
            self.check_override()
            if not rclpy.ok() or time.monotonic() - t0 > ready_timeout:
                self.get_logger().error(f"[{self.name}] Could not enter GUIDED within {ready_timeout:.0f} s")
                return False
            self.get_logger().info(f"[{self.name}] Waiting for GUIDED (position estimate not ready?)...")
            time.sleep(1.0)

        # Activate external mode-change watchdog once we have entered GUIDED
        with self._lock:
            self._FlightMode_ready = True
            self.home = (self.pos[0], self.pos[1], self.pos[2])
        z0 = self.home[2]

        # 2. Arm
        while not (self.arm() or self.state.armed):   # a slow reply can hide a successful arm
            self.check_override()
            if not rclpy.ok() or time.monotonic() - t0 > ready_timeout:
                self.get_logger().error(f"[{self.name}] Could not arm within {ready_timeout:.0f} s")
                return False
            self.get_logger().info(f"[{self.name}] Waiting for pre-arm checks...")
            time.sleep(1.5)

        # 3. Takeoff (ArduCopter's takeoff altitude is relative to the takeoff point)
        if not self._send_takeoff(alt, z0):
            self.get_logger().error(f"[{self.name}] Takeoff command rejected")
            return False

        self.get_logger().info(f"[{self.name}] Taking off to {alt:.1f}m above takeoff point...")
        if not wait:
            return True

        # 4. Wait for the climb, measured relative to the ground reading (baro offset safe)
        target_z = z0 + float(alt)
        t1 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t1 < climb_timeout:
            self.check_override()
            if abs(self.get_pos()[2] - target_z) <= tol:
                self.get_logger().info(f"[{self.name}] Hovering at {alt:.1f}m.")
                return True
            time.sleep(0.2)

        climbed = self.get_pos()[2] - z0
        self.get_logger().warn(
            f"[{self.name}] Target height not reached in {climb_timeout:.0f} s (climbed {climbed:.2f} m of {alt:.2f} m)"
        )
        return False

    def _wait_mode(self, mode: str, timeout: float) -> bool:
        """True if the FCU reports `mode` within `timeout` seconds."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if (self.state.mode or "").upper() == mode.upper():
                return True
            time.sleep(0.1)
        return False

    def _climbing(self, z0: float, timeout: float, rise: float = 0.15) -> bool:
        """True if the drone rises `rise` m above z0 within `timeout` seconds."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.get_pos()[2] - z0 > rise:
                return True
            time.sleep(0.2)
        return False

    def _send_takeoff(self, alt: float, z0: float, attempts: int = 2) -> bool:
        """Send the takeoff command. A reply that is slow or lost is OK if the drone is
        actually climbing (the FCU executed it); otherwise retry once."""
        for i in range(attempts):
            self.check_override()
            res = self._call(self.takeoff_client, CommandTOL.Request(altitude=float(alt)))
            if res and res.success:
                return True
            if self.state.armed and self._climbing(z0, timeout=5.0):
                self.get_logger().warn(f"[{self.name}] Takeoff reply {'failed' if res else 'timed out'}, "
                                       f"but the drone is climbing: continuing")
                return True
            if not self.state.armed:
                return False      # disarmed meanwhile (e.g. auto-disarm after idling on the ground)
            self.get_logger().warn(f"[{self.name}] Takeoff not confirmed, retrying ({i + 1}/{attempts})")
        return False

    def land(self, wait: bool = True, timeout: float = 90.0) -> bool:
        """Command LAND.

        With wait=True, blocks until the FCU disarms after touchdown (ArduCopter
        auto-disarms on landing), so it doesn't depend on z reading exactly 0.
        Returns True once disarmed (or, with wait=False, when the land command is
        accepted). Returns False if the command is rejected or the drone is still
        armed after `timeout` seconds (the FCU stays in LAND either way).
        """
        self.check_override()  # never command LAND over a pilot who took control
        self.allow_mode("LAND")
        res = self._call(self.land_client, CommandTOL.Request(altitude=0.0))
        if not (res and res.success) and not self._wait_mode("LAND", 3.0):
            self.get_logger().error(f"[{self.name}] Land command rejected")
            return False
        self.get_logger().info(f"[{self.name}] Landing...")
        if not wait:
            return True

        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout:
            self.check_override()  # abort the wait if the pilot takes over mid-landing
            if not self.state.armed:
                self.get_logger().info(f"[{self.name}] Landed and disarmed.")
                return True
            time.sleep(0.4)

        self.get_logger().warn(f"[{self.name}] Still armed {timeout:.0f} s after LAND")
        return False

    def goto(self, x: float, y: float, z: float, yaw: float | None = None):
        """Publish a non-blocking local position setpoint (ignored if manual override is active)."""
        self.check_override()
        msg = PoseStamped()
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation = yaw_to_quat(yaw) if yaw is not None else deepcopy(self.pose_msg.pose.orientation)
        self.setpoint_pub.publish(msg)

    def step_body(self, forward: float, left: float = 0.0, dz: float = 0.0, dyaw: float = 0.0, max_leash: float = 2.0):
        self.check_override()
        cx, cy, cz = self.get_pos()
        _, _, cyaw = self.get_rpy()

        step_x = forward * math.cos(cyaw) - left * math.sin(cyaw)
        step_y = forward * math.sin(cyaw) + left * math.cos(cyaw)

        dist = math.hypot(step_x, step_y)
        if dist > max_leash:
            step_x = max_leash * (step_x / dist)
            step_y = max_leash * (step_y / dist)

        self.goto(cx + step_x, cy + step_y, cz + dz, yaw=wrap_angle(cyaw + dyaw))

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float | None = None,
        tol: float = 0.45,
        timeout: float = 30.0,
        safe_dist: float = 0.0,
    ) -> bool:
        """Blocking move to (x, y, z) with manual override preemption and obstacle hold."""
        t0 = time.time()
        while rclpy.ok():
            self.check_override()

            if safe_dist > 0.0 and self.min_dist(-45.0, 45.0) < safe_dist:
                self.get_logger().warn(f"[{self.name}] Obstacle within {safe_dist}m! Holding.")
                cx, cy, cz = self.get_pos()
                self.goto(cx, cy, cz)
                return False

            self.goto(x, y, z, yaw)
            cx, cy, cz = self.get_pos()
            if math.dist((cx, cy, cz), (x, y, z)) <= tol:
                return True
            if time.time() - t0 > timeout:
                return False
            time.sleep(0.1)
        return False

    def wait(self, seconds: float):
        """Sleep that still honours manual override (use instead of time.sleep in missions)."""
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self.check_override()
            time.sleep(min(0.1, max(0.0, end - time.monotonic())))
        self.check_override()


# =============================================================================
# SWARM EXECUTOR MANAGER
# =============================================================================
class Swarm:
    """Manages ROS 2 initialization and background MultiThreadedExecutor spinning."""

    def __init__(
        self,
        names: tuple[str, ...] | list[str] = ("drone0",),
        spacing: float = 2.0,
        mapping: bool = True,
        sim_time: bool | None = None,
        **drone_kwargs,
    ):
        if not rclpy.ok():
            # No rclpy SIGINT handler: Ctrl+C raises KeyboardInterrupt in the main thread
            # while the executor keeps spinning, so a mission's except/finally can still
            # land the drone. (rclpy's default handler shuts ROS down first, which makes
            # every service call in the cleanup path fail.)
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

        self.executor = MultiThreadedExecutor(num_threads=max(4, len(names) * 2))
        self.drones: dict[str, Drone] = {}

        for i, name in enumerate(names):
            d = Drone(
                name=name,
                offset=(0.0, i * spacing),
                mapping=mapping,
                sim_time=sim_time,
                **drone_kwargs,
            )
            self.drones[name] = d
            self.executor.add_node(d)

        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def __getitem__(self, name: str) -> Drone:
        return self.drones[name]

    def check_overrides(self):
        """Raise ManualOverride if any drone in the swarm has been overridden."""
        for d in self.drones.values():
            d.check_override()

    def connect_all(self):
        for d in self.drones.values():
            d.connect()

    def _run_all(self, fn):
        """Run fn(drone) for every drone in parallel; re-raise ManualOverride from any worker."""
        errors = []

        def worker(d):
            try:
                fn(d)
            except ManualOverride as e:
                errors.append(e)

        threads = [threading.Thread(target=worker, args=(d,)) for d in self.drones.values()]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        if errors:
            raise errors[0]

    def takeoff_all(self, alt: float = 1.0):
        self._run_all(lambda d: d.takeoff(alt, True))

    def land_all(self):
        self._run_all(lambda d: d.land(True))

    def close(self):
        self.executor.shutdown()
        for d in self.drones.values():
            d.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()