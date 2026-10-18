import math
import threading
import time
from copy import deepcopy
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from nav_msgs.msg import OccupancyGrid


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


def yaw_to_quat(yaw_rad: float) -> Quaternion:
    """Convert yaw angle (radians) to a level Quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


def trace_ray(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    """Bresenham line algorithm returning grid cells along a laser beam."""
    cells = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    if dx > dy:
        err = dx / 2.0
        while x != x1:
            cells.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            cells.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    cells.append((x1, y1))
    return cells


class Drone(Node):
    """Simple, all-in-one ROS 2 Drone API for flight control, LiDAR, and 2D mapping."""

    def __init__(
        self,
        name: str = "drone0",
        offset: tuple[float, float] = (0.0, 0.0),
        mapping: bool = True,
        map_size: float = 60.0,
        map_res: float = 0.20,
    ):
        super().__init__(f"{name}_node")
        self.name = name
        self.offset_x = float(offset[0])
        self.offset_y = float(offset[1])

        self.cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        # ---------------- Simple Public Attributes ----------------
        self.pos = [0.0, 0.0, 0.0]   # [x, y, z] in local frame
        self.rpy = [0.0, 0.0, 0.0]   # [roll, pitch, yaw] in radians
        self.state = State()
        self.pose_msg = PoseStamped()
        self.imu = Imu()
        self.gps = NavSatFix()
        self.scan: LaserScan | None = None

        # ---------------- QoS Profiles ----------------
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Support both /{name}/mavros/... and /{name}/... namespaces automatically
        self.prefix = f"/{name}/mavros"
        self._setup_interfaces(self.prefix)

        # LiDAR scan is always on /{name}/scan
        self.create_subscription(LaserScan, f"/{name}/scan", self.on_scan, self.sensor_qos, callback_group=self.cb_group)

        # ---------------- 2D Occupancy Grid Mapping ----------------
        self.mapping = mapping
        self.map_res = float(map_res)
        self.map_w = int(map_size / self.map_res)
        self.map_h = int(map_size / self.map_res)
        self.origin_x = -map_size / 2.0
        self.origin_y = -map_size / 2.0
        self.log_map = np.zeros((self.map_h, self.map_w), dtype=np.float32)

        if self.mapping:
            self.map_pub = self.create_publisher(OccupancyGrid, f"/{name}/map", self.reliable_qos, callback_group=self.cb_group)
            self.create_timer(0.5, self.publish_map, callback_group=self.cb_group)

    def _setup_interfaces(self, prefix: str):
        self.prefix = prefix
        self.arm_client = self.create_client(CommandBool, f"{prefix}/cmd/arming", callback_group=self.cb_group)
        self.mode_client = self.create_client(SetMode, f"{prefix}/set_mode", callback_group=self.cb_group)
        self.takeoff_client = self.create_client(CommandTOL, f"{prefix}/cmd/takeoff", callback_group=self.cb_group)
        self.land_client = self.create_client(CommandTOL, f"{prefix}/cmd/land", callback_group=self.cb_group)

        self.state_sub = self.create_subscription(State, f"{prefix}/state", self.on_state, self.reliable_qos, callback_group=self.cb_group)
        self.pose_sub = self.create_subscription(PoseStamped, f"{prefix}/local_position/pose", self.on_pose, self.sensor_qos, callback_group=self.cb_group)
        self.imu_sub = self.create_subscription(Imu, f"{prefix}/imu/data", self.on_imu, self.sensor_qos, callback_group=self.cb_group)
        self.gps_sub = self.create_subscription(NavSatFix, f"{prefix}/global_position/global", self.on_gps, self.sensor_qos, callback_group=self.cb_group)

        self.setpoint_pub = self.create_publisher(
            PoseStamped, f"{prefix}/setpoint_position/local", self.sensor_qos, callback_group=self.cb_group
        )

    # =========================================================================
    # CALLBACKS
    # =========================================================================
    def on_state(self, msg: State):
        with self._lock:
            self.state = msg

    def on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        r, pitch, y = quat_to_euler(msg.pose.orientation)
        with self._lock:
            self.pose_msg = deepcopy(msg)
            self.pos = [p.x, p.y, p.z]
            self.rpy = [r, pitch, y]

    def on_imu(self, msg: Imu):
        with self._lock:
            self.imu = deepcopy(msg)

    def on_gps(self, msg: NavSatFix):
        with self._lock:
            self.gps = deepcopy(msg)

    def on_scan(self, msg: LaserScan):
        with self._lock:
            self.scan = msg
        if self.mapping:
            self.update_map(msg)

    # =========================================================================
    # GETTERS (For Mission Scripts)
    # =========================================================================
    def get_pos(self, world: bool = False) -> tuple[float, float, float]:
        with self._lock:
            x, y, z = self.pos
        if world:
            return (x + self.offset_x, y + self.offset_y, z)
        return (x, y, z)

    def get_rpy(self, deg: bool = False) -> tuple[float, float, float]:
        with self._lock:
            r, p, y = self.rpy
        if deg:
            return (math.degrees(r), math.degrees(p), math.degrees(y))
        return (r, p, y)

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

        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = np.degrees(scan.angle_min + np.arange(len(ranges), dtype=np.float32) * scan.angle_increment)
        valid = (
            np.isfinite(ranges)
            & (ranges >= scan.range_min)
            & (ranges <= scan.range_max)
            & (angles >= min_deg)
            & (angles <= max_deg)
        )
        return float(np.min(ranges[valid])) if np.any(valid) else float("inf")

    def get_obstacles(self, world: bool = True, max_tilt: float = 12.0) -> np.ndarray:
        scan = self.get_scan()
        if scan is None:
            return np.empty((0, 2), dtype=np.float32)

        r, p, yaw = self.get_rpy(deg=True)
        if abs(r) > max_tilt or abs(p) > max_tilt:
            return np.empty((0, 2), dtype=np.float32)

        dx, dy, _ = self.get_pos(world=world)
        yaw_rad = math.radians(yaw)

        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges), dtype=np.float32) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges < scan.range_max - 0.05)
        if not np.any(valid):
            return np.empty((0, 2), dtype=np.float32)

        d = ranges[valid]
        ang = angles[valid] + yaw_rad
        return np.column_stack((dx + d * np.cos(ang), dy + d * np.sin(ang)))

    def get_map(self) -> np.ndarray:
        grid = np.full(self.log_map.shape, -1, dtype=np.int8)
        grid[self.log_map < -0.2] = 0
        grid[self.log_map > 0.2] = 100
        return grid

    # =========================================================================
    # FLIGHT ACTIONS (For Mission Scripts)
    # =========================================================================
    def _call(self, client, req, timeout: float = 5.0):
        event = threading.Event()
        future = client.call_async(req)
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout=timeout):
            self.get_logger().warn(f"[{self.name}] Service {client.srv_name} timed out")
            return None
        return future.result()

    def connect(self):
        """Wait for MAVROS services (auto-detecting namespace) and FCU heartbeat."""
        self.get_logger().info(f"[{self.name}] Connecting to MAVROS...")

        # Check if services are at /{name}/mavros/cmd/arming or /{name}/cmd/arming
        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            alt_prefix = f"/{self.name}"
            alt_client = self.create_client(CommandBool, f"{alt_prefix}/cmd/arming", callback_group=self.cb_group)
            if alt_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f"[{self.name}] Detected MAVROS namespace at '{alt_prefix}'")
                self._setup_interfaces(alt_prefix)

        for srv in (self.arm_client, self.mode_client, self.takeoff_client, self.land_client):
            while rclpy.ok() and not srv.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"[{self.name}]   waiting for service {srv.srv_name}...")

        self.get_logger().info(f"[{self.name}] All MAVROS services ready!")

    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode.upper()
        res = self._call(self.mode_client, req)
        return bool(res and res.mode_sent)

    def arm(self) -> bool:
        req = CommandBool.Request()
        req.value = True
        res = self._call(self.arm_client, req)
        return bool(res and res.success)

    def disarm(self) -> bool:
        req = CommandBool.Request()
        req.value = False
        res = self._call(self.arm_client, req)
        return bool(res and res.success)

    def stop(self) -> bool:
        self.get_logger().warn(f"[{self.name}] Emergency STOP (BRAKE)!")
        return self.set_mode("BRAKE")

    def takeoff(self, alt: float = 1.2, wait: bool = True, tol: float = 0.4) -> bool:
        """Set GUIDED mode, wait for arming, and take off to `alt` meters."""
        while rclpy.ok() and not self.set_mode("GUIDED"):
            self.get_logger().info(f"[{self.name}] Waiting to set GUIDED mode...")
            time.sleep(1.0)

        while rclpy.ok() and not self.arm():
            self.get_logger().info(f"[{self.name}] Waiting for EKF / pre-arm checks...")
            time.sleep(1.5)

        req = CommandTOL.Request()
        req.altitude = float(alt)
        res = self._call(self.takeoff_client, req)
        if not (res and res.success):
            return False

        self.get_logger().info(f"[{self.name}] Taking off to {alt}m...")
        if wait:
            t0 = time.time()
            while rclpy.ok() and (time.time() - t0 < 25.0):
                if abs(self.get_pos()[2] - alt) <= tol:
                    self.get_logger().info(f"[{self.name}] Takeoff complete.")
                    return True
                time.sleep(0.2)
        return True

    def land(self, wait: bool = True) -> bool:
        req = CommandTOL.Request()
        req.altitude = 0.0
        res = self._call(self.land_client, req)
        if not (res and res.success):
            return False

        self.get_logger().info(f"[{self.name}] Landing...")
        if wait:
            while rclpy.ok() and self.get_pos()[2] > 0.2:
                time.sleep(0.4)
            self.get_logger().info(f"[{self.name}] Landed.")
        return True

    def goto(self, x: float, y: float, z: float, yaw: float | None = None):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation = yaw_to_quat(yaw) if yaw is not None else deepcopy(self.pose_msg.pose.orientation)
        self.setpoint_pub.publish(msg)

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float | None = None,
        tol: float = 0.5,
        timeout: float = 30.0,
        safe_dist: float = 0.0,
    ) -> bool:
        self.get_logger().info(f"[{self.name}] Moving to ({x:.1f}, {y:.1f}, {z:.1f})...")
        t0 = time.time()
        while rclpy.ok():
            if safe_dist > 0.0 and self.min_dist(-60.0, 60.0) < safe_dist:
                self.get_logger().warn(f"[{self.name}] Obstacle ahead! Holding current position.")
                cx, cy, cz = self.get_pos()
                self.goto(cx, cy, cz)
                return False

            self.goto(x, y, z, yaw)
            cx, cy, cz = self.get_pos()
            if math.dist((cx, cy, cz), (x, y, z)) <= tol:
                self.get_logger().info(f"[{self.name}] Reached ({x:.1f}, {y:.1f}, {z:.1f}).")
                return True

            if time.time() - t0 > timeout:
                self.get_logger().warn(f"[{self.name}] Timeout moving to ({x:.1f}, {y:.1f}, {z:.1f}).")
                return False
            time.sleep(0.1)
        return False

    # =========================================================================
    # INTERNAL MAPPING HELPERS
    # =========================================================================
    def _to_grid(self, wx: float, wy: float) -> tuple[int, int] | None:
        gx = int((wx - self.origin_x) / self.map_res)
        gy = int((wy - self.origin_y) / self.map_res)
        if 0 <= gx < self.map_w and 0 <= gy < self.map_h:
            return gx, gy
        return None

    def update_map(self, scan: LaserScan, max_tilt: float = 12.0):
        dx, dy, dz = self.get_pos(world=True)
        r, p, yaw_deg = self.get_rpy(deg=True)
        if dz < 0.3 or abs(r) > max_tilt or abs(p) > max_tilt:
            return

        start = self._to_grid(dx, dy)
        if start is None:
            return
        yaw = math.radians(yaw_deg)

        for i in range(0, len(scan.ranges), 2):
            dist = scan.ranges[i]
            if not math.isfinite(dist) or dist < scan.range_min:
                continue

            hit = dist < (scan.range_max - 0.1)
            d = min(dist, scan.range_max)
            ang = scan.angle_min + i * scan.angle_increment + yaw
            end = self._to_grid(dx + d * math.cos(ang), dy + d * math.sin(ang))
            if end is None:
                continue

            cells = trace_ray(start[0], start[1], end[0], end[1])
            for cx, cy in (cells[:-1] if hit else cells):
                self.log_map[cy, cx] = max(-3.5, self.log_map[cy, cx] - 0.40)
            if hit:
                hx, hy = cells[-1]
                self.log_map[hy, hx] = min(3.5, self.log_map[hy, hx] + 0.85)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.map_res
        msg.info.width = self.map_w
        msg.info.height = self.map_h
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.get_map().flatten().tolist()
        self.map_pub.publish(msg)


class Swarm:
    """Background manager so mission scripts don't need any rclpy or executor boilerplate."""

    def __init__(self, names: list[str] = ["drone0"], spacing: float = 2.0, mapping: bool = True):
        if not rclpy.ok():
            rclpy.init()

        self.executor = MultiThreadedExecutor(num_threads=max(4, len(names) * 2))
        self.drones: dict[str, Drone] = {}

        for i, name in enumerate(names):
            d = Drone(name=name, offset=(0.0, i * spacing), mapping=mapping)
            self.drones[name] = d
            self.executor.add_node(d)

        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def __getitem__(self, name: str) -> Drone:
        return self.drones[name]

    def connect_all(self):
        for d in self.drones.values():
            d.connect()

    def takeoff_all(self, alt: float = 1.2):
        threads = [threading.Thread(target=d.takeoff, args=(alt, True)) for d in self.drones.values()]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

    def land_all(self):
        threads = [threading.Thread(target=d.land, args=(True,)) for d in self.drones.values()]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

    def close(self):
        self.executor.shutdown()
        for d in self.drones.values():
            d.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()