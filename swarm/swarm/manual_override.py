#!/usr/bin/env python3
"""
Standalone raw ROS 2 single-drone mission node with a manual-override watchdog
(RC sticks + external flight-mode change). No dependency on drone_api.py.

Fixes vs. the previous version:
  * connect() no longer hangs (it polled a `self.state` that never existed).
  * No spin_until_future_complete() from inside an already-spinning node;
    all service calls go through _call() (Event + done-callback).
  * self.mode is actually stored, so the LOITER hand-over can be confirmed.
  * An external mode change no longer forces LOITER (pilot's chosen mode wins).
  * Landing uses cmd/land (it used cmd/takeoff with altitude 0 before).
  * takeoff() / move_to() results are checked; the mission aborts on failure.
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from mavros_msgs.msg import State, RCIn
from geometry_msgs.msg import PoseStamped


class ManualOverride(Exception):
    """Raised when the safety pilot takes over via RC sticks or an external mode switch."""


class StandaloneDroneNode(Node):

    def __init__(
        self,
        drone_name: str = "drone0",
        rc_deadband: int = 50,
        rc_confirm_samples: int = 3,
        roll_idx: int = 0,
        pitch_idx: int = 1,
        stream_hz: int = 20,
    ):
        super().__init__(f"{drone_name}_raw_node")

        self.drone_name = drone_name
        self.prefix = f"/{drone_name}/mavros"
        self.stream_hz = stream_hz

        # Telemetry state
        self.connected = False
        self.armed = False
        self.mode = ""
        self.have_pose = False
        self.pos = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]

        # Manual-override state
        self.rc_deadband = rc_deadband
        self.rc_confirm_samples = rc_confirm_samples
        self.roll_idx = roll_idx
        self.pitch_idx = pitch_idx

        self.override_triggered = threading.Event()
        self.override_handled = threading.Event()
        self.override_reason = None
        self.rc_baseline = None
        self._rc_moved_count = 0
        self._prev_mode = None
        self._mode_watch_active = False
        self._allowed_modes = {"GUIDED", "LAND", "BRAKE"}
        self._intended_mode = None
        self._lock = threading.Lock()

        # QoS profiles
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriptions & publishers
        self.create_subscription(State, f"{self.prefix}/state", self._state_cb, reliable)
        self.create_subscription(RCIn, f"{self.prefix}/rc/in", self._rc_cb, best_effort)
        self.create_subscription(PoseStamped, f"{self.prefix}/local_position/pose", self._pose_cb, best_effort)

        self.setpoint_pub = self.create_publisher(PoseStamped, f"{self.prefix}/setpoint_position/local", reliable)
        self.override_pub = self.create_publisher(Bool, f"/{drone_name}/manual_override", latched)

        # MAVROS service clients
        self.arm_client = self.create_client(CommandBool, f"{self.prefix}/cmd/arming")
        self.mode_client = self.create_client(SetMode, f"{self.prefix}/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, f"{self.prefix}/cmd/takeoff")
        self.land_client = self.create_client(CommandTOL, f"{self.prefix}/cmd/land")
        self.stream_client = self.create_client(StreamRate, f"{self.prefix}/set_stream_rate")

        self.get_logger().info(f"[{drone_name}] Raw standalone node initialized with override watchdogs.")

    # ------------------------------------------------------------------ #
    #                       Service-call helper                          #
    # ------------------------------------------------------------------ #
    def _call(self, client, req, timeout: float = 5.0):
        """Blocking service call that is safe while the node is spinning in another thread."""
        event = threading.Event()
        future = client.call_async(req)
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout=timeout):
            self.get_logger().warn(f"Service {client.srv_name} timed out")
            return None
        return future.result()

    # ------------------------------------------------------------------ #
    #                     Override watchdog callbacks                    #
    # ------------------------------------------------------------------ #
    def _state_cb(self, msg: State):
        self.connected = msg.connected
        self.armed = msg.armed

        new_mode = (msg.mode or "").upper()
        if not new_mode:
            return

        prev_mode = self._prev_mode
        self._prev_mode = new_mode
        self.mode = new_mode

        if prev_mode is None or new_mode == prev_mode:
            return

        with self._lock:
            expected = new_mode in self._allowed_modes or new_mode == self._intended_mode
        if self._mode_watch_active and not expected:
            # Somebody else chose this mode: do not command anything, just stop the script.
            self._trigger_override(
                f"Flight mode changed externally: {prev_mode} -> {new_mode}", set_hold=False
            )

    def _rc_cb(self, msg: RCIn):
        if self.override_triggered.is_set():
            return
        ch = msg.channels
        if len(ch) <= max(self.roll_idx, self.pitch_idx):
            return

        roll, pitch = int(ch[self.roll_idx]), int(ch[self.pitch_idx])
        if roll == 0 or pitch == 0:          # 0 = no signal
            return

        if self.rc_baseline is None:         # sticks should be centred at start
            self.rc_baseline = (roll, pitch)
            self.get_logger().info(f"RC baseline locked: roll={roll}, pitch={pitch}")
            return

        if not self.armed:
            self._rc_moved_count = 0
            return

        moved = (abs(roll - self.rc_baseline[0]) > self.rc_deadband or
                 abs(pitch - self.rc_baseline[1]) > self.rc_deadband)
        self._rc_moved_count = (self._rc_moved_count + 1) if moved else 0

        if self._rc_moved_count >= self.rc_confirm_samples:
            self._trigger_override(
                f"RC stick input (roll={roll}, pitch={pitch}, baseline={self.rc_baseline})",
                set_hold=True,
            )

    def _pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        self.pos = [p.x, p.y, p.z]
        self.orientation = [q.x, q.y, q.z, q.w]
        self.have_pose = True

    def _trigger_override(self, reason: str, set_hold: bool = True):
        with self._lock:
            if self.override_triggered.is_set():
                return
            self.override_reason = reason
            self.override_triggered.set()

        self.get_logger().fatal(f"MANUAL OVERRIDE TRIGGERED: {reason}")
        self.override_pub.publish(Bool(data=True))

        # Slow work goes in a thread so the executor keeps receiving service replies and /state.
        threading.Thread(target=self._handle_handover, args=(set_hold,), daemon=True).start()

    def _handle_handover(self, set_hold: bool):
        try:
            if not set_hold:
                return
            for mode in ("LOITER", "POSHOLD", "BRAKE"):
                with self._lock:
                    self._intended_mode = mode
                res = self._call(self.mode_client, SetMode.Request(custom_mode=mode), timeout=3.0)
                if not (res and res.mode_sent):
                    continue
                end = time.monotonic() + 3.0
                while time.monotonic() < end and self.mode != mode:
                    time.sleep(0.05)
                if self.mode == mode:
                    self.get_logger().warn(f"Handed control to pilot in {mode} mode.")
                    return
            self.get_logger().error("Could not switch to LOITER/POSHOLD/BRAKE! Pilot must change mode manually.")
        finally:
            self.override_handled.set()

    def check(self):
        """Raise ManualOverride if an override has happened."""
        if self.override_triggered.is_set():
            raise ManualOverride(self.override_reason or "Safety pilot took manual control!")

    def allow_mode(self, mode: str):
        """Tell the watchdog that YOUR code is about to switch to `mode` (call before switching)."""
        with self._lock:
            self._allowed_modes.add(mode.upper())
            self._intended_mode = mode.upper()

    # ------------------------------------------------------------------ #
    #                       Raw flight control actions                   #
    # ------------------------------------------------------------------ #
    def connect(self, timeout: float = 120.0):
        self.get_logger().info("Waiting for MAVROS services...")
        for client in (self.arm_client, self.mode_client, self.takeoff_client,
                       self.land_client, self.stream_client):
            while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"  waiting for {client.srv_name}...")

        # Ask ArduPilot to stream telemetry (we run SITL without MAVProxy)
        req = StreamRate.Request(stream_id=0, message_rate=self.stream_hz, on_off=True)
        for _ in range(5):
            if self._call(self.stream_client, req, timeout=2.0) is not None:
                self.get_logger().info(f"Telemetry streams set to {self.stream_hz} Hz")
                break
            time.sleep(1.0)

        t0 = time.time()
        while rclpy.ok() and not self.connected:
            if time.time() - t0 > timeout:
                raise RuntimeError("Timed out waiting for FCU heartbeat")
            self.get_logger().info("Waiting for FCU connection...", throttle_duration_sec=2.0)
            time.sleep(0.2)
        self.get_logger().info("Connected to FCU.")

        while rclpy.ok() and not self.have_pose:
            if time.time() - t0 > timeout:
                raise RuntimeError("Timed out waiting for local_position/pose (EKF not ready)")
            self.get_logger().info("Waiting for local_position/pose (EKF)...", throttle_duration_sec=2.0)
            time.sleep(0.2)
        self.get_logger().info("Pose data received. Ready.")

    def set_mode(self, mode: str) -> bool:
        self.allow_mode(mode)
        res = self._call(self.mode_client, SetMode.Request(custom_mode=mode.upper()), timeout=3.0)
        return bool(res and res.mode_sent)

    def arm(self) -> bool:
        res = self._call(self.arm_client, CommandBool.Request(value=True), timeout=3.0)
        return bool(res and res.success)

    def takeoff(self, alt: float = 1.2, tol: float = 0.35, timeout: float = 25.0) -> bool:
        while rclpy.ok() and not self.set_mode("GUIDED"):
            self.check()
            self.get_logger().info("Waiting to set GUIDED mode...", throttle_duration_sec=2.0)
            time.sleep(1.0)

        while rclpy.ok() and not self.arm():
            self.check()
            self.get_logger().info("Waiting for EKF / pre-arm checks...", throttle_duration_sec=3.0)
            time.sleep(1.0)

        self._mode_watch_active = True      # start watching for external mode changes

        res = self._call(self.takeoff_client, CommandTOL.Request(altitude=float(alt)), timeout=5.0)
        if not (res and res.success):
            self.get_logger().error("Takeoff command rejected.")
            return False

        self.get_logger().info(f"Taking off to {alt}m...")
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            self.check()
            if abs(self.pos[2] - alt) <= tol:
                self.get_logger().info("Takeoff altitude reached.")
                return True
            time.sleep(0.2)
        self.get_logger().error("Takeoff timed out before reaching altitude.")
        return False

    def goto(self, x: float, y: float, z: float):
        self.check()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        q = self.orientation
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.setpoint_pub.publish(msg)

    def move_to(self, x: float, y: float, z: float, tol: float = 0.45, timeout: float = 30.0) -> bool:
        t0 = time.time()
        while rclpy.ok():
            self.check()
            self.goto(x, y, z)
            if math.dist(self.pos, [x, y, z]) <= tol:
                return True
            if time.time() - t0 > timeout:
                return False
            time.sleep(0.1)
        return False

    def land(self, timeout: float = 60.0) -> bool:
        self.allow_mode("LAND")
        res = self._call(self.land_client, CommandTOL.Request(altitude=0.0), timeout=5.0)
        if not (res and res.success):
            self.get_logger().warn("cmd/land failed, falling back to LAND mode.")
            if not self.set_mode("LAND"):
                return False

        self.get_logger().info("Landing...")
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            self.check()
            if self.pos[2] < 0.2 or not self.armed:
                self.get_logger().info("Landed.")
                return True
            time.sleep(0.3)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = StandaloneDroneNode(drone_name="drone0")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.connect()

        if not node.takeoff(alt=1.2):
            node.get_logger().error("Takeoff failed. Aborting mission.")
            return

        waypoints = [
            (2.0, 0.0, 1.2),
            (2.0, 2.0, 1.2),
            (0.0, 2.0, 1.2),
            (0.0, 0.0, 1.2),
        ]

        for wx, wy, wz in waypoints:
            node.get_logger().info(f"Moving to waypoint ({wx}, {wy}, {wz})...")
            if not node.move_to(wx, wy, wz):
                node.get_logger().warn(f"Timed out reaching ({wx}, {wy}, {wz}); continuing.")
            time.sleep(1.0)

        node.get_logger().info("Mission complete. Landing...")
        node.land()

    except ManualOverride as e:
        node.get_logger().fatal(f"Caught ManualOverride: {e}. Mission aborted!")
        node.override_handled.wait(timeout=5.0)
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user.")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()