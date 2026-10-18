import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
import math
from copy import deepcopy


class Drone(Node):
    """Reusable controller class for a single ArduPilot + MAVROS drone."""
    def __init__(self, drone_id: str = "drone0"):
        super().__init__(f"{drone_id}_controller")
        self.drone_id = drone_id

        # 1. Service Clients (using /{drone_id}/mavros/... namespace)
        self.arming_client = self.create_client(CommandBool, f"/{drone_id}/mavros/cmd/arming")
        self.setmode_client = self.create_client(SetMode, f"/{drone_id}/mavros/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, f"/{drone_id}/mavros/cmd/takeoff")
        self.landing_client = self.create_client(CommandTOL, f"/{drone_id}/mavros/cmd/land")

        # 2. Pub/Sub for Local Position (ENU frame relative to drone's own EKF origin)
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.SYSTEM_DEFAULT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            f"/{drone_id}/mavros/local_position/pose",
            self._pos_callback,
            qos_profile=self.qos_profile
        )
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            f"/{drone_id}/mavros/setpoint_position/local",
            qos_profile=self.qos_profile
        )

        self.current_pos = Point()

    def _pos_callback(self, msg: PoseStamped):
        self.current_pos = deepcopy(msg.pose.position)

    def wait_for_services(self):
        services = [self.arming_client, self.setmode_client, self.takeoff_client, self.landing_client]
        for srv in services:
            while not srv.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"[{self.drone_id}] Waiting for {srv.srv_name}...")
                rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"[{self.drone_id}] All MAVROS services ready.")

    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode.upper()
        future = self.setmode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return bool(future.result() and future.result().mode_sent)

    def arm(self) -> bool:
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return bool(future.result() and future.result().success)

    def takeoff(self, altitude: float) -> bool:
        req = CommandTOL.Request()
        req.altitude = float(altitude)
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return bool(future.result() and future.result().success)

    def land(self) -> bool:
        req = CommandTOL.Request()
        req.altitude = 0.0
        future = self.landing_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return bool(future.result() and future.result().success)

    def goto_local(self, x: float, y: float, z: float):
        """Publish a single local ENU setpoint (relative to this drone's spawn/home)."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        self.setpoint_pub.publish(msg)

    def distance_to(self, x: float, y: float, z: float) -> float:
        """Euclidean distance from current local position to target (x, y, z)."""
        return math.sqrt(
            (self.current_pos.x - x) ** 2 +
            (self.current_pos.y - y) ** 2 +
            (self.current_pos.z - z) ** 2
        )


def main(args=None):
    rclpy.init(args=args)
    drone = Drone("drone0")

    try:
        # --- A. CONNECT & ARM ---
        drone.wait_for_services()
        drone.set_mode("GUIDED")
        while not drone.arm():
            drone.get_logger().info("Waiting for EKF pre-arm checks...")
            time.sleep(2.0)

        # --- B. TAKE OFF ---
        target_alt = 5.0
        drone.takeoff(target_alt)
        while rclpy.ok():
            rclpy.spin_once(drone, timeout_sec=0.1)
            if abs(drone.current_pos.z - target_alt) <= 0.5:
                break
            time.sleep(0.2)

        # --- C. MOVE TO A COORDINATE (X=10, Y=5, Z=5) ---
        target_x, target_y, target_z = 10.0, 5.0, 5.0
        while rclpy.ok():
            rclpy.spin_once(drone, timeout_sec=0.1)
            drone.goto_local(target_x, target_y, target_z)
            if drone.distance_to(target_x, target_y, target_z) <= 0.5:
                break
            time.sleep(0.1)

        # --- D. LAND ---
        drone.land()
        while rclpy.ok():
            rclpy.spin_once(drone, timeout_sec=0.1)
            if drone.current_pos.z <= 0.2:
                break
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        drone.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()