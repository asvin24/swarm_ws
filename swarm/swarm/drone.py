import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, CommandHome
from geometry_msgs.msg import PoseStamped, Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
import math
from copy import deepcopy


class Drone(Node):
    """Base class for any drone instance in the swarm."""
    def __init__(self, drone_id: str, offset_x: float = 0.0, offset_y: float = 0.0):
        super().__init__(f"{drone_id}_controller")
        self.drone_id = drone_id
        # Optional extra offset relative to this drone's own spawn point (0, 0)
        self.offset_x = float(offset_x)
        self.offset_y = float(offset_y)

        # Initialize clients with the drone's specific namespace
        self.arming_client = self.create_client(CommandBool, f"/{drone_id}/mavros/cmd/arming")
        self.setmode_client = self.create_client(SetMode, f"/{drone_id}/mavros/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, f"/{drone_id}/mavros/cmd/takeoff")
        self.landing_client = self.create_client(CommandTOL, f"/{drone_id}/mavros/cmd/land")
        self.set_home_client = self.create_client(CommandHome, f"/{drone_id}/mavros/cmd/set_home")

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.SYSTEM_DEFAULT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.local_position_subscriber = self.create_subscription(
            PoseStamped,
            f"/{drone_id}/mavros/local_position/pose",
            self.set_current_pos,
            qos_profile=self.qos_profile
        )

        self.new_local_position_publisher = self.create_publisher(
            PoseStamped,
            f"/{drone_id}/mavros/setpoint_position/local",
            qos_profile=self.qos_profile
        )

        self.current_position_local = Point()

    def set_current_pos(self, msg: PoseStamped):
        self.current_position_local = deepcopy(msg.pose.position)

    def wait_for_services(self):
        self.get_logger().info(f"[{self.drone_id}] Waiting for MAVROS services...")
        services = {
            'arming': self.arming_client,
            'set_mode': self.setmode_client,
            'takeoff': self.takeoff_client,
            'land': self.landing_client,
        }
        for name, service in services.items():
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"[{self.drone_id}]   ...still waiting on {name}")
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info(f"[{self.drone_id}]   {name} OK")
        self.get_logger().info(f"[{self.drone_id}] All services active.")

    def setmode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode.upper()
        future = self.setmode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f"[{self.drone_id}] Mode set to {mode}")
            return True
        return False

    def arming(self) -> bool:
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f"[{self.drone_id}] Armed successfully")
            return True
        return False

    def takeoff(self, altitude: float) -> bool:
        req = CommandTOL.Request()
        req.altitude = float(altitude)
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f"[{self.drone_id}] Taking off to {altitude}m")
            return True
        return False

    def land(self) -> bool:
        req = CommandTOL.Request()
        req.altitude = 0.0
        future = self.landing_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f"[{self.drone_id}] Landing...")
            return True
        return False

    def goto_local(self, x: float, y: float, z: float):
        """Publish local setpoint shifted by this drone's relative offset."""
        target_x = float(x) + self.offset_x
        target_y = float(y) + self.offset_y
        target_z = float(z)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = target_x
        msg.pose.position.y = target_y
        msg.pose.position.z = target_z
        self.new_local_position_publisher.publish(msg)

    def has_reached(self, x: float, y: float, z: float, tolerance: float = 0.5) -> bool:
        """Check if the drone is within `tolerance` meters of (x + offset_x, y + offset_y, z)."""
        target_x = float(x) + self.offset_x
        target_y = float(y) + self.offset_y
        target_z = float(z)

        dx = self.current_position_local.x - target_x
        dy = self.current_position_local.y - target_y
        dz = self.current_position_local.z - target_z
        return math.sqrt(dx * dx + dy * dy + dz * dz) <= tolerance


def move_swarm_to_waypoint(drones: list[Drone], wp_x: float, wp_y: float, wp_z: float, tolerance: float = 0.5, timeout: float = 30.0):
    """Command both drones to a waypoint simultaneously and wait until all reach it."""
    for d in drones:
        d.get_logger().info(
            f"[{d.drone_id}] Heading to local ({wp_x + d.offset_x:.1f}, {wp_y + d.offset_y:.1f}, {wp_z:.1f})"
        )

    start_time = time.time()
    while rclpy.ok():
        all_reached = True
        for d in drones:
            rclpy.spin_once(d, timeout_sec=0.05)
            d.goto_local(wp_x, wp_y, wp_z)
            if not d.has_reached(wp_x, wp_y, wp_z, tolerance):
                all_reached = False

        if all_reached:
            for d in drones:
                d.get_logger().info(f"[{d.drone_id}] Reached waypoint!")
            return True

        if time.time() - start_time > timeout:
            for d in drones:
                d.get_logger().warn(f"[{d.drone_id}] Waypoint timeout reached, moving to next step...")
            return False

        time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)

    # 1. Instantiate the two drones.
    # Because each drone's local (0,0) is already its own spawn point,
    # offset_x=0.0, offset_y=0.0 makes each fly a square starting from its spawn point.
    # Change offset_x / offset_y if you want an extra shift relative to their spawn point!
    drone_0 = Drone("drone0", offset_x=0.0, offset_y=0.0)
    drone_1 = Drone("drone1", offset_x=0.0, offset_y=0.0)
    drones = [drone_0, drone_1]

    target_alt = 5.0
    side_length = 10.0

    # Square trajectory relative to each drone's starting point + offset
    square_waypoints = [
        (side_length, 0.0, target_alt),          # Corner 1
        (side_length, side_length, target_alt),  # Corner 2
        (0.0, side_length, target_alt),          # Corner 3
        (0.0, 0.0, target_alt),                  # Corner 4 (back above spawn/offset)
    ]

    try:
        # 2. Wait for MAVROS services on both drones
        for d in drones:
            d.wait_for_services()

        # 3. Set mode to GUIDED on all drones
        for d in drones:
            while not d.setmode('GUIDED'):
                d.get_logger().info(f"[{d.drone_id}] Waiting to set GUIDED mode...")
                time.sleep(1.0)

        # 4. Arm and Immediately Take Off each drone once EKF is ready
        # Wait until all drones are armed (if one disarms while waiting for the other, re-arm it)
        while rclpy.ok():
            all_armed = True
            for d in drones:
                rclpy.spin_once(d, timeout_sec=0.1)
                if not d.arming():
                    d.get_logger().info(f"[{d.drone_id}] Waiting for EKF/pre-arm checks to pass...")
                    all_armed = False
            if all_armed:
                break
            time.sleep(2.0)

        # Immediately trigger takeoff on all drones so none auto-disarm on the ground
        for d in drones:
            d.takeoff(target_alt)

        # Wait until both drones reach target altitude
        start_t = time.time()
        while rclpy.ok() and (time.time() - start_t < 25.0):
            for d in drones:
                rclpy.spin_once(d, timeout_sec=0.05)
            if all(abs(d.current_position_local.z - target_alt) <= 0.5 for d in drones):
                print("Both drones reached takeoff altitude!")
                break
            time.sleep(0.1)

        # 5. Fly both drones around the square in formation
        for wp_x, wp_y, wp_z in square_waypoints:
            move_swarm_to_waypoint(drones, wp_x, wp_y, wp_z, tolerance=0.5, timeout=30.0)
            time.sleep(1.0)

        # 6. Land both drones simultaneously
        for d in drones:
            d.land()

        while rclpy.ok():
            for d in drones:
                rclpy.spin_once(d, timeout_sec=0.05)
            if all(d.current_position_local.z <= 0.2 for d in drones):
                print("Both drones landed safely!")
                break
            time.sleep(0.3)

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        for d in drones:
            d.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()