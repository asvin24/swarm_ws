import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, CommandHome
from geometry_msgs.msg import PoseStamped, Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
import math
from copy import deepcopy


class Drone(Node):
    """Controller class for a single drone"""
    def __init__(self, drone_id="drone0"):
        super().__init__(f"{drone_id}_controller")
        self.drone_id = drone_id
        
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
            self.get_logger().info(f"[{self.drone_id}]   checking {name} -> {service.srv_name}")
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"[{self.drone_id}]   ...still waiting on {name}")
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info(f"[{self.drone_id}]   {name} OK")
        self.get_logger().info(f"[{self.drone_id}] All services active.")

    def setmode(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode.upper()
        future = self.setmode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f"[{self.drone_id}] Mode set to {mode}")
            return True
        return False

    def arming(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f"[{self.drone_id}] Armed successfully")
            return True
        return False

    def takeoff(self, altitude: float):
        req = CommandTOL.Request()
        req.altitude = altitude
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f"[{self.drone_id}] Taking off to {altitude}m")
            return True
        return False

    def land(self):
        req = CommandTOL.Request()
        req.altitude = 0.0
        future = self.landing_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f"[{self.drone_id}] Landing...")
            return True
        return False

    def goto_local(self, x: float, y: float, z: float):
        """Send a simple local position setpoint"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        self.new_local_position_publisher.publish(msg)

    def wait_for_position(self, target_x: float, target_y: float, target_z: float, tolerance: float = 0.5, timeout: float = 30.0):
        """Spin and republish setpoint until the drone is within `tolerance` meters of the target."""
        self.get_logger().info(f"[{self.drone_id}] Moving to (X: {target_x}, Y: {target_y}, Z: {target_z})...")
        start_time = time.time()

        while rclpy.ok():
            # Process incoming pose callbacks
            rclpy.spin_once(self, timeout_sec=0.1)

            # Republish setpoint at ~5 Hz to ensure MAVROS/ArduPilot doesn't drop it
            self.goto_local(target_x, target_y, target_z)

            dx = self.current_position_local.x - target_x
            dy = self.current_position_local.y - target_y
            dz = self.current_position_local.z - target_z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            if dist <= tolerance:
                self.get_logger().info(f"[{self.drone_id}] Reached waypoint (X: {target_x}, Y: {target_y}, Z: {target_z})")
                return True

            if time.time() - start_time > timeout:
                self.get_logger().warn(f"[{self.drone_id}] Timeout waiting for waypoint ({target_x}, {target_y}, {target_z}), continuing...")
                return False

            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    # 1. Instantiate the single drone (drone0)
    drone = Drone("drone0")

    target_alt = 5.0
    side_length = 10.0  # 10m x 10m square

    # Define the 4 corners of the square in local ENU coordinates
    square_waypoints = [
        (side_length, 0.0, target_alt),          # Corner 1: Forward 10m
        (side_length, side_length, target_alt),  # Corner 2: Left 10m
        (0.0, side_length, target_alt),          # Corner 3: Back 10m
        (0.0, 0.0, target_alt),                  # Corner 4: Return to start (0, 0)
    ]

    try:
        # 2. Wait for MAVROS services
        drone.wait_for_services()

        # 3. Set mode to GUIDED, Arm, and Take off
        drone.setmode('GUIDED')
        time.sleep(1.0)

        # Retry arming if ArduPilot EKF is still initializing
        while not drone.arming():
            drone.get_logger().info(f"[{drone.drone_id}] Waiting for EKF/pre-arm checks to pass...")
            time.sleep(2.0)

        drone.takeoff(target_alt)

        # Wait until takeoff altitude is reached (~90% of target_alt)
        drone.get_logger().info(f"[{drone.drone_id}] Climbing to {target_alt}m...")
        start_t = time.time()
        while rclpy.ok() and (time.time() - start_t < 20.0):
            rclpy.spin_once(drone, timeout_sec=0.1)
            if abs(drone.current_position_local.z - target_alt) <= 0.5:
                drone.get_logger().info(f"[{drone.drone_id}] Takeoff altitude reached.")
                break
            time.sleep(0.2)

        # 4. Fly in a square
        for wp_x, wp_y, wp_z in square_waypoints:
            drone.wait_for_position(wp_x, wp_y, wp_z, tolerance=0.5, timeout=25.0)
            time.sleep(1.0)  # Brief pause at each corner

        # 5. Land
        drone.land()

        # Wait until landed on the ground
        while rclpy.ok():
            rclpy.spin_once(drone, timeout_sec=0.1)
            if drone.current_position_local.z <= 0.2:
                drone.get_logger().info(f"[{drone.drone_id}] Touchdown confirmed.")
                break
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        drone.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()