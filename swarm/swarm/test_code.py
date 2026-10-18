import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, CommandHome
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
import traceback
import cv2
import math
import numpy as np
from copy import deepcopy
import time

import rclpy.executors
from rclpy.executors import MultiThreadedExecutor
import rclpy.callback_groups

class AutopilotNode(Node):
    "Autopilot Control Node"

    def __init__(self):
        super().__init__("Autopilot")

        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.setmode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.landing_client = self.create_client(CommandTOL, "/mavros/cmd/land")
        self.set_home_client = self.create_client(CommandHome,  "/mavros/cmd/set_home")

        

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.SYSTEM_DEFAULT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.control_messages_callback_grp = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()


        self.local_position_subscriber = self.create_subscription(PoseStamped, "/mavros/local_position/pose", qos_profile=self.qos_profile, callback_group=self.control_messages_callback_grp, callback=self.set_current_pos)
        self.new_local_position_publisher = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", qos_profile=self.qos_profile) # publishers don't have callbacks

        # self.home_position_subscriber = self.create_subscription(NavSatFix, "/mavros/global_position/global", qos_profile=self.qos_profile, callback_group=self.control_messages_callback_grp, callback=self.set_home_pos)  # no need as this is given by competition people

        self.current_position_local : Point = None
        self.start_position : NavSatFix = None
        self.old_position : Point = None
        self.current_mode : str = None
        self.armed : bool = None

    
    def set_current_pos(self, msg: PoseStamped) -> None:

        position = msg.pose.position

        self.get_logger().info(
                f"POSITION CALLBACK: "
                f"x={position.x:.2f}, "
                f"y={position.y:.2f}, "
                f"z={position.z:.2f}"
            )

        if position != self.current_position_local:
            self.old_position = deepcopy(self.current_position_local)
            self.current_position_local = deepcopy(position)

    def set_home_position(self, home_pos : NavSatFix) -> bool:
        self.get_logger().info("Set home location requested...")
        req = CommandHome.Request()
    
        req.current_gps = False  #  home position given explicitly by the organisation hosting competition
        req.altitude = home_pos.altitude
        req.longitude = home_pos.longitude
        req.latitude = home_pos.latitude

        future = self.set_home_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)  # Setting home position is important hence the node should pause until the home position is set

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Home Position set successfully')
                return True
            else:
                self.get_logger().warn('Failed to set home position')
                return False
        else:
            self.get_logger().error('Homing service call failed')
            return False


    def wait_for_services(self) -> None:
        "Wait for all services to be available"
        services = [self.arming_client, self.set_home_client, self.landing_client, self.takeoff_client, self.setmode_client]

        for service in services:
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {service} service...')
        
        self.get_logger().info('All services active.')

    def arming(self) -> bool:
        "Arm the drone"

        req = CommandBool.Request()
        req.value = True

        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Vehicle armed successfully')
                return True
            else:
                self.get_logger().warn('Failed to arm vehicle')
                return False
        else:
            self.get_logger().error('Arming service call failed')
            return False

    def disarming(self) -> bool:
        "Disarm the drone"

        req = CommandBool.Request()
        req.value = False

        future = self.arming_client.call_async(req) # async call
        rclpy.spin_until_future_complete(self, future) # wait until response arrives

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Vehicle disarmed successfully')
                return True
            else:
                self.get_logger().warn('Failed to disarm vehicle')
                return False
        else:
            self.get_logger().error('Disarming service call failed')
            return False

    def takeoff(self, altitude: float) -> bool:
        """Takeoff the drone to a certain altitude.
        
            :param altitude: in meters"""

        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0  # Use current position
        req.longitude = 0.0
        req.altitude = altitude

        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Takeoff command sent. Target altitude: {altitude}m')

                # Wait for 10 sec to takeoff
                for _ in range(100):
                    rclpy.spin_once(self, timeout_sec=0.1)

                return True
            else:
                self.get_logger().warn('Takeoff command failed')
                return False
        else:
            self.get_logger().error('Takeoff service call failed')
            return False

    def setmode(self, mode: str) -> bool:
        """
        Set flight mode.

        Common modes: STABILIZED, GUIDED, RTL, LAND
        """
        mode = mode.upper() # convert all to caps
        req = SetMode.Request()
        req.custom_mode = mode

        future = self.setmode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info(f'Mode set to {mode}')
                return True
            else:
                self.get_logger().warn(f'Failed to set mode to {mode}')
                return False
        else:
            self.get_logger().error('Set mode service call failed')
            return False

    def land(self) -> bool:
        """Takeoff the drone to a certain altitude.
        
            :param altitude: in meters"""

        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0  # Use current position
        req.longitude = 0.0
        req.altitude = 0.0

        future = self.landing_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Land command sent.')
                return True
            else:
                self.get_logger().warn('Land command failed')
                return False
        else:
            self.get_logger().error('Land service call failed')
            return False

    def move_forward_x(self ) -> PoseStamped:
        delta_x = 1.0

        self.get_logger().info(
            f"CURRENT: x={self.current_position_local.x:.2f}, "
            f"y={self.current_position_local.y:.2f}, "
            f"z={self.current_position_local.z:.2f}"
        )
        new_position = PoseStamped()
        
        new_position.pose.position.x = self.current_position_local.x + delta_x
        new_position.pose.position.y = self.current_position_local.y
        new_position.pose.position.z = self.current_position_local.z

        self.get_logger().info(
            f"TARGET: x={new_position.pose.position.x:.2f}, "
            f"y={new_position.pose.position.y:.2f}, "
            f"z={new_position.pose.position.z:.2f}"
        )

        return new_position

        def move_forward_y(self) -> PoseStamped:
            delta_y = 1.0

            self.get_logger().info(
                f"CURRENT: x={self.current_position_local.x:.2f}, "
                f"y={self.current_position_local.y:.2f}, "
                f"z={self.current_position_local.z:.2f}"
            )
            new_position = PoseStamped()
            
            new_position.pose.position.x = self.current_position_local.x 
            new_position.pose.position.y = self.current_position_local.y + delta_y
            new_position.pose.position.z = self.current_position_local.z

            self.get_logger().info(
                f"TARGET: x={new_position.pose.position.x:.2f}, "
                f"y={new_position.pose.position.y:.2f}, "
                f"z={new_position.pose.position.z:.2f}"
            )

            return new_position

    def move_10_fwd(self):

        tolerance = 0.1       # 10 cm
        timeout = 5.0        # maximum wait for each 1 m movement

        for i in range(10):
            # Generate target = current position + 1 m
            new_position = self.move_forward_x()
            target_x = new_position.pose.position.x

            # Publish target
            self.new_local_position_publisher.publish(new_position)
            self.get_logger().info(
                f"Step {i+1}/10: Target x = {target_x:.2f}"
            )

            # Wait until drone reaches target
            elapsed = 0.0
            while rclpy.ok() and elapsed < timeout:
                # Allow local_position callback to update current_position_local
                rclpy.spin_once(self, timeout_sec=0.1)
                current_x = self.current_position_local.x
                self.get_logger().info(
                    f"Current x = {current_x:.2f}, "
                    f"Target x = {target_x:.2f}"
                )
                # Check whether target has been reached
                if abs(current_x - target_x) < tolerance:
                    self.get_logger().info(
                        f"Reached target x = {target_x:.2f}"
                    )
                    break
                elapsed += 0.1
            else:
                self.get_logger().warn(
                    f"Timeout waiting for target x = {target_x:.2f}"
                )



def main(args=None):
    rclpy.init(args=args)

    try:
        # Create TaskControl instance
        task_control = AutopilotNode()

        task_control.get_logger().info('=== Image Centering Flight Sequence ===')
        import time

        time.sleep(2)

        # Change at time of competition
        home_position = NavSatFix()
        home_position.altitude = 237.9
        home_position.longitude = 10.3240712
        home_position.latitude = 60.2157199

        # Set home position
        task_control.get_logger().info('Setting given position as home position...')
        if not task_control.set_home_position(home_position):
            task_control.get_logger().warn('Failed to set home position, continuing anyway...')
        time.sleep(1)

        # Set to GUIDED mode
        task_control.get_logger().info('Setting mode to GUIDED...')
        if not task_control.setmode('GUIDED'):
            task_control.get_logger().error('Failed to set GUIDED mode. Exiting...')
            return
        time.sleep(2)

        # Arm the vehicle
        task_control.get_logger().info('Arming the vehicle...')
        if not task_control.arming():
            task_control.get_logger().error('Failed to arm vehicle. Exiting...')
            return
        time.sleep(2)

        # Takeoff to 5 meters
        task_control.get_logger().info('Taking off to 5 meters...')
        if not task_control.takeoff(5.0):
            task_control.get_logger().error('Failed to send takeoff command. Landing...')
            task_control.land()
            return
        task_control.get_logger().info('Drone is taking off...')
        # time.sleep(10)  # Wait for takeoff to complete

        

        # Move drone 10m in local x
        task_control.move_10_fwd()

        # Land
        task_control.get_logger().info('Land')
        while not task_control.land():
            task_control.get_logger().error('Failed to send land command. Retrying...')
            time.sleep(2)
            # task_control.land()
            

    except KeyboardInterrupt:
        task_control.get_logger().info('Flight interrupted by user')
    except Exception as e:
        task_control.get_logger().error(f"An error occurred: {type(e).__name__}: {repr(e)}")
        traceback.print_exc()
    finally:
        task_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    


