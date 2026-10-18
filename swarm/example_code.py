import sys
import time
import argparse

import rclpy
import rclpy.callback_groups
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.node import Node
import rclpy.executors
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoint, GeoPoseStamped
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.msg import StatusText

from mavros_msgs.srv import CommandTOL, CommandBool, CommandHome, SetMode

from .autopilot_configurations import Mission
from artemis_waypoints.frame_conversions import field_frame_to_enu_frame, geoid_height_to_amsl, get_quaternion, quaternion_to_yaw, rotation_to_quaternion
import cv2
import numpy as np
import imutils
import pytz
import datetime
import json
import math


class Autopilot(Node):
    """Autopilot node for Artemis"""
    
    def __init__(self, mission : Mission, executor : rclpy.executors.MultiThreadedExecutor) -> None:
        node_name = f"autopilot_{mission.mission_name}"
        super().__init__(node_name)
        
        # Mission parameters
        self.waypoints = mission.waypoints
        self.field_rotation = mission.field_rotation
        self.quaternion = mission.quaternion
        self.object_drop_rc_channel = mission.object_drop_rc_channel
        self.object_drop_rc_channel_value = mission.object_drop_rc_channel_value
        self.object_drop_publish_time_secs = mission.object_drop_publish_time_secs
        self.mission_start_point_latitude = mission.mission_start_point_latitude
        self.mission_start_point_longitude = mission.mission_start_point_longitude
        self.mission_altitude_offset = mission.mission_altitude_offset
        self.mission_waypoint_tolerance = mission.mission_waypoint_tolerance
        self.camera_width_along_x_axis = mission.camera_width_along_x_axis

        self.Break_coordinate = Point()
        self.detected_center = []
        self.Follow_target = False
        self.vehicle_local_position = Point()
        
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.SYSTEM_DEFAULT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Callback groups
        # control_messages_callback_group is ONLY for control messages such as mode changes, arming and other major control messages 
        self.control_messages_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.main_loop_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        
        self.inbound_telemetry_messages_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.outbound_telemetry_messages_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.control_message_utility_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()        
        
        self.executor : rclpy.executors.MultiThreadedExecutor = executor
        
        # For callbacks. Do not remove this block
        self.starting_position : NavSatFix = None
        self.current_mode : str = None
        self.armed : bool = None
        self.mavros_connected : bool = None
        self.requested_mode : str = None
        self.current_local_pose : Point = None
        self.current_waypoint_index = -1
        self.reach_break_pt = False
        # For manual overrides
        self.DRONE_RTLD_ON_INTENTION : bool = False
        self.DRONE_MODE_CHANGED_ON_INTENTION : str = ""
        
        self.mav_pub = self.create_publisher(StatusText, "/artemis/status", 10)

        self.target_height : float = 15.0
        self.capture = cv2.VideoCapture(0)
        self.detected_hotspots = []
        self.object_counting_done = False
        self.offset = None
        self.detected_obj = []
        self.object_counts = {"triangle":0,"square":0}
        self.recent_poses = []
        self.rectangle_array =[]
        self.triangle_array = []
        self.image_array = []
        self.mask_array =[]
        self.num_detected_hotspots = 0
        self.new_hotspot = False
        self.is_new_hotspot = False
        self.t0 = time.time()
        self.checker_list = []
        self.stop_for_object = False
        self.hovering_obj = False
        
        while self.starting_position is None:
            self.get_starting_position()

        self.startup_loop_timer = self.create_timer(1.0, callback=self.startup_loop , callback_group=None)
        self.roi_type = None
        self.data = {}
    
    def is_new_circle(self, point: Point, threshold: float = 35.0) -> bool:
        """Check if the detected circle is new or has been visited before."""
        for center in self.detected_center:
            if (abs(center.pose.position.x - point.x) < threshold and
                abs(center.pose.position.y - point.y) < threshold):
                return False
        return True
    
    def startup_loop(self) -> None:
        self.startup_loop_timer.cancel()
        self.get_logger().info("Execution of Startup Sequence Started!")
        self.change_mode("GUIDED")
        self.initialize()
        time.sleep(2)
        self.arm_drone()
        
        msg = StatusText()
        msg.severity = 1
        msg.text = f"Starting Mission 2"
        self.mav_pub.publish(msg)

        self.ardu_offset = None
        while self.ardu_offset is None:
            self.ardu_offset : Point = Point(x = self.current_local_pose.x, y = self.current_local_pose.y, z = self.current_local_pose.z)
        self.takeoff_at_position(self.starting_position, altitude=15.0)
        time.sleep(5)
        
        temp_msg = PoseStamped(
                    header = Header(frame_id="map", stamp=self.get_clock().now().to_msg()),
                    pose=Pose(
                    position = Point(
                        x = self.ardu_offset.x,
                        y = self.ardu_offset.y,
                        z = 15.0+ self.ardu_offset.z),
                    orientation = self.quaternion))  
        self.waypoint_publisher.publish(temp_msg)

        time.sleep(6)
        self.move_to_waypoint(self.mission_start_point_latitude, self.mission_start_point_longitude)
        self.get_logger().debug("Startup Loop Execution Complete!")
        while self.offset is None:
            self.offset = Point(x = self.current_local_pose.x, y = self.current_local_pose.y, z = self.current_local_pose.z)    

        self.get_logger().debug("Starting Main Loop Execution")
        
        msg = StatusText()
        msg.severity = 1
        msg.text = f"Starting Search Algorithm"
        self.mav_pub.publish(msg)

        self.main_loop_timer = self.create_timer(0.25, callback=self.main_loop, callback_group=self.main_loop_callback_group)

    def main_loop(self) -> None:
        self.navigate()

    # Control and Navigation related functions
    def navigate(self) -> None:
        if self.num_detected_hotspots == 3 and self.object_counting_done == True:
            self.finish_mission()
            return

        z_actual = self.current_local_pose.z - self.ardu_offset.z

        self.height_check = False
        if (z_actual > self.target_height - 1) and (z_actual < self.target_height + 1):
            self.height_check = True

        if self.height_check == True and self.Follow_target == False and self.reach_break_pt == False:

            self.roi_type,self.data = self.computer_vision()

        if self.roi_type == "hotspot" and self.reach_break_pt == False and self.stop_for_object == False:
            if self.Follow_target == False:
                self.Break_coordinate = Point(x=self.current_local_pose.x, y=self.current_local_pose.y, z=self.current_local_pose.z)

            self.Follow_target = True
            target_point = self.data["pose"]
            
            self.publish_target_point(target_point, self.quaternion)
            self.get_logger().info(f"going to Hotspot center")

            if abs(self.current_local_pose.x - target_point.x) < self.mission_waypoint_tolerance and abs(self.current_local_pose.y - target_point.y ) < self.mission_waypoint_tolerance:
                self.get_logger().info("Hovering at circle center for 5 seconds.")
                time.sleep(3)
                _, frame = self.capture.read()
                _,mask,res,img = self.hotspot_checker(frame)
                self.report_hotspot(img,mask,target_point)
                time.sleep(2)

                self.Follow_target = False

                break_msg = PoseStamped(
                    header = Header(frame_id="map", stamp=self.get_clock().now().to_msg()),
                    pose=Pose(
                    position = Point(
                        x = self.Break_coordinate.x,
                        y = self.Break_coordinate.y,
                        z = self.Break_coordinate.z),
                    orientation = self.quaternion))  
                self.waypoint_publisher.publish(break_msg)
                self.reach_break_pt = True
            else:    
                self.publish_target_point(target_point, self.quaternion)

        elif self.roi_type == "object" and self.reach_break_pt == False:
            target_point = self.data["pose"]

            if self.stop_for_object ==False:
                self.Break_coordinate = Point(x=self.current_local_pose.x, y=self.current_local_pose.y, z=self.current_local_pose.z)
                self.Follow_target = True
                self.stop_for_object = True

                self.publish_target_point(target_point, self.quaternion)
                self.get_logger().info(f"going to object center")
                msg = StatusText()
                msg.severity = 1
                msg.text = f"going to object center"
                self.mav_pub.publish(msg)
                


            if abs(self.current_local_pose.x - target_point.x) < self.mission_waypoint_tolerance and abs(self.current_local_pose.y - target_point.y ) < self.mission_waypoint_tolerance and self.Follow_target ==True:
                self.hovering_obj = True
            else:
                self.publish_target_point(target_point, self.quaternion)
                        
            if self.hovering_obj == True:
                if(len(self.rectangle_array)<10):
                    self.publish_target_point(target_point, self.quaternion)

                    ret, frame = self.capture.read()
                    if ret: 
                        _, mask_obj , rect , triangle, img,mask_white= self.object_checker(frame)
                        self.triangle_array.append(triangle)
                        self.rectangle_array.append(rect)
                        self.image_array.append(img)
                        self.mask_array.append(mask_obj)
                else:
                    self.report_object()
                    break_msg = PoseStamped(
                        header = Header(frame_id="map", stamp=self.get_clock().now().to_msg()),
                        pose=Pose(
                        position = Point(
                            x = self.Break_coordinate.x,
                            y = self.Break_coordinate.y,
                            z = self.Break_coordinate.z),
                        orientation = self.quaternion))  
                    self.waypoint_publisher.publish(break_msg)

                    self.reach_break_pt = True
                    self.stop_for_object = False
                    self.Follow_target = False
                    
                

        elif self.reach_break_pt:
            i = 0 
            while abs(self.current_local_pose.x - self.Break_coordinate.x) > self.mission_waypoint_tolerance or abs(self.current_local_pose.y - self.Break_coordinate.y) > self.mission_waypoint_tolerance:
                break_msg = PoseStamped(
                    header = Header(frame_id="map", stamp=self.get_clock().now().to_msg()),
                    pose=Pose(
                    position = Point(
                        x = self.Break_coordinate.x,
                        y = self.Break_coordinate.y,
                        z = self.Break_coordinate.z),
                    orientation = self.quaternion))  
                self.waypoint_publisher.publish(break_msg)
                if i ==0:
                    self.get_logger().info("Moving towards the break coordinate")
                    i +=1
            if abs(self.current_local_pose.x - self.Break_coordinate.x) < self.mission_waypoint_tolerance and abs(self.current_local_pose.y - self.Break_coordinate.y ) < self.mission_waypoint_tolerance:
                self.reach_break_pt = False
                self.get_logger().info("start reentery")
        
        # Search Algo
        elif self.current_waypoint_index == -1:
            msg = PoseStamped()
            msg.pose.position = Point(x = self.offset.x, y = self.offset.y, z = self.offset.z + self.ardu_offset.z)
            msg.pose.orientation = self.quaternion

            while self.current_local_pose.z <= 14.0 + self.ardu_offset.z:
                self.waypoint_publisher.publish(msg)
            
            self.get_logger().info("Takeoff complete. Starting Search Algorithm")
            self.current_waypoint_index = 0
            return

        elif self.current_waypoint_index < len(self.waypoints):
            current_waypoint = self.waypoints[self.current_waypoint_index]
            
            if abs(self.current_local_pose.x - current_waypoint.x - self.offset.x) < self.mission_waypoint_tolerance and abs(self.current_local_pose.y - current_waypoint.y - self.offset.y) < self.mission_waypoint_tolerance:
                self.current_waypoint_index += 1
                self.get_logger().debug(f"Waypoint index {self.current_waypoint_index} reached")
                if self.current_waypoint_index < len(self.waypoints):
                    self.publish_waypoint(self.current_waypoint_index, self.quaternion)
                    self.get_logger().info(f"Published Waypoint: x : {self.waypoints[self.current_waypoint_index].x + self.offset.x}, y : {self.waypoints[self.current_waypoint_index].y + self.offset.y}")
                else:
                    self.finish_mission()
            else:
                self.publish_waypoint(self.current_waypoint_index, self.quaternion)


    def calculate_new_coordinate(self, mask, altitude, orientation : Quaternion) -> Point:
        "Calculate Target Position of the Drone relative to the image pose"
        if mask.ndim == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        mask_center_x = x + w // 2
        mask_center_y = y + h // 2

        # Compute the center of the image
        image_height, image_width = mask.shape
        image_center_x = image_width // 2
        image_center_y = image_height // 2

        x_distance_pixels = mask_center_x - image_center_x        
        y_distance_pixels = image_center_y - mask_center_y 

        fov_degrees_x = 81
        fov_degrees_y = 46.2

        fov_radians_x = math.radians(fov_degrees_x)
        fov_radians_y = math.radians(fov_degrees_y)

        actual_width_x = 2 * math.tan(fov_radians_x/2) * altitude 
        actual_width_y = 2 * math.tan(fov_radians_y/2) * altitude
        
        x_difference = (x_distance_pixels / image_width) * actual_width_x
        y_difference = (y_distance_pixels / image_height) * actual_width_y
        
        target_point = Point(
            x = float(y_difference),
            y = -float(x_difference),
            z = 0.0
        )
        
        yaw = quaternion_to_yaw(orientation)
        
        return field_frame_to_enu_frame(
            point=target_point,
            field_rotation_in_radians=yaw
        )        

    def computer_vision(self) -> Point:
        
        if self.capture.isOpened():
            ret, frame = self.capture.read()
            if ret:
                image_pose = Point(x = self.current_local_pose.x, y = self.current_local_pose.y, z = self.current_local_pose.z)
                image_orientation = Quaternion(x=self.current_orientation.x,y=self.current_orientation.y,z=self.current_orientation.z,w=self.current_orientation.w)

                check_circle,mask,res,img = self.hotspot_checker(frame)
                check_object, mask_obj , rectangle1 , triangle, img_obj,mask_white = self.object_checker(frame)

                if check_circle is None and check_object is None:
                    self.c=0
                    return None,None
 
            else:
                self.c=0
                self.get_logger().error("Failed to read frame from camera")
                return None,None
        else:
            self.c=0
            self.get_logger().error("Capture source not open")
            return None,None


        if check_circle != None:
            if self.c ==2:
                self.checker_list = []
                self.c = 0 

            else:
                
                for i in self.checker_list:
                    if self.calculate_psnr(i,mask)<30:
                        self.c=0
                        self.checker_list = []
                        return None,None

                self.c+=1
                self.checker_list.append(mask)
            
            threshold_distance = 5.0 
            new_coord = self.calculate_new_coordinate(mask,image_pose.z - self.ardu_offset.z,image_orientation)
            new_coord.z = self.target_height + self.ardu_offset.z
            new_coord.x = new_coord.x + image_pose.x
            new_coord.y = new_coord.y + image_pose.y            

            for hotspot in self.detected_hotspots:
                
                hotspot_x = hotspot['x']
                hotspot_y = hotspot['y']
            
                if abs(new_coord.x - hotspot_x) < threshold_distance and abs(new_coord.y - hotspot_y) < threshold_distance:
                    return None,None
            

            target_point = Point(
            x = new_coord.x,
            y = new_coord.y,
            z = new_coord.z
        )
            data = {"img":img,"mask":mask,"pose":target_point}
            self.get_logger().info(f"Hotspot detected at x : {new_coord.x:.3f} y : {new_coord.y:.3f}") 
            roi_type = "hotspot" 
        else:
            if self.object_counting_done:
                return None,None

            if(len(self.rectangle_array)>10):
                    return None,None
            roi_type = "object"
            _, binary_mask = cv2.threshold(mask_white, 127, 255, cv2.THRESH_BINARY)
            new_coord = self.calculate_new_coordinate(binary_mask,image_pose.z - self.ardu_offset.z,image_orientation)
            new_coord.z = self.target_height + self.ardu_offset.z
            new_coord.x = new_coord.x + image_pose.x
            new_coord.y = new_coord.y + image_pose.y 

            target_point = Point(
            x = new_coord.x,
            y = new_coord.y,
            z = new_coord.z
        )
            i = len(self.rectangle_array) + self.hovering_obj
            data = {"mask":mask_obj,"triangles":triangle,"squares":rectangle1,"img":img_obj,"pose":target_point}
            cv2.imwrite(f"./mission2/mask{i}.jpg",mask_obj)
            

        return roi_type,data

    def calculate_psnr(self,img1, img2):
        # Check if the images have the same shape
        if img1.shape != img2.shape:
            raise ValueError("Images do not have the same shape")
        
        mse = np.mean((img1 - img2) ** 2)
        if mse == 0:
            return float('inf')  # PSNR is infinite if there is no difference between the images
        
        max_pixel = 255.0
        psnr = 20 * np.log10(max_pixel / np.sqrt(mse))
        return psnr
    def add_HSV_filter(self,frame):

        blur = cv2.GaussianBlur(frame,(5,5),0) 

        # Converting RGB to HSV
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        l_b_r = np.array([0,50,0])        # Lower limit for red ball
        u_b_r = np.array([179, 255, 255])       # Upper limit for red ball
        l_b_l = np.array([143,110,50])        # Lower limit for red ball
        u_b_l = np.array([179, 255, 255])       # Upper limit for red ball
        mask_l = cv2.inRange(hsv,l_b_l,u_b_l)
        mask_r = cv2.inRange(hsv,l_b_r,u_b_r)
        mask = mask_l + mask_r
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        return mask
    def hotspot_checker(self,image):
        image_copy = image.copy()
        frame = self.apply_area_filter(image)
        mask = self.add_HSV_filter(frame)

        _, binary_mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)

        kernel = np.ones((3,3), np.uint8)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter out non-circular and small contours
        filtered_mask = np.zeros_like(binary_mask)

        # Define the area and circularity thresholds
        min_area = 2400  # Minimum area of the circle (adjust based on your image)
        circularity_threshold = 0.85  # Circularity threshold
        i = None
        for contour in contours:
            # Calculate the area and perimeter
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            if perimeter == 0:
                continue  # Avoid division by zero

            # Calculate circularity
            circularity = (4 * np.pi * area) / (perimeter ** 2)

            # If circularity and area meet the criteria, keep it
            if circularity > circularity_threshold and area > min_area:
                i =1
                cv2.drawContours(filtered_mask, [contour], -1, 255, thickness=cv2.FILLED)

        # cv2.imwrite("./test_images/mask01.jpg",filtered_mask)

        res = cv2.bitwise_and(image, image, mask=filtered_mask) 

        return i,filtered_mask,res,image_copy
    
    def find_max_frequency(self,arr):
        if not arr:
            return None, None  # Handle empty array case

        frequency = {}
        for index, value in enumerate(arr):
            if value in frequency:
                frequency[value] += 1
            else:
                frequency[value] = 1
        max_freq_value = None
        max_freq_count = 0
        max_freq_index = -1

        for index, value in enumerate(arr):
            if frequency[value] > max_freq_count:
                max_freq_count = frequency[value]
                max_freq_value = value
                max_freq_index = index

        return max_freq_index, max_freq_value

    def report_object(self):
        if(self.object_counting_done == False):
            rect_idx , rectangle = self.find_max_frequency(self.rectangle_array)
            triangle_idx , triangle = self.find_max_frequency(self.triangle_array)
            img = self.image_array[rect_idx]
            mask = self.mask_array[rect_idx]
            if(rect_idx ==  triangle_idx) :
                img = self.image_array[rect_idx]
                mask = self.mask_array[rect_idx]
            else :
                for i in range(10):
                    if(rectangle == self.rectangle_array[i] and triangle == self.triangle_array[i]):
                        rect_idx = i
                        img = self.image_array[rect_idx]
                        mask = self.mask_array[rect_idx]
                    else :
                        pass
            time_now = time.time()
            obj_position = {
                    'x': self.current_local_pose.x,
                    'y': self.current_local_pose.y,
                    'square': rectangle,
                    'triangle' : triangle,
                    'Real time': str(datetime.datetime.now(pytz.timezone('Asia/Kolkata'))),
                    'Time from mission start': f"{round(time_now - self.t0,2)} seconds"
                    }
            self.detected_obj.append(obj_position)
            self.get_logger().info(f' Square {rectangle} , triangle {triangle}')
            self.save_obj(img,mask)  # Save the updated hotspots to JSON
            msg = StatusText()
            msg.severity = 1
            msg.text = f"object detected {rectangle} Sqaures and {triangle} triangles"
            self.mav_pub.publish(msg)
            self.object_counting_done = True
        else :
            self.get_logger().info('objects already detected')

    def save_obj(self,img,mask):
        with open("./mission2/object_count.json", 'w+') as f:
            json.dump(self.detected_obj, f, indent=4)
        cv2.imwrite(f"./mission2/object_image.jpg",img)
        cv2.imwrite(f"./mission2/object_mask.jpg",mask)

    def apply_area_filter(self,frame, min_area=1000, max_area=100000000000):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])  # Lower bound for white
        upper_white = np.array([180, 25, 255])  # Upper bound for white
        mask = cv2.inRange(hsv_frame, lower_white, upper_white)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_white_portions = np.zeros_like(frame)
        for contour in contours:
            area = cv2.contourArea(contour)
            if min_area < area < max_area:
                contour_mask = np.zeros_like(mask)
                cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
                filtered_white_portions += cv2.bitwise_and(frame, frame, mask=contour_mask)
        return filtered_white_portions

    def find_red_shapes(self,binary_mask):
        kernel = np.ones((5, 5), np.uint8)
        # binary_mask = cv2.GaussianBlur(binary_mask, (5, 5), 0)
        # binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)
        # binary_mask = cv2.dilate(binary_mask, kernel, iterations=1)  # Enhanced noise reduction

        min_area = 500# Adjust based on actual size requirements
        max_area = 10000000  # Maximum area constraint

        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        filtered_mask = np.zeros_like(binary_mask)
        rectangle_count = 0
        triangle_count = 0

        for contour in contours:
            area = cv2.contourArea(contour  )
            if min_area < area <= max_area:
                epsilon = 0.04 * cv2.arcLength(contour, True)  # Adjust epsilon for better approximation
                approx = cv2.approxPolyDP(contour, epsilon, True)

                if len(approx) == 3:  # Triangle
                    triangle_count += 1
                    cv2.drawContours(filtered_mask, [approx], -1, 255, thickness=cv2.FILLED)
                elif len(approx) == 4:  # Rectangle or square
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h
                    if 0.8 < aspect_ratio < 1.2 or cv2.isContourConvex(approx):  # More general check for rectangles
                        rectangle_count += 1
                        cv2.drawContours(filtered_mask, [approx], -1, 255, thickness=cv2.FILLED)

        return filtered_mask, rectangle_count, triangle_count
   
    def add_red_filter(self,image):
        blur = cv2.GaussianBlur(image, (5, 5), 0) 

        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)


        l_b_r = np.array([0, 25, 0])
        u_b_r = np.array([179, 255, 255])
        l_b_l = np.array([0, 0, 60])
        u_b_l = np.array([100, 100, 100])

        mask_r = cv2.inRange(hsv, l_b_r, u_b_r)
        mask_l = cv2.inRange(hsv, l_b_l, u_b_l)
        mask = mask_r + mask_l

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        _, binary_mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
        
        filtered_mask, rectangle_count, triangle_count = self.find_red_shapes(binary_mask)
        return filtered_mask,rectangle_count, triangle_count

    def object_checker(self,image):
        image_copy = image.copy()
        white_filter = self.apply_area_filter(image_copy)
        mask , rectangle2, triangle = self.add_red_filter(white_filter)
        if(rectangle2==0 or triangle==0):
            return None, mask , rectangle2 , triangle, image_copy,white_filter
        else :
            return True ,mask, rectangle2 ,triangle,image_copy,white_filter
    
    def report_hotspot(self,img,mask,pose):
        self.num_detected_hotspots +=1
        time_now = time.time()
        hotspot_position = {
                'x': pose.x,
                'y': pose.y,
                'Hotspot_ID': self.num_detected_hotspots,
                'Real time': str(datetime.datetime.now(pytz.timezone('Asia/Kolkata'))),
                'Time from mission start': f"{round(time_now - self.t0,2)} seconds"
                }
        self.detected_hotspots.append(hotspot_position)
        self.save_hotspots(img,mask)  # Save the updated hotspots to JSON
        self.get_logger().info(f"Hotspot detected: {hotspot_position}")
        msg = StatusText()
        msg.severity = 1
        msg.text = f"Hotspot{self.num_detected_hotspots} detected"
        self.mav_pub.publish(msg)


    def save_hotspots(self,img,mask):
        with open("./mission2/Hotspots.json", 'w+') as f:
            json.dump(self.detected_hotspots, f, indent=4)
        cv2.imwrite(f"./mission2/hotspot{self.num_detected_hotspots}.jpg",img)
        cv2.imwrite(f"./mission2/mask{self.num_detected_hotspots}.jpg",mask)



    def move_to_waypoint(self, latitude, longitude) -> None:
        """One time use global reposition to new point. This function is blocking"""
        self.get_logger().info(f"Moving to waypoint {latitude}, {longitude}")
        self.publisher = self.create_publisher(GeoPoseStamped, "/mavros/setpoint_position/global", qos_profile=1, callback_group=self.control_message_utility_callback_group)
        message = GeoPoseStamped()
        message.pose.position = GeoPoint()
        message.pose.position.latitude = latitude
        message.pose.position.longitude = longitude
        message.pose.position.altitude = geoid_height_to_amsl(lat=latitude, lon=longitude, height=self.starting_position.altitude + 15.0)
        initial_position : NavSatFix = None
        wpt_reached_flag = False
        def update_current_position(position : NavSatFix) -> None:
            nonlocal initial_position, wpt_reached_flag
            if wpt_reached_flag:
                return
            if initial_position is None:
                initial_position = position
            if abs(latitude - position.latitude) < 5e-6 and abs(longitude - position.longitude) < 5e-6:
                wpt_reached_flag = True
                self.get_logger().info(f"Waypoints reached")
            else:
                self.get_logger().debug(f"Current position at {position.latitude}, {position.longitude}")
                time.sleep(3) 
        subscription = self.create_subscription(NavSatFix, "/mavros/global_position/global", callback_group=self.control_messages_callback_group, callback=update_current_position, qos_profile=self.qos_profile)
        while initial_position is None:
            time.sleep(0.5)
        message.pose.orientation = get_quaternion(initial_position.latitude, initial_position.longitude, latitude, longitude)
        while wpt_reached_flag == False:
            self.publisher.publish(message)
            time.sleep(1)
        self.get_logger().info("Waypoint reached")  
        self.destroy_subscription(subscription)

    def publish_waypoint(self, waypoint_index : int, quaternion : Quaternion) -> None:
        waypoint : Point = self.waypoints[waypoint_index]
        msg = PoseStamped(
            header = Header(frame_id="map", stamp=self.get_clock().now().to_msg()),
            pose=Pose(
            position = Point(
                x = self.offset.x + waypoint.x,
                y = self.offset.y + waypoint.y,
                z = waypoint.z + self.ardu_offset.z),
            orientation = quaternion))
        self.waypoint_publisher.publish(msg)
         

    def publish_target_point(self, point : Point, quaternion : Quaternion) -> None:
        msg = PoseStamped(
            header = Header(frame_id="map", stamp=self.get_clock().now().to_msg()),
            pose=Pose(
            position = Point(
                x = point.x,
                y = point.y,
                z = point.z),
            orientation = quaternion))
        self.waypoint_publisher.publish(msg)

    def get_starting_position(self) -> None:   
        subscription = self.create_subscription(NavSatFix, "/mavros/global_position/global", callback_group=self.inbound_telemetry_messages_callback_group, callback=self.current_globalpos_callback, qos_profile=self.qos_profile)
        while (self.starting_position is None) and (type(self.starting_position) != NavSatFix) :
            rclpy.spin_once(self)
        self.destroy_subscription(subscription)

    def finish_mission(self) -> None:
        self.get_logger().info("Mission Complete! Returning to Launch")
        self.change_mode("RTL")
        self.main_loop_timer.cancel()
        rclpy.shutdown()

    # Initialization functions
    
    def initialize(self) -> None:
        self.initialize_subscriptions()
        self.initialize_publishers()
        self.initialize_control_services()
        self.initialize_service_clients()
        self.get_logger().info("Initialization complete")

    # Subscriptions
    def initialize_subscriptions(self) -> None:
        self.state_subscription = self.create_subscription(State, "/mavros/state", callback_group=self.inbound_telemetry_messages_callback_group, callback=self.state_update_callback, qos_profile=self.qos_profile)
        self.local_position_subscription = self.create_subscription(PoseStamped, "/mavros/local_position/pose", callback_group=self.inbound_telemetry_messages_callback_group, callback=self.current_localpos_callback, qos_profile=self.qos_profile)
        

    # Pushlishers
    def initialize_publishers(self) -> None:
        self.waypoint_publisher = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", callback_group=self.outbound_telemetry_messages_callback_group, qos_profile=self.qos_profile)

    # Timers
    def initialize_control_services(self) -> None:
        self.manual_override_timer = self.create_timer(1.0, callback=self.manual_control_override, callback_group=self.control_message_utility_callback_group)        
        
    # Service clients
    def initialize_service_clients(self) -> None:
        ...
   

    def state_update_callback(self, state : State) -> None:
        self.armed = state.armed
        self.mavros_connected = state.connected
        self.current_mode = state.mode
    
    def current_localpos_callback(self, position : PoseStamped) -> None:
        self.current_local_pose : Point = position.pose.position
        self.current_orientation : Quaternion = position.pose.orientation


    def current_globalpos_callback(self, position : NavSatFix) -> None:
        self.starting_position = position

    # Control functions
    def change_mode(self, mode : str) -> None:
        if mode != "RTL":
            self.DRONE_MODE_CHANGED_ON_INTENTION = mode
        else:
            self.DRONE_RTLD_ON_INTENTION = True
        mode_service = self.create_client(SetMode, "/mavros/set_mode", callback_group=self.control_messages_callback_group)
        mode_service.wait_for_service()
        self.get_logger().debug("Mode service available!")
        while True:
            self.get_logger().debug(f"Sending {mode} mode request")
            result : SetMode.Response = mode_service.call(
                SetMode.Request(custom_mode=mode)
            )
            if result.mode_sent:
                self.get_logger().info(f"Switched to {mode} mode")
                break
            else:
                self.get_logger().warning(f"Failed to send {mode} mode. Retrying in 5 seconds")
                time.sleep(5)

    def set_home_current_gps(self) -> None:
        client = self.create_client(CommandHome, "/mavros/cmd/set_home", callback_group=self.control_messages_callback_group)
        client.wait_for_service()
        home_set = False
        while home_set is False:
            request = CommandHome.Request(
                current_gps=True,
                latitude=0.0,
                longitude=0.0,
                altitude=0.0,
            )
            response : CommandHome.Response = client.call(request)
            if response.success:
                self.get_logger().info("Home location set to current GPS location")
                break
            self.get_logger().warning("Failed to set home location. Retrying in 10 seconds")
            time.sleep(10)
    
    def set_home(self, position : NavSatFix) -> None:
        self.get_logger().info("Setting home location requested")
        client = self.create_client(CommandHome, "/mavros/cmd/set_home", callback_group=self.control_messages_callback_group)
        client.wait_for_service()
        home_set = False
        while home_set is False:
            request = CommandHome.Request(
                current_gps=False,
                latitude=position.latitude,
                longitude=position.longitude,
                altitude = position.altitude,)
            response : CommandHome.Response = client.call(request)
            if response.success:
                self.get_logger().info(f"Home location set to {position.latitude}, {position.longitude}, {position.altitude}")
                break
            self.get_logger().warning("Failed to set home location. Retrying in 10 seconds")
            time.sleep(10)
            
    def arm_drone(self) -> None:
        "Arms the drone"
        # Mode is set, arm the drone
        client = self.create_client(CommandBool, "/mavros/cmd/arming", callback_group=self.control_messages_callback_group)
        client.wait_for_service()
        while self.armed is False:
            request = CommandBool.Request(value=True)
            response : CommandBool.Response = client.call(request)
            if response.success:    
                self.get_logger().info("Drone arming response: Success")
                break
            self.get_logger().warning("Failed to arm drone. Retrying in 10 seconds")
            time.sleep(10)
            if self.armed:
                break

    def takeoff_at_position(self, position : NavSatFix, altitude : float = 15.0):
        client = self.create_client(CommandTOL, "/mavros/cmd/takeoff", callback_group=self.control_messages_callback_group)
        client.wait_for_service()
        while True:
            request = CommandTOL.Request(
                latitude=position.latitude,
                longitude=position.longitude,
                altitude=float(altitude),
            )
            result : CommandTOL.Response = client.call(request)
            if result.success:
                break
            self.get_logger().warning("Failed to takeoff. Retrying in 10 seconds")
            time.sleep(10)
   
    def manual_control_override(self) -> None:
        """WARNING! Use with caution. This function is capable of killing ABSOLUTELY EVERYTHING that your node is currently doing."""
        current_mode = self.current_mode.strip()
        if self.DRONE_MODE_CHANGED_ON_INTENTION == current_mode:
            return
        if current_mode == "RTL" and self.DRONE_RTLD_ON_INTENTION:
            return
        self.get_logger().fatal(f'Drone switched out of {"RTL" if self.DRONE_RTLD_ON_INTENTION else "GUIDED"} Mode to {current_mode}.\n Killing ALL Tasks')
        rclpy.shutdown()
        raise KeyboardInterrupt()


def command_line_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mission_configuration", type=str, required=True, help="Path to the mission configuration JSON file")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    args , *unknown = parser.parse_known_args()
    return args

def main():
    # Get the appropriate command line arguments
    # arguments = command_line_args()
    rclpy.init()
    multi_threaded_executor = MultiThreadedExecutor(num_threads=4 )
    
    mission = Mission("src/Arty-Part2/artemis/autopilot_configs/mission2.json")
    autopilot_node = Autopilot(mission=mission, executor=multi_threaded_executor)
    
    if True:    
        autopilot_node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    
    multi_threaded_executor.add_node(autopilot_node)
    try:
        autopilot_node.get_logger().info("Starting Autopilot Node. CTRL-C to shutdown")
        multi_threaded_executor.spin()
    except KeyboardInterrupt:
        print("Got Keyboard Interrupt, killing all tasks")
        rclpy.shutdown()
        multi_threaded_executor.shutdown()
        autopilot_node.destroy_node()
        sys.exit(1)
    except Exception as e:
        print(e)
        rclpy.shutdown()
        autopilot_node.destroy_node()
        multi_threaded_executor.shutdown()
        sys.exit(1)        

if __name__ == "__main__":
    main()
