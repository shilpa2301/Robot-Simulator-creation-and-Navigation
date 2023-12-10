# Authors: Shilpa Mukhopadhyay
# UIN: 433003777
# Instructor: Dr. Jason O'Kane
# TA: Aaron Kingery
# Course ID: CSCE 752
# Course Title: Robotics and Spatial Intelligence
# Project 4_d
#Contributions :Class Simulator_Node and class Velocity_translator_node is a joint Contribution with Junsuk Kim and Amrita Mohandas
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Float64
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from math import sin, cos, pi
import math
from sensor_msgs.msg import LaserScan
import time
import copy
from functools import partial
import sys
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

COLLISION_TIME_THRESHOLD=0.5
class Simulator_node(Node):
    def __init__(self):
        super().__init__('robot_simulator')
        #We need the parameters radius, distance, etc.
        self.declare_parameter('radius', 0.2)
        self.declare_parameter('distance', 0.3)
        self.declare_parameter('error_update_rate', 60.0)
        self.declare_parameter('error_variance_left',0.0)
        self.declare_parameter('error_variance_right',0.0)
        self.declare_parameter('initial_pose',[0.0,0.0,0.0])
        
        # The Broadcaster will publish tf data
        self.tf_broadcaster = TransformBroadcaster(self)

        # Get the pose and parameters
        self.robot_radius = self.get_parameter('radius').value
        self.wheel_distance = self.get_parameter('distance').value  
        self.error_update_rate = self.get_parameter('error_update_rate').value
        self.error_variance_left = self.get_parameter('error_variance_left').value
        self.error_variance_right = self.get_parameter('error_variance_right').value
        self.initial_pose=self.get_parameter('initial_pose').value
        self.current_x=self.initial_pose[0]
        self.current_y=self.initial_pose[1]
        self.current_theta=0.0 #self.initial_pose[2]
        
        # We have to subsribe the vl, vr with 100 to not lose any data
        self.vl_subscription = self.create_subscription(Float64, '/vl', self.vl_callback, 100)
        self.vr_subscription = self.create_subscription(Float64, '/vr', self.vr_callback, 100)
        # 0.01 to have a fair value
        self.timer = self.create_timer(0.001, self.update_pose)    
            
        self.collision_timer=self.create_timer(0.001,self.collision_builder)
        # Initialize the vl, vr and errors for each one
        self.vl = 0
        self.vr = 0
        self.vr_err=1
        self.vl_err=1
        
        # For delta T, we need the time
        self.last_update_time = self.get_clock().now()
        self.last_vel_time = self.get_clock().now()
        # We also need the time_for_error_update
        self.time_for_error_update = self.get_clock().now()
###############################################
        self.declare_parameter('laser_rate', 1)
        self.declare_parameter('laser_count', 25)
        self.declare_parameter('laser_angle_min', -1.8)
        self.declare_parameter('laser_angle_max', 1.8)
        self.declare_parameter('laser_range_min', 0.01)
        self.declare_parameter('laser_range_max', 40.00)
        self.declare_parameter('laser_error_variance', 0.001)
        self.declare_parameter('laser_fail_probability', 0.1),
        self.laser_rate = self.get_parameter('laser_rate').value
        self.laser_count = self.get_parameter('laser_count').value
        self.laser_angle_min = self.get_parameter('laser_angle_min').value
        self.laser_angle_max = self.get_parameter('laser_angle_max').value
        self.laser_range_min = self.get_parameter('laser_range_min').value
        self.laser_range_max = self.get_parameter('laser_range_max').value
        self.laser_error_variance = self.get_parameter('laser_error_variance').value
        self.laser_fail_probability = self.get_parameter('laser_fail_probability').value
        self.laserscan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        frame_id = 'laser'
        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = frame_id

        self.laser_subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 100)
        self.collision_flag = False
        
##################################################
        # Create a publisher for the /map topic
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        # Set the frame_id to 'world'
        frame_id = 'world'
        # Create an OccupancyGrid message
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = frame_id
        self.declare_parameter('resolution', 0.0)
        self.declare_parameter('map','')
       
        # Get robot information
        self.robot_map = self.get_parameter('map').value
        
        self.map_msg.info.resolution=self.get_parameter('resolution').value
        self.resolution=self.map_msg.info.resolution
        self.map_msg.info.width = 0
        self.map_msg.info.height = 0
        
        # Adjust the origin to flip the map
        self.map_msg.info.origin.position.x = 0.0
        self.map_msg.info.origin.position.y = 0.0
        self.map_msg.info.origin.position.z = 0.0
        
        # Flip the orientation to achieve the desired orientation
        self.map_msg.info.origin.orientation.x = 0.0
        self.map_msg.info.origin.orientation.y = 0.0
        self.map_msg.info.origin.orientation.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0
        self.map_data = []
        self.num_rows=0
        self.num_cols=0
        # Iterate over the reversed order of lines to flip the map vertically
        self.map_coordinate_data = {}

        self.construct_map()
        self.timer = self.create_timer(3.0, self.publish_map)
        self.timer = self.create_timer(1 / self.laser_rate, self.publish_laserscan)

        self.robot_pose_publisher = self.create_publisher(PoseStamped, '/robot_pose', 10)

    
    def calculate_points(self, start_point, angle):
        # Calculate the end point of the laser beam
        current_theta = self.current_theta
        if current_theta < -math.pi:
            current_theta += 2 * math.pi
        elif current_theta > math.pi:
            current_theta -= 2 * math.pi

        # Calculate the start and end points of the laser beam
        laser_start_x = start_point[0]
        laser_start_y = start_point[1]
        angle_ = angle + current_theta

        # Normalize the angle
        if angle_ < -math.pi:
            angle_ += 2 * math.pi
        elif angle_ > math.pi:
            angle_ -= 2 * math.pi

        #calculate the end point of the laser beam
        laser_end_x = laser_start_x + self.laser_range_max * math.cos(angle_)
        laser_end_y = laser_start_y + self.laser_range_max * math.sin(angle_)

        # Find intersection point with obstacles

        min_distance = float('inf')  # Initialize with infinity
        
        #calculate the box segments
        for obstacle in self.map_coordinate_data.keys():
            obstacle_x1, obstacle_y1, obstacle_x2, obstacle_y2 = obstacle
            obstacles=self.get_box_segments([[obstacle_x1, obstacle_y1], [obstacle_x2, obstacle_y2]])

            for i in range(4):
                intersection = self.find_intersection(
                    [laser_start_x, laser_start_y],[laser_end_x, laser_end_y],
                    obstacles[i][0],obstacles[i][1]
                )
                if intersection is not None:
                    # Calculate distance to the intersection point
                    distance = math.sqrt((laser_start_x - intersection[0]) ** 2 + (laser_start_y - intersection[1]) ** 2)

                    # Check if this intersection is closer than the previous ones
                    if distance < min_distance:
                        min_distance = distance


        return min_distance
    def get_box_segments(self,box):
        x1, y1 = box[0]
        x2, y2 = box[1]

        # Box left
        p_left = [x1, y1]
        q_left = [x1, y2]

        # Box right
        p_right = [x2, y1]
        q_right = [x2, y2]

        # Box top
        p_top = [x1, y2]
        q_top = [x2, y2]

        # Box bottom
        p_bottom = [x1, y1]
        q_bottom = [x2, y1]

        return [p_left, q_left], [p_right, q_right], [p_top, q_top], [p_bottom, q_bottom]

    def find_intersection(self,p1, q1, p2, q2):
        def orientation(p, q, r):
            val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            if val == 0:
                return 0
            return 1 if val > 0 else 2

        # Check if the points are on the same line
        def on_segment(p, q, r):
            return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                    q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))

        # if the points are on the same line, check if they overlap
        def do_intersect(p1, q1, p2, q2):
            o1 = orientation(p1, q1, p2)
            o2 = orientation(p1, q1, q2)
            o3 = orientation(p2, q2, p1)
            o4 = orientation(p2, q2, q1)

            if o1 != o2 and o3 != o4:
                return True

            if o1 == 0 and on_segment(p1, p2, q1):
                return True
            if o2 == 0 and on_segment(p1, q2, q1):
                return True
            if o3 == 0 and on_segment(p2, p1, q2):
                return True
            if o4 == 0 and on_segment(p2, q1, q2):
                return True

            return False

        if do_intersect(p1, q1, p2, q2):
            # Get the point that intersects
            x1, y1 = p1
            x2, y2 = q1
            x3, y3 = p2
            x4, y4 = q2

            # x
            x_intersection = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))

            # y
            y_intersection = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
            return x_intersection, y_intersection

        return None
    def laser_callback(self,msg):
        return msg

    def publish_laserscan(self):
         # Calculate angle increment based on the number of laser beams
        self.scan_msg.angle_increment = (self.laser_angle_max - self.laser_angle_min) / (self.laser_count - 1)
        selected_angles = [self.laser_angle_min + i * self.scan_msg.angle_increment for i in range(self.laser_count)]

        ranges = []


        for angle in selected_angles:
            min_dist=None
            laser_start_x = self.current_x
            laser_start_y = self.current_y
            
            min_dist=self.calculate_points([laser_start_x,laser_start_y],angle)
            if min_dist!=None:
                ranges.append(min_dist)
            else:
                ranges.append(self.laser_range_max)
        self.scan_msg.ranges = ranges
        self.scan_msg.intensities = []
        self.scan_msg.angle_max=self.laser_angle_max
        self.scan_msg.angle_min=self.laser_angle_min
        self.scan_msg.range_max=self.laser_range_max
        self.scan_msg.range_min=self.laser_range_min

        # Calculate time increment and total scan time based on laser rate
        self.scan_msg.time_increment = 1 / self.laser_rate / self.laser_count
        self.scan_msg.scan_time = 1 / self.laser_rate
        self.scan_msg.header.stamp = self.get_clock().now().to_msg()
        self.laserscan_publisher.publish(self.scan_msg)

    
    def collision_builder(self):
        self.collision_flag = False

        # Safety radius around the robot
        safety_distance = self.robot_radius  

        # Make a circle to approximate the robot
        for angle in range(0,360 ,1): 
            radian_angle = math.radians(angle)
          
            # Calculate the point on the circle
            point_x = self.current_x + safety_distance * math.cos(radian_angle)
            point_y = self.current_y + safety_distance * math.sin(radian_angle)
        
            # Iterate over the obstacles
            for obstacle in self.map_coordinate_data.keys():
                obstacle_x1, obstacle_y1, obstacle_x2, obstacle_y2 = obstacle
                obstacle_start = [obstacle_x1, obstacle_y1]
                obstacle_end = [obstacle_x2, obstacle_y2]

                # Find the intersection point between the circle and the obstacle
                intersection_point = self.find_intersection(
                    [self.current_x, self.current_y], [point_x, point_y], obstacle_start, obstacle_end
                )

                if intersection_point is not None:
                    distance_to_obstacle = math.sqrt(
                        (self.current_x - intersection_point[0]) ** 2 + (self.current_y - intersection_point[1]) ** 2
                    )
    
                    # Check if the distance to the obstacle is less than the safety distance
                    if distance_to_obstacle < safety_distance:
                        self.collision_flag = True
                        self.vl=0
                        self.vr=0
                       
                        break
                    denominator = (self.vl+self.vr)/2
                    if denominator == 0:
                            # Handle the case where the denominator is 0
                        time_to_collision = float('inf')  # or any other appropriate default value
                    else:
                        time_to_collision = distance_to_obstacle / denominator
                    if time_to_collision < COLLISION_TIME_THRESHOLD:
                        self.vl = 0
                        self.vr = 0
                        self.collision_flag = True
                        return
                    else:
                        self.collision_flag = False

        # Check if the robot is outside the map
        return self.collision_flag
    def vl_callback(self, msg):
        # Left wheel velocity callback
        self.vl = msg.data
        # If we receive a new data, update the last_velocity_time
        self.last_vel_time = self.get_clock().now()
    def vr_callback(self, msg):
        # Right wheel velocity callback
        self.vr = msg.data
        # If we receive a new data, update the last_velocity_time
        self.last_vel_time = self.get_clock().now()
        
    def update_pose(self):
        #Get the current time to calculate Delta T
        current_time = self.get_clock().now()
        
        pause_duration = 1.0  
                
        self.delta_t = (current_time - self.last_update_time).nanoseconds / 1e9  # Convert to seconds

        # Update the time after the delta T before pause time checking starts
        self.last_update_time = current_time
        
        # Pause if no cmd_vel or vl/vr data received for more than a certain duration
        if (current_time - self.last_vel_time).nanoseconds / 1e9 > pause_duration-0.001:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.current_x
            t.transform.translation.y = self.current_y
            #quaternion
            t.transform.rotation.z = math.sin(self.current_theta / 2)
            t.transform.rotation.w = math.cos(self.current_theta / 2)
            self.tf_broadcaster.sendTransform(t)
            self.time_for_error_update=current_time
            
        # If it didn't pause for over a second
        else:
            #Errors to the vl and vr to apply it on velocity and angular velocity
            if (current_time - self.time_for_error_update).nanoseconds/1e9 >= self.error_update_rate-0.001:
                
                self.vl_err= np.random.normal(1, self.error_variance_left)
                self.vr_err= np.random.normal(1, self.error_variance_right)
                self.time_for_error_update=current_time

            # In order to update the error, we need to multiply it
            current_vl=self.vl*self.vl_err
            current_vr=self.vr*self.vr_err

            
            if self.collision_flag :
                current_vl=0
                current_vr=0
            
            # Calculate linear and angular velocity        
            v = (current_vl + current_vr) / 2.0
            w = (current_vr - current_vl) / self.wheel_distance  # Reference from Differential Drive
            # self.get_logger().info(f'vr, vl = {current_vr}, {current_vl}')
    
            # if (vl-vr) != 0, we use standard differential drive equations
            if current_vr != current_vl:
                R = (self.wheel_distance/2.0) *((current_vr+current_vl)/(current_vr - current_vl))
                c_x = self.current_x - R*math.sin(self.current_theta)
                c_y = self.current_y + R*math.cos(self.current_theta)

                self.current_x = (self.current_x - c_x)*math.cos(w*self.delta_t) - (self.current_y - c_y)*math.sin(w*self.delta_t) + c_x
                self.current_y = (self.current_x - c_x)*math.sin(w*self.delta_t) + (self.current_y - c_y)*math.cos(w*self.delta_t) + c_y
                self.current_theta = self.current_theta + (w*self.delta_t)

            else:
                #if (current_vl-current_vr) = 0, we use modified equation because ICC is at infinity
                # Update position and orientation
                self.current_x += v * math.cos(self.current_theta)*self.delta_t
                self.current_y += v * math.sin(self.current_theta)*self.delta_t
          

############# Before we broadcast, we know the state of the robot
            # Broadcast the transform from world to base_link
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.current_x
            t.transform.translation.y = self.current_y
            t.transform.rotation.z = math.sin(self.current_theta / 2)
            t.transform.rotation.w = math.cos(self.current_theta / 2)
            self.tf_broadcaster.sendTransform(t)


            # Publish the robot pose as PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'world'
            pose_msg.pose.position.x = self.current_x
            pose_msg.pose.position.y = self.current_y
            pose_msg.pose.orientation.z = math.sin(self.current_theta / 2)
            pose_msg.pose.orientation.w = math.cos(self.current_theta / 2)
            self.robot_pose_publisher.publish(pose_msg)


    def construct_map(self):
        for y_index, line in enumerate(reversed(self.robot_map.split('\n'))):
            if line:
                row = [100 if c == '#' else 0 for c in line]
                self.num_rows += 1
                self.num_cols = max(self.num_cols, len(row))
    
        for y_index, line in enumerate(reversed(self.robot_map.split('\n'))):
            if line:
                row = [100 if c == '#' else 0 for c in line]
                self.map_data.extend(row)
                
               
                # Iterate over the elements in the row and calculate x-coordinate
                for x_index, elem in enumerate(row):
                    x_start = int(x_index) * self.resolution
                    y_start = int(y_index - 1) * self.resolution
    
                    # Store coordinates as ranges (x_start, y_start, x_end, y_end)
                    x_end = x_start + self.resolution
                    y_end = y_start + self.resolution
    
                    if elem == 100:
                        # Store the range as a key with the corresponding value
                        self.map_coordinate_data[(x_start, y_start, x_end, y_end)] = elem


    def publish_map(self):
        
        self.map_msg.info.width = self.num_cols
        self.map_msg.info.height = self.num_rows
        self.map_msg.data = [0] * (self.map_msg.info.width * self.map_msg.info.height)
        self.map_msg.data= self.map_data
        self.map_publisher.publish(self.map_msg)

# Change cmd_vel -> vl,vr
class Velocity_translator_node(Node):
    def __init__(self):
        super().__init__('Velocity_transfer_node')
        # subscribe cmd_vel
        self.twist_subscriber = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 100)
        
        # FOr each 1 cmd_vel -> 1 vl + 1 vr
        self.left_publisher = self.create_publisher(Float64, 'vl', 100)
        self.right_publisher = self.create_publisher(Float64, 'vr', 100)
        
        # In order to calculate the vl,vr
        self.declare_parameter('distance', 0.3)
        self.distance_ = self.get_parameter('distance').value

    # only job -> translate linear.x and angular.x
    
    # A callback function for subscribing and publishing.
    def twist_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate wheel velocities based on a differential drive kinematic model
        # You can adjust the following parameters according to your robot's specifications
        base_width = self.distance_

        vr = (2 * linear_x + angular_z * base_width) / 2
        vl = (2 * linear_x - angular_z * base_width) / 2
        
        vr_msg = Float64()
        vr_msg.data = vr
        vl_msg = Float64()
        vl_msg.data = vl

        self.left_publisher.publish(vl_msg)
        self.right_publisher.publish(vr_msg)

#New class for 4D
class Navigate_to_Goal_node(Node):
    def __init__(self):
        
        
        super().__init__('Navigate_to_Goal_node')

        self.declare_parameter('radius', 0.2)
        self.robot_radius = self.get_parameter('radius').value
        self.declare_parameter('resolution', 0.0)
        self.resolution=self.get_parameter('resolution').value
        
        self.goal_pose = PoseStamped()  
        self.robot_pose = PoseStamped()  

        self.map_data = None  # Occupancy grid map
        self.map_coordinate_data = {}

        # Create subscribers
        self.goal_pose_subscription = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.laser_scan_subscription = self.create_subscription(LaserScan, '/scan', self.laser_scan_callback, 10)
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.robot_pose_subscription = self.create_subscription(PoseStamped,'/robot_pose', self.current_pose_callback, 10 )
        
        # Create a publisher for controlling the robot 
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.prev_goal=PoseStamped()
        self.start_new_goal = False
        self.connection_distance = 0.5#min(self.robot_radius, self.resolution)
        self.goal_reached = False
        self.min_dist_from_all_nodes = 0.05
        self.tree_2_reached = False
        self.tree_1_reached = False
        self.path=[]
        self.nodes=[]

        self.timer= self.create_timer(0.1, self.navigate_to_goal)
        self.num_samples = 7000

        self.prev_linear_vel_error = 0.0
        self.prev_angular_vel_error = 0.0
        self.prev_angle = 0.0
        self.id =0

        #visualization of RRT
        self.nodes_list = []
        self.path_list =[]
        self.marker_publisher = self.create_publisher(Marker, 'path_markers', 10)
        self.timer = self.create_timer(0.5, self.publish_markers)


        # #map
        # self.declare_parameter('resolution', 0.0)
        # self.resolution=self.get_parameter('resolution').value
        self.map_timer = self.create_timer(3.0, self.map_processing)
                

        self.get_logger().info('init')


    #callbacks
    def goal_pose_callback(self, msg):
        # Callback for receiving goal poses
        self.get_logger().info(f'goal_callback = {msg}')
        self.goal_pose = msg
        if self.prev_goal is not None:
        # Check if the current goal pose is different from the previous one
            if (
                msg.pose.position.x != self.prev_goal.pose.position.x or
                msg.pose.position.y != self.prev_goal.pose.position.y or
                msg.pose.position.z != self.prev_goal.pose.position.z
            ):
                
                self.get_logger().info('New goal received in callback')
                self.start_new_goal = True
                # time.sleep(1)
        self.prev_goal = self.goal_pose
         
        

    def laser_scan_callback(self, msg):
       
        pass

    def map_callback(self, msg):
        # Callback for receiving map data
        # self.get_logger().info('map_callback')
        self.map_data = msg

    def current_pose_callback(self, msg):
        self.robot_pose =msg
        # self.get_logger().info(f'Robot_current_pose = {msg}')

    def publish_markers(self):
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "world"  # Change the frame_id as per your requirement
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  
        marker.scale.y = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        
        nodes = self.nodes_list    
        path = self.path_list   

        marker.points = [node.pose.position for index, node in enumerate(nodes) if index in path]

        self.marker_publisher.publish(marker)
        # self.get_logger().info('Path Markers Published')

    #Main function for 4d
    def navigate_to_goal(self):
        tree_1 = []
        nodes_1 =[]
        tree_2 = []
        nodes_2 =[]
        updated_goal = copy.deepcopy(self.goal_pose)
        
        if self.start_new_goal:

            self.get_logger().info('NEW CLICKED GOAL STARTED')
            self.start_new_goal = False
            tree_1 = []
            nodes_1 =[]            
            nodes_1 = [copy.deepcopy(self.robot_pose)]
            tree_2= []
            nodes_2 =[]            
            nodes_2 = [copy.deepcopy(self.goal_pose)]

            self.get_logger().info(f'nodes_1 = {nodes_1}')
            self.get_logger().info(f'nodes_2 = {nodes_2}')
            
            goal_pose = copy.deepcopy(self.goal_pose)
            source_pose = copy.deepcopy(self.robot_pose)

            self.get_logger().info(f'new goal_pose = {goal_pose}')
            self.get_logger().info(f'new robot= {source_pose}')

            self.tree_1_reached = False
            self.tree_2_reached = False
            
            for ns in range(self.num_samples):
                # self.get_logger().info(f'sample = {ns}')
                random_point_1 = self.generate_random_point()
                random_point_2 = self.generate_random_point()
                
                nearest_pose_1, nearest_pose_id_1 = self.find_nearest_pose(nodes_1, random_point_1)
                nearest_pose_2, nearest_pose_id_2 = self.find_nearest_pose(nodes_2, random_point_2)

                new_pose_1 = self.find_new_pose(nearest_pose_1, random_point_1)
                new_pose_2 = self.find_new_pose(nearest_pose_2, random_point_2)

                if all(self.calculate_distance(existing_pose, new_pose_1) >= self.min_dist_from_all_nodes for existing_pose in nodes_1):
                    # self.get_logger().info('new node found')
                    # If the condition is satisfied, add new_pose to nodes
                  
                  self.get_logger().info(f'new pose= {new_pose_1.pose.position.x}, {new_pose_1.pose.position.y}')
                #   if not self.collision_builder(new_pose_1.pose.position.x, new_pose_1.pose.position.y):
                  if not self.check_collision(new_pose_1.pose.position.x, new_pose_1.pose.position.y):
                    nodes_1.append(new_pose_1)
                    tree_1.append([-1])
                    # self.get_logger().info(f'new entry added for node= {len(nodes)-1}')
                    # self.get_logger().info(f'nearest_pose id = {nearest_pose_id}')
                    temp_node = tree_1[nearest_pose_id_1]
                    if len(temp_node)==1 and temp_node[0]==-1:
                        # self.get_logger().info(f'existing temp_node = {temp_node}')
                        adj_list = [len(nodes_1)-1]
                        tree_1[nearest_pose_id_1] = adj_list
                    else:
                        adj_list = tree_1[nearest_pose_id_1]
                        # self.get_logger().info(f'existing adj_list = {adj_list}')
                        adj_list.append(len(nodes_1)-1)
                        tree_1[nearest_pose_id_1] = adj_list

                    
                #goal_tree
                if all(self.calculate_distance(existing_pose, new_pose_2) >= self.min_dist_from_all_nodes for existing_pose in nodes_2):
                    # self.get_logger().info('new node found')
                    # If the condition is satisfied, add new_pose to nodes
                #   if not self.collision_builder(new_pose_2.pose.position.x, new_pose_2.pose.position.y):

                  if not self.check_collision(new_pose_2.pose.position.x, new_pose_2.pose.position.y):# and self.map_permitted(new_pose_2.pose.position.x, new_pose_2.pose.position.y):
                    nodes_2.append(new_pose_2)
                    tree_2.append([-1])
                    # self.get_logger().info(f'new entry added for node= {len(nodes)-1}')
                    # self.get_logger().info(f'nearest_pose id = {nearest_pose_id}')
                    temp_node = tree_2[nearest_pose_id_2]
                    if len(temp_node)==1 and temp_node[0]==-1:
                        # self.get_logger().info(f'existing temp_node = {temp_node}')
                        adj_list = [len(nodes_2)-1]
                        tree_2[nearest_pose_id_2] = adj_list
                    else:
                        adj_list = tree_2[nearest_pose_id_2]
                        # self.get_logger().info(f'existing adj_list = {adj_list}')
                        adj_list.append(len(nodes_2)-1)
                        tree_2[nearest_pose_id_2] = adj_list

                   
                #merging trees
                # Check if trees can be merged during exploration
                if not self.tree_1_reached and not self.tree_2_reached:
                    # Find the nearest nodes in tree_1 and tree_2
                    # self.get_logger().info('checking if merge is possible')
                    nearest_node_1, nearest_node_id_1 = self.find_nearest_pose(nodes_1, goal_pose)
                    nearest_node_2, nearest_node_id_2 = self.find_nearest_pose(nodes_2, source_pose)

                    # Check if the distance between the nearest nodes is within the connection distance threshold
                    self.get_logger().info(f'at merge condition dist of trees = {self.calculate_distance(nearest_node_1, nearest_node_2)}')
                    if self.calculate_distance(nearest_node_1, nearest_node_2) < self.connection_distance:
                        self.get_logger().info('Inside merge check')
                        # Extract the path from the root to the nearest nodes in both trees
                        tree_1.append([-1])
                        tree_2.append([-1])
                        self.get_logger().info(f'len_tree1={len(tree_1)}, len_nodes1={len(nodes_1)}')
                        self.get_logger().info(f'len_tree2={len(tree_2)}, len_nodes2={len(nodes_2)}')
                        path_1 = self.extract_path(tree_1, nearest_node_id_1)
                        path_2 = self.extract_path(tree_2, nearest_node_id_2)

                        # Reverse the path in tree_2 and add len(nodes_1) to each index
                        # len_p2 = len(reversed(path_2))
                        path_2_reversed = [len(nodes_1) + node_id for node_id in range(len(path_2))]
                        
                        # Append the reversed path_2 after tree_1
                        path_1.extend(path_2_reversed)
                        self.get_logger().info(f'path_1= {path_1}')
                        # tree_1[nearest_node_id_1].extend(path_2_reversed)

                        # Append nodes from tree_2 to nodes_1 by reversing and appending
                        # self.get_logger().info(f'tree 2 merging indices = {path_2[::-1]}')
                        nodes_1.extend([copy.deepcopy(nodes_2[node_id ]) for node_id in path_2[::-1]])
                        
                        self.get_logger().info('Trees Merged!')
                        self.goal_reached = True
                        self.tree_1_reached = True
                        self.tree_2_reached = False
                        # tree_1.append([-1])
                        # self.get_logger().info(f'length of nodes = {len(nodes)}, length of tree={len(tree)}')
                        updated_goal = tree_1[-1]
                        break

                
        
        # self.get_logger().info(f'tree= {tree}')

        

        if (len(tree_1)>0 or len(tree_2)>0) and self.goal_reached:
            self.goal_reached = False

            
            # path = self.extract_path(tree, updated_goal)
            if len(path_1)>0:
                path = path_1
                nodes = nodes_1

                self.nodes_list = nodes
                self.path_list = path            
                self.publish_markers()

                # self.get_logger().info(nodes)

                self.get_logger().info('printing nodes')
                for i in path:
                    self.get_logger().info(f"i= {i}")
                    self.get_logger().info(f'{nodes[i].pose.position.x}, {nodes[i].pose.position.y}')
                # self.execute_path(path, nodes)
                
                self.id =1

                
                self.path = path
                self.nodes = nodes
                time.sleep(1.0)
                
                self.timer = self.create_timer(2.0, self.execute_path)
        else:
            pass
            # self.get_logger().info('Goal not reached, need more samples')
       
    #get map data as dictionary for easy calculation
    def map_processing (self):
        if self.map_data is None:
            return 
        
        resolution = self.resolution
        # map_coordinate_data = {}

        for y_index in range(self.map_data.info.height):
            for x_index in range(self.map_data.info.width):
                elem = self.map_data.data[y_index * self.map_data.info.width + x_index]
                x_start = x_index * resolution
                y_start = (self.map_data.info.height - y_index - 1) * resolution
                x_end = x_start + resolution
                y_end = y_start + resolution

                if elem == 100:
                    self.map_coordinate_data[(x_start, y_start, x_end, y_end)] = elem

        return self.map_coordinate_data


    #start controller for path execution
    def execute_path(self):
        
        self.get_logger().info(f'id={self.id}')
        path = self.path
        nodes = self.nodes
        if self.id <len(path):

            index = path[self.id]
            target_pose = nodes[index]

            
            angle = math.atan2(target_pose.pose.position.y - self.robot_pose.pose.position.y,
                                        target_pose.pose.position.x - self.robot_pose.pose.position.x)
            
            self.get_logger().info(f'ang={angle}')
            self.get_logger().info(f'prev={self.prev_angle}')
            curr_err = angle - self.prev_angle

            if abs(curr_err) > 0.01:

                if abs(angle - self.prev_angle) > 0.5:
                    #pd
                    kp_ang = 0.9
                    kd_ang = 0.03
                    ang_vel = kp_ang*curr_err 
                    if self.prev_angular_vel_error !=0.0: 
                        ang_vel += kd_ang*(curr_err - self.prev_angular_vel_error)
                    self.publish_cmd_vel(0.0, ang_vel)
                else:
                    self.publish_cmd_vel(0.0, curr_err)          

                time.sleep(1.0)
            else:
                self.publish_cmd_vel(0.0, 0.0)
            self.prev_angle = angle
            self.prev_angular_vel_error = curr_err
            

            linear_velocity= self.pd_controller(target_pose, self.robot_pose)
            # self.get_logger().info(f'v,w={linear_velocity}')

            # Publish velocities
            self.publish_cmd_vel(linear_velocity, 0.0)
            

            # Check if the target pose is reached
            if math.sqrt((target_pose.pose.position.x - self.robot_pose.pose.position.x)**2 +
                        (target_pose.pose.position.y - self.robot_pose.pose.position.y)**2) < 0.1:
                self.get_logger().info('WAYPOINT REACHED')
                self.prev_linear_vel_error = 0.0
                self.id +=1
                # break

        else:
            self.get_logger().info('WAITING FOR NEW GOAL')
  

    #PD Controller for linear vel
    def pd_controller(self, target_pose, current_pose):
        # PD controller gains
        Kp_vel = 0.05#0.08
        Kd_vel = 0.001 #0.05

        delta_t = 1.0
        # self.delta_t = (current_time - self.last_update_time).nanoseconds / 1e9  # Convert to seconds

        self.get_logger().info(f'x2,y2,x1,y1 = {target_pose.pose.position.x}, {target_pose.pose.position.y}, {current_pose.pose.position.x},{current_pose.pose.position.y}')

        linear_error = self.calculate_distance(current_pose, target_pose)

        if self.prev_linear_vel_error  == 0 or self.prev_linear_vel_error >=linear_error:
            if self.prev_linear_vel_error  == 0:
                self.get_logger().info('NEW WAYPOINT TARGETTED')
            
            linear_velocity = Kp_vel * linear_error
            if self.prev_linear_vel_error != 0:
                linear_velocity +=Kd_vel * (linear_error - self.prev_linear_vel_error )/delta_t
                        
            #+ K_d * (0.0 - current_pose.twist.linear.x)
            self.prev_linear_vel_error = linear_error
        else:
            linear_velocity = 0.0
            self.id +=1
            self.prev_linear_vel_error = 0.0
            # pass     
            
            

        return linear_velocity

    #using BFS

    def extract_path(self, tree, updated_goal):
        visited = [False] * len(tree)
        queue = [0]
        parent = [-1] * len(tree)

        while queue:
            current_node = queue.pop(0)
            visited[current_node] = True

            if current_node == updated_goal:
                break

            for child in tree[current_node]:
                if not visited[child]:
                    queue.append(child)
                    parent[child] = current_node

        path_indices = []
        while current_node != -1:
            path_indices.append(current_node)
            current_node = parent[current_node]

        path_indices.reverse()
        self.get_logger().info(f'extracted_path = {path_indices}')

        return path_indices

  
    def check_collision(self, x, y):
        # Helper function to check collision using the map data
        if self.map_data is None:
            return False
        
        # self.get_logger().info(f'Map= {self.map_data}')

        x_map = int(x / self.map_data.info.resolution)
        y_map = int(y / self.map_data.info.resolution)

        # Check if the coordinates are within the map boundaries
        if 0 <= x_map < self.map_data.info.width and 0 <= y_map < self.map_data.info.height:
            # Get the value at the corresponding map cell
            map_value = self.map_data.data[y_map * self.map_data.info.width + x_map]

            # Check for collision (map value of 100 indicates an obstacle)
            if map_value == 100:
                return True  # Collision detected
            else:
                #create a circle of radius of the robot
                for ang in range(0,360):
                    
                    # point_x = int(x / self.map_data.info.resolution)
                    # point_y = int(y / self.map_data.info.resolution)
                    x_boundary = x + (1.2)*self.robot_radius * math.cos(ang)
                    y_boundary = y + (1.2)*self.robot_radius * math.sin(ang)
                    
                    x_map_b = int(x_boundary/ self.map_data.info.resolution)
                    y_map_b = int(y_boundary / self.map_data.info.resolution)

                    # self.get_logger().info(f'x= {x}')
                    # self.get_logger().info(f'y= {y}')
                    # self.get_logger().info(f'x_map_b= {x_map_b}')
                    # self.get_logger().info(f'y_map_b= {y_map_b}')
                    # self.get_logger().info(f'x_boundary= {x_boundary}')
                    # self.get_logger().info(f'y_boundary= {y_boundary}')

                    # time.sleep(1)

                    
                #check any point in collision
                    if 0 <= x_map_b < self.map_data.info.width and 0 <= x_map_b < self.map_data.info.height:
                    # Get the value at the corresponding map cell
                        map_point = self.map_data.data[y_map_b * self.map_data.info.width + x_map_b]
                        #if not false
                        # self.get_logger().info(f'map_point= {map_point}')
                        if map_point == 100:
                            return True  # Collision detected
                    
                        
                
                #if yes true
                return False  # No collision
        else:
            return True 
        
    def find_new_pose(self, nearest_pose, random_point):
        # Helper function to extend the tree towards the random point
        distance = self.calculate_distance(nearest_pose, random_point)

        if distance < self.connection_distance:
            return random_point
        else:
            angle = math.atan2(random_point.pose.position.y - nearest_pose.pose.position.y,
                       random_point.pose.position.x - nearest_pose.pose.position.x)

            # Calculate the slope (m) and intercept (c) of the line
            m = math.tan(angle)
            c = nearest_pose.pose.position.y - m * nearest_pose.pose.position.x

            # Calculate the coordinates of the new point at self.connection_distance along the line
            new_x = nearest_pose.pose.position.x + self.connection_distance / math.sqrt(1 + m**2)
            new_y = m * new_x + c

            # Create a new PoseStamped message for the calculated point
            new_pose = PoseStamped()
            new_pose.header = nearest_pose.header
            new_pose.pose.position.x = new_x
            new_pose.pose.position.y = new_y
            new_pose.pose.orientation = nearest_pose.pose.orientation

            return new_pose

    def calculate_distance(self, pose1, pose2):
        # Helper function to calculate distance between two poses
        x1, y1 = pose1.pose.position.x, pose1.pose.position.y
        x2, y2 = pose2.pose.position.x, pose2.pose.position.y
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def find_nearest_pose(self, nodes, random_point):
        # Helper function to find the nearest pose in the tree to the random point
        distances = [self.calculate_distance(pose, random_point) for pose in nodes]
        nearest_pose = nodes[np.argmin(distances)]
        nearest_pose_id = np.argmin(distances)
        return nearest_pose,nearest_pose_id

    def generate_random_point(self):
        # Helper function to generate a random point in the free space
        if self.map_data is None:
            return PoseStamped()
        map_width = self.map_data.info.width
        map_height = self.map_data.info.height
        while True:

            x,y = np.random.rand(2)*(self.map_data.info.width, self.map_data.info.height)
            
            # self.get_logger().info(f'random samples= {x}, {y}')         
            point = PoseStamped()
            point.pose.position.x = x
            point.pose.position.y = y
            return point

    
    def publish_cmd_vel(self, linear_velocity, angular_velocity):
        # Helper function to publish velocities to /cmd_vel
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            twist_msg.angular.z = angular_velocity
            self.get_logger().info(f'Publishing(v,w)={linear_velocity}, {angular_velocity}')
            self.cmd_vel_publisher.publish(twist_msg)



   
def main_sim(args=None):
    rclpy.init(args=args)
    print('node_1 start')
    node = Simulator_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main_vel(args=None):
    rclpy.init(args=args)
    print('node_2 start')
    node_2=Velocity_translator_node()
    rclpy.spin(node_2)
    node_2.destroy_node()
    rclpy.shutdown()

def main_navigate(args=None):
    rclpy.init(args=args)
    print('node_3 start')
    node_3=Navigate_to_Goal_node()
    rclpy.spin(node_3)
    node_3.destroy_node()
    rclpy.shutdown()
