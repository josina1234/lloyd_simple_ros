#!/usr/bin/env python3
import math
import time
from enum import Enum, auto

import numpy as np
import rclpy
from geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
)
from hippo_msgs.msg import Float64Stamped, BoolStamped

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
from scenario_msgs.srv import MoveToStart

from lloyd_simple.scripts.barriers import barriers
from lloyd_simple.scripts.lloyd_path import Lloyd, applyrules
import copy

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time


class State(Enum):
    UNSET = auto()
    INIT = auto()
    IDLE = auto()
    MOVE_TO_START = auto()
    NORMAL_OPERATION = auto()


class LloydPathPlanner(Node):

    def __init__(self):
        super().__init__(node_name='lloyd_path_planner')
        self.state = State.UNSET
        self.recomputation_required = True
        self.target_viewpoint_index = -1  # TODO ggf unnötig
        self.path_marker: Marker  # TODO ggf unnötig
        self.init_path_marker()  # TODO ggf unnötig
        self.path_marker_pub = self.create_publisher(Marker, '~/marker',
                                                     1)  # TODO ggf unnötig
        
        # Initialize weighted centroids marker
        self.weighted_centroids_marker: Marker
        self.init_weighted_centroids_marker()
        self.weighted_centroids_pub = self.create_publisher(Marker, '~/weighted_centroids', 1)
        
        self.setpoints = [] # zum Speichern der c1

        self.progress = -1.0

        # Add flag to track if we have received pose data
        self.pose_received = False
        self.at_start_position = False

        self.compute_full_path = False  # TODO ggf unnötig
        self.init_params()  # bereits mit eigenen Parametern ersetzt

        self.init_services()
        self.init_robot_subscribers()  # Neue Methode für Multi-Robot Subscriber


        start_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # subscriber für startzeit topic
        mission_ns = 'mission'  # oder aus parameter/configuration lesen
        self.start_sub = self.create_subscription(
            Time, f'/{mission_ns}/start', self.on_start_time, start_qos
        ) # leading / vor mission_ns wichtig, da sonst relativ zum eigenen namespace gesucht wird
        self.get_logger().info(f'{self.own_namespace}: waiting for start time on /{mission_ns}/start')


        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                 'vision_pose_cov',
                                                 self.on_pose, 1)

        # Add setpoint publisher for position control
        self.setpoint_pub = self.create_publisher(msg_type=PointStamped, topic='setpoint_position', qos_profile=1)
        
        # Create timer for Lloyd algorithm execution at 50Hz
        self.lloyd_timer = self.create_timer(0.02, self.lloyd_timer_callback)  # 50Hz = 0.02s

        # Create publisher for temp_goal
        self.temp_goal_pub = self.create_publisher(msg_type=PointStamped, topic='~/temp_goal_position', qos_profile=1)

        # Create Publisher for beta value double value
        self.beta_pub = self.create_publisher(msg_type=Float64Stamped, topic='~/beta_value', qos_profile=1)

        #Create Publisher for final_goal
        self.final_goal_pub = self.create_publisher(msg_type=PointStamped, topic='~/final_goal_position', qos_profile=1)
        
        # create Publisher for initial POsition
        self.init_position_pub = self.create_publisher(msg_type=PointStamped, topic='~/initial_position', qos_profile=1)

        # Create Publisher for theta value
        self.theta_pub = self.create_publisher(msg_type=Float64Stamped, topic='~/theta_value', qos_profile=1)

        # Create publisher für at goal
        self.mission_active_pub = self.create_publisher(msg_type=BoolStamped, topic='~/mission_active', qos_profile=1)

        # create Publisher for minimum distance to barriers
        self.min_dist_pub = self.create_publisher(msg_type=Float64Stamped, topic='~/min_distance_to_barriers', qos_profile=1)

        # create publisher for size of cell - for debugging and evaluation
        self.cell_size_pub = self.create_publisher(msg_type=Float64Stamped, topic='~/cell_size', qos_profile=1)

        self.ready_pub = self.create_publisher(BoolStamped, 'ready', 1) # bluerovxy/ready topic

        # Don't call move_to_start() immediately - wait for pose data
        # self.move_to_start()  # Remove this line

        # Create timer to check when to start moving to start position
        self.start_timer = self.create_timer(0.1, self.check_start_conditions)
        
        self.initialize_lloyd_algorithm() # erst wenn alle roboter am start sind
        #

    def check_start_conditions(self):
        """Check if conditions are met to start moving to start position"""
        if self.pose_received and not self.at_start_position:
            self.start_timer.destroy()  # Stop the timer
            self.move_to_start()

    def move_to_start(self):
        """Move robot to its start position using a timer-based approach"""
        self.state = State.MOVE_TO_START
        self.get_logger().info(f'{self.own_namespace}: starting move to start position {self.init_position}')
        
        # Create timer for moving to start position
        self.move_timer = self.create_timer(0.1, self.move_to_start_callback)

    def move_to_start_callback(self):
        """Timer callback for moving to start position"""
        if not hasattr(self, 'current_pose'):
            return
            
        # Publish setpoint to move to start position
        self.publish_setpoint(np.array(self.init_position))
        
        # Check distance to start position
        current_pos = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y
        ])
        dist_to_start = np.linalg.norm(self.init_position - current_pos)
        
        if dist_to_start <= 0.1:
            if not hasattr(self, 'start_reached_time'):
                # First time reaching start position
                self.start_reached_time = self.get_clock().now()
                self.get_logger().info(f'{self.own_namespace}: reached start position, stabilizing...')
            else:
                # Check if we've been at start position for 5 seconds
                elapsed_time = (self.get_clock().now() - self.start_reached_time).nanoseconds / 1e9
                if elapsed_time >= 5.0:
                    # Stop the timer and publish ready
                    self.move_timer.destroy()
                    self.at_start_position = True
                    
                    # Publish ready message
                    ready_msg = BoolStamped()
                    ready_msg.header.stamp = self.get_clock().now().to_msg()
                    ready_msg.data = True
                    self.ready_pub.publish(ready_msg)
                    
                    self.get_logger().info(f'{self.own_namespace}: stabilized at start position, ready!')
                    self.state = State.IDLE  # Changed from NORMAL_OPERATION to IDLE
        else:
            # Reset start time if we moved away from start position
            if hasattr(self, 'start_reached_time'):
                delattr(self, 'start_reached_time')

    def on_start_time(self, msg: Time):
        """Callback for receiving start time"""
        self.start_time_msg = msg
        self.get_logger().info(f'{self.own_namespace}: received start time: {msg.sec}.{msg.nanosec:09d}')
        self.wait_until_start_and_go()

    def wait_until_start_and_go(self):
        """Wait until the specified start time and then start the Lloyd algorithm"""
        if self.start_time_msg is None:
            return  # No start time received yet

        # Calculate start time in seconds
        start_time_sec = self.start_time_msg.sec + self.start_time_msg.nanosec * 1e-9
        current_time = self.get_clock().now().seconds_nanoseconds()
        current_time_sec = current_time[0] + current_time[1] * 1e-9

        wait_duration = start_time_sec - current_time_sec
        if wait_duration > 0:
            self.get_logger().info(f'{self.own_namespace}: waiting for {wait_duration:.3f} seconds to start...')
            time.sleep(wait_duration)

        self.get_logger().info(f'{self.own_namespace}: starting Lloyd algorithm now.')
        self.ready = True  # Set ready flag to True
        self.state = State.NORMAL_OPERATION

    def init_params(self):
        self.own_namespace = self.get_namespace().strip(
            '/')  # eigener Namespace ohne '/'
        self.declare_parameters(
            namespace='',
            # folgende Parameter werden aus yaml-Datei geladen
            parameters=[
                ('pool_limits_x',
                 rclpy.Parameter.Type.DOUBLE_ARRAY),  # aus barrers yaml
                ('pool_limits_y',
                 rclpy.Parameter.Type.DOUBLE_ARRAY),  # aus barreirs yaml
                ('cell_resolution',
                 rclpy.Parameter.Type.DOUBLE),  # aus barriers yaml
                ('obstacle_limits_x',
                 rclpy.Parameter.Type.DOUBLE_ARRAY),  # aus barriers yaml
                ('obstacle_limits_y',
                 rclpy.Parameter.Type.DOUBLE_ARRAY),  # aus barriers yaml
                ('radius', rclpy.Parameter.Type.DOUBLE),  # sensing radius
                ('size', rclpy.Parameter.Type.DOUBLE),  # robot size
                ('d1', rclpy.Parameter.Type.DOUBLE),
                ('d2', rclpy.Parameter.Type.DOUBLE),
                ('betaD', rclpy.Parameter.Type.DOUBLE),
                ('beta_min', rclpy.Parameter.Type.DOUBLE),
                ('encumbrance_barriers', rclpy.Parameter.Type.DOUBLE),
                ('waiting_time', rclpy.Parameter.Type.INTEGER),
                ('init_pos', rclpy.Parameter.Type.DOUBLE_ARRAY),  # Startposition
                ('goal_pos', rclpy.Parameter.Type.DOUBLE_ARRAY),  # Zielposition
                ('bluerov_names', rclpy.Parameter.Type.STRING_ARRAY),
                ('num_vehicles', rclpy.Parameter.Type.INTEGER),
            ],
            # alle Parameter, die für die Lloyd-Initialisierung benötigt werden
        )

        # parameterinitialieriung wie in bluerov_simulation in python code
        self.max_velocity = 0  # tracks max vel for reporting
        # N, Number of robots wird nicht direkt als Parameter genutzt, da nun komplett dezentralisiert also keine For schleife
        # N wird ohnehin beim Launchen durch Anzahl der 'vehicle_names' definiert
        self.c1 = np.zeros((2, 1))  # initial centroid positions - Fixed dimensions
        self.c2 = np.zeros((2, 1))  # initial centroid positions without neighbours - Fixed dimensions
        self.c1_no_rotation = np.zeros((2, 1))  # initial centroid positions without rotation rule - Fixed dimensions
        # self.c2_no_rotation = np.zeros((2, 1))  # initial centroid positions without neighbours and rotation rule #TODO ggf unnötig - Fixed dimensions
        self.flag = 0  # ACHTUNG ggf Fehler wegen Wechsel auf Integer vs np.array
        # Initialization of the flag that indicates if the robot i is in its goal region
        self.flag_convergence = 0  # Initialization of the flag that indicates if all the robots have entered their goal regions
        self.theta = 0.0  # ACHTUNG ggf Fehler wegen Wechsel auf Integer vs np.array
        # initialization of current orientations
        self.current_position = None  # TODO
        self.Lloyd = None
        self.beta = self.get_parameter('betaD').value  # Remove .copy() as it's a scalar
        self.step = 0
        #####################################################

        self.pool_limits_x = self.get_parameter('pool_limits_x').value
        self.pool_limits_y = self.get_parameter('pool_limits_y').value
        self.cell_resolution = self.get_parameter('cell_resolution').value
        self.obstacle_limits_x = self.get_parameter('obstacle_limits_x').value
        self.obstacle_limits_y = self.get_parameter('obstacle_limits_y').value
        self.radius = self.get_parameter('radius').value
        self.get_logger().info(f'Sensing radius set to: {self.radius} m')
        self.size = self.get_parameter('size').value
        self.d1 = self.get_parameter('d1').value
        self.d2 = self.get_parameter('d2').value
        self.betaD = self.get_parameter('betaD').value
        self.beta_min = self.get_parameter('beta_min').value
        self.encumbrance_barriers = self.get_parameter(
            'encumbrance_barriers').value
        self.waiting_time = self.get_parameter('waiting_time').value
        self.init_position = self.get_parameter(
            'init_pos').value  # double mit x und y position
        self.goal_position = self.get_parameter(
            'goal_pos').value  # double mit x und y position
        # Convert goal_position to numpy array for Lloyd algorithm
        self.goal_position = np.array(self.goal_position)
        
        self.current_goal_position = None  
        ##
        self.num_vehicles = self.get_parameter(
            'num_vehicles').value  # Anzahl der Fahrzeuge im Netzwerk
        self.bluerov_names = self.get_parameter(
            'bluerov_names').value  # Liste der Roboternamen
        self.neighbour_positions = np.zeros(
            (self.num_vehicles - 1,
             2))  # speichert Positionen der Nachbarroboter
        # Dictionary für alle Roboterpositionen initialisieren
        self.robot_poses = {}


        ###########################################################

        Barriers = barriers(self.cell_resolution, self.pool_limits_x,
                            self.pool_limits_y, self.obstacle_limits_x,
                            self.obstacle_limits_y)
        self.all_barriers = Barriers.def_barriers()
        self.basin_limits, self.obstacle_limits = Barriers.get_limits()
        ################################################################

        self.start_time_msg = None  # to store received start time message

        self.ready = False  # indicates if ready to start Lloyd algorithm

    def initialize_lloyd_algorithm(self):
        """Lloyd-Algorithmus Initialisierung"""
        
        self.Lloyd = None
        self.final_goal = copy.deepcopy(self.goal_position)  # Zwischenspeicher

        # self.dt = 0.02  # 50Hz timer period instead of 0.25

        #erstellung des lloyd-objektes - Fix initialization
        self.Lloyd = Lloyd(self.radius, self.size, self.cell_resolution, 
                          self.encumbrance_barriers, self.all_barriers, 
                          self.basin_limits, self.obstacle_limits)

    def init_services(self):
        self.start_service = self.create_service(
            Trigger, '~/start', self.serve_start)  # trigger beibehalten? TODO
        self.stop_service = self.create_service(Trigger, '~/stop',
                                                self.serve_stop)

    def init_robot_subscribers(self):
        """Erstellt Subscriber für alle anderen Roboter im Netzwerk"""
        self.robot_pose_subscribers = {}

        # for schleife soll nur num_vehicles viele strings in self.bluerov_names berücksichtigen
        for robot_name in self.bluerov_names[:self.num_vehicles]:
            if robot_name == self.own_namespace:
                # Eigenen Subscriber überspringen
                continue

            # Topic-Name für anderen Roboter
            topic_name = f'/{robot_name}/vision_pose_cov'

            # Callback-Funktion mit Lambda für jeweiligen Roboter
            callback = lambda msg, name=robot_name: self.on_other_robot_pose(
                msg, name)

            # Subscriber erstellen
            subscriber = self.create_subscription(PoseWithCovarianceStamped,
                                                  topic_name, callback, 1)

            self.robot_pose_subscribers[robot_name] = subscriber
            self.get_logger().info(f'Subscribed to {topic_name}')

    def on_pose(self, msg: PoseWithCovarianceStamped):
        """Callback für eigene Position"""
        self.current_pose = msg.pose.pose
        self.pose_received = True  # Mark that we've received pose data
        # Eigene Position in Dictionary speichern
        self.robot_poses[self.own_namespace] = msg.pose.pose  # aktualisiert die eigene Position im Dictionary
        # Nach Update der eigenen Position Nachbarn neu berechnen
        self.update_neighbour_positions()

    def on_other_robot_pose(self, msg: PoseWithCovarianceStamped,
                            robot_name: str):
        """Callback für Positionen anderer Roboter"""
        self.robot_poses[robot_name] = msg.pose.pose
        self.get_logger().debug(f'Received pose from {robot_name}',
                                throttle_duration_sec=1.0)
        # Nach Update der Nachbarpositionen Nachbarn neu berechnen
        self.update_neighbour_positions()

    def update_neighbour_positions(self):
        """Aktualisiert kontinuierlich die Liste der Nachbarpositionen"""
        if self.own_namespace not in self.robot_poses:
            self.neighbour_positions = []
            return

        own_pos = self.robot_poses[self.own_namespace].position
        neighbours = []

        for robot_name, pose in self.robot_poses.items():
            if robot_name == self.own_namespace:
                continue

            # Distanz berechnen
            dx = pose.position.x - own_pos.x
            dy = pose.position.y - own_pos.y
            distance = np.sqrt(dx**2 + dy**2)

            # Nur Nachbarn innerhalb des Sensorradius hinzufügen
            if distance <= self.radius:
                neighbours.append([pose.position.x, pose.position.y])

        self.neighbour_positions = np.array(neighbours)
        self.get_logger().debug(
            f'Updated neighbours: {len(self.neighbour_positions)} robots in range',
            throttle_duration_sec=1.0)

    def get_current_neighbour_positions(self):
        """Gibt die aktuellen Nachbarpositionen zurück"""
        return self.neighbour_positions.copy()

    def get_all_robot_positions(self):
        """Gibt alle bekannten Roboterpositionen zurück"""
        positions = {}
        for robot_name, pose in self.robot_poses.items():
            positions[robot_name] = [pose.position.x, pose.position.y]
        return positions

    def lloyd_timer_callback(self):
        """Timer callback für Lloyd-Algorithmus Ausführung bei 50Hz"""
        if self.state != State.NORMAL_OPERATION:
            return
            
        if self.own_namespace not in self.robot_poses:
            return
            
        if not self.ready:
            return
        
        # Check if goal is reached
        if self.handle_mission_completed():
            self.publish_mission_active(False)
        else:
            self.publish_mission_active(True)
            # return    
        self.Lloyd_iteration()

    def Lloyd_iteration(self):
        """Führt eine Iteration des Lloyd-Algorithmus durch"""
        if self.own_namespace not in self.robot_poses:
            self.get_logger().warn('Own position not available yet')
            return
        
        self.now=self.get_clock().now()

        self.dt = (self.now - self.last_time).nanoseconds * 1e-9 if hasattr(self, 'last_time') else 0.02

        current_position = [
            self.robot_poses[self.own_namespace].position.x,
            self.robot_poses[self.own_namespace].position.y
        ]
        current_position = np.array(current_position) 

        # Aktuelle Nachbarpositionen abrufen
        neighbours = self.get_current_neighbour_positions()

        # Lloyd-Algorithmus mit aktuellen Daten ausführen
        self.Lloyd.aggregate(
            neighbours, self.beta, self.goal_position,
            self.final_goal, self.dt, current_position)

        # centroids
        self.c1, self.c2, self.c1_no_rotation = self.Lloyd.get_centroid()

        # Apply rules to update beta, theta, and goal_position
        self.goal_position, self.beta, self.theta = applyrules(self.d1, self.d2, self.dt, self.betaD, self.beta_min,
                   self.beta, current_position, self.c1, self.c2, self.theta,
                   self.final_goal, self.c1_no_rotation, self.goal_position)

        # Publish setpoint for position controller
        self.publish_setpoint(self.c1)

        # Publish self.beta for debugging
        self.publish_beta()
        
        # Publish self.goal_position for debugging
        self.publish_temp_goal()

        # publish self.final_goal for debugging
        self.publish_final_goal()

        # publish self.init_position for debugging
        self.publish_initial_position()

        # publish self.theta for debugging
        self.publish_theta()

        # publish min distance to barriers
        self.publish_min_dist()

        # publish cell size for debugging and evaluation
        self.publish_cellsize()

        # # Lloyd-Algorithmus mit aktuellen Daten ausführen - Debug output
        # self.get_logger().debug(f'Current position: {current_position.flatten()}')
        # self.get_logger().debug(f'Current neighbours: {len(neighbours)} robots')
        # self.get_logger().debug(f'Goal position: {self.goal_position.flatten()}')
        # self.get_logger().debug(f'Setpoint: {self.c1.flatten()}')
        self.last_time = self.now

    def publish_setpoint(self, point):
        """Publish setpoint for position controller based on Lloyd centroid"""
        if self.c1 is None or not hasattr(self, 'current_pose'):
            return
         # gewünschter message type: PointStanmed
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # make z a constant altitude at -0.5 meters
        # Convert centroid to position
        msg.point.x = float(point[0])
        msg.point.y = float(point[1])
        msg.point.z = -0.5  # Constant altitude at -0.5 meters
        
        self.setpoint_pub.publish(msg)
        
        # Store setpoint for debugging
        self.setpoints.append([float(point[0]), float(point[1])])
        
        # Publish weighted centroids marker
        self.publish_weighted_centroids_marker()

    def publish_beta(self):
        """Publish beta value for debugging"""
        msg = Float64Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = float(self.beta)
        self.beta_pub.publish(msg)

    def publish_temp_goal(self):
        """Publish temporary goal position for debugging"""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = float(self.goal_position[0])
        msg.point.y = float(self.goal_position[1])
        msg.point.z = -0.5  # Constant altitude
        self.temp_goal_pub.publish(msg)

    def publish_final_goal(self):
        """Publish final goal position for debugging"""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = float(self.final_goal[0])
        msg.point.y = float(self.final_goal[1])
        msg.point.z = -0.5  # Constant altitude
        self.final_goal_pub.publish(msg)

    def publish_initial_position(self):
        """Publish initial position for debugging"""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = float(self.init_position[0])
        msg.point.y = float(self.init_position[1])
        msg.point.z = -0.5  # Constant altitude
        self.init_position_pub.publish(msg)

    def publish_theta(self):
        """Publish theta value for debugging"""
        msg = Float64Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = float(self.theta)
        self.theta_pub.publish(msg)

    def publish_min_dist(self):
        """Publish minimum distance to barriers for debugging"""
        min_dist = self.Lloyd.get_minimum_distance_to_barriers()
        msg = Float64Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = float(min_dist)
        self.min_dist_pub.publish(msg)

    def publish_cellsize(self):
        """Publish cell size for debugging and evaluation"""
        cell_size = self.Lloyd.get_cell_size()
        msg = Float64Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = float(cell_size)
        self.cell_size_pub.publish(msg)

    def serve_start(self, request, response):
        if self.state != State.NORMAL_OPERATION:
            self.get_logger().info('Starting Lloyd algorithm normal operation.')
            self.get_logger().info(f'Robot goal: {self.goal_position}')
            self.reset_internals()
        self.state = State.NORMAL_OPERATION
        response.success = True
        return response

    def serve_stop(self, request, response):
        if self.state != State.IDLE:
            self.get_logger().info('Asked to stop. Going to idle mode.')
        self.state = State.IDLE
        response.success = True
        return response

    # Simplify do_stop method - no path follower client needed
    def do_stop(self):
        self.state = State.IDLE
        self.reset_internals()
        return True

    def handle_mission_completed(self):
        """Check if robot reached its individual goal"""
        if not hasattr(self, 'current_pose') or not hasattr(self, 'goal_position'):
            return False
            
        current_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        goal_pos = np.array([self.goal_position[0], self.goal_position[1]])
        distance_to_goal = np.linalg.norm(current_pos - goal_pos)
        
        # Check if close enough to goal (within robot size)
        if distance_to_goal < self.size:
            # self.do_stop()
            return True
        return False
    
    def publish_mission_active(self, boolean):
        """Publish a message indicating the robot has reached its goal"""
        msg = BoolStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = boolean
        self.mission_active_pub.publish(msg)
        # Here you can implement any additional logic needed when the robot reaches its goal

    def reset_internals(self):
        self.target_viewpoint_index = -1
        self.recomputation_required = True
        # Clear setpoints when resetting
        self.setpoints.clear()

    def init_path_marker(self):
        msg = Marker()
        msg.action = Marker.ADD
        msg.ns = 'path'
        msg.id = 0
        msg.type = Marker.LINE_STRIP
        msg.header.frame_id = 'map'
        msg.color.a = 1.0
        msg.color.r = 0.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        msg.scale.x = 0.02
        msg.scale.y = 0.02
        msg.scale.z = 0.02
        self.path_marker = msg

    def publish_path_marker(self, segments):
        msg = self.path_marker
        world_points = self.segments_to_world_points(segments)
        msg.points = [Point(x=p[0], y=p[1], z=-0.5) for p in world_points]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.path_marker_pub.publish(msg)

    def init_weighted_centroids_marker(self):
        msg = Marker()
        msg.action = Marker.ADD
        msg.ns = 'weighted_centroids'
        msg.id = 0
        msg.type = Marker.LINE_STRIP
        msg.header.frame_id = 'map'
        msg.color.a = 1.0
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 1.0
        msg.scale.x = 0.02
        msg.scale.y = 0.02
        msg.scale.z = 0.02
        self.weighted_centroids_marker = msg

    def publish_weighted_centroids_marker(self):
        """Publish weighted centroids as a line strip marker"""
        msg = self.weighted_centroids_marker
        msg.points = [Point(x=p[0], y=p[1], z=-0.5) for p in self.setpoints]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.weighted_centroids_pub.publish(msg)


def main():
    rclpy.init()



    node = LloydPathPlanner()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    try:
        exec.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
