#!/usr/bin/env python3
import math
import time
from enum import Enum, auto
from itertools import permutations

import numpy as np
import rclpy
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
)
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scenario_msgs.msg import Viewpoint, Viewpoints
from scenario_msgs.srv import MoveToStart, SetPath
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

from final_project_solution import search


class Lloyd:

    def __init__(self, robot_i_position, radius, encumbrance_i,
                 encumbrance_neighbours, k_p_i, cell_resolution, dt,
                 encumbrance_barriers, v_max):  #ggf noch v_max TODO
        # initialisieren des Klassenobjektes des i-ten Roboters
        self.robot_position = robot_i_position  # zunächst Startpositionen
        self.radius = radius  # same for all robots
        self.encumbrance = encumbrance_i
        self.encumbrance_neighbours_unfiltered = encumbrance_neighbours
        self.k_p = k_p_i
        self.cell_resolution = cell_resolution
        self.dt = dt
        self.encumbrance_barriers_float = encumbrance_barriers
        obstacle_limits = None #pass in Node Init
        pool_limits = None #pass in Node Init

        # ggf noch v_max TODO
        self.v_max = v_max
        Barriers = barriers(cell_resolution, obstacle_limits, pool_limits)
        self.barriers_unfiltered = Barriers.def_barriers()  # TODO entfernen
        self.basin_limits, self.obstacle_limits = Barriers.get_limits()

    # wird in jeder Iteration aufgerufen und deklariert die Nachbarpositionen neu
    def aggregate(self, neighbour_positions, beta_i, goal_position_i):
        self.neighbour_positions = neighbour_positions
        self.beta = beta_i
        self.goal_position = goal_position_i
        self.filter_neighbours()
        self.filter_barriers()

    def filter_neighbours(self):
        # Nachbarn filtern, die innerhalb eines bestimmten Abstands sind (2*r) maximaler Sensing radius
        # ggf hier noch mal sicherstellen. dass neighbour_positions und robot_position ein numpy array sind
        if not isinstance(self.neighbour_positions, np.ndarray):
            self.neighbour_positions = np.array(self.neighbour_positions)
        if not isinstance(self.robot_position, np.ndarray):
            self.robot_position = np.array(self.robot_position)

        dist = np.linalg.norm(self.neighbour_positions - self.robot_position,
                              axis=1)
        valid_indices = np.where(dist <= 2 * self.radius)[0]

        self.neighbour_positions = self.neighbour_positions[
            valid_indices].tolist()
        self.encumbrance_neighbours = [
            self.encumbrance_neighbours_unfiltered[i] for i in valid_indices
        ]

    def filter_barriers(self):
        # Hindernisse filtern, die innerhalb eines bestimmten Abstands sind 1*r
        # ggf hier noch mal sicherstellen. dass barriers und robot_position ein numpy array sind
        if not isinstance(self.barriers_unfiltered, np.ndarray):
            self.barriers_unfiltered = np.array(self.barriers_unfiltered)
        if not isinstance(self.robot_position, np.ndarray):
            self.robot_position = np.array(self.robot_position)

        dist = np.linalg.norm(self.barriers_unfiltered - self.robot_position,
                              axis=1)
        valid_indices = np.where(dist
                                 <= (self.radius))  # radius = sensing radius

        self.barrier_positions = self.barriers_unfiltered[
            valid_indices].tolist()
        self.encumbrance_barriers = [
            self.encumbrance_barriers_float
            for i in range(len(self.barrier_positions))
        ]  # prüfen ob gewüschte form herauskommt

    def get_centroid(self):
        # neue Centroiden berechnen
        circle_points = self.get_circle_points()  # tupel-liste

        # filter out points too close to barriers and all points behind barriers
        circle_points = self.consider_barriers(circle_points)

        circle_points = self.find_closest_points_(circle_points,
                                                  self.barrier_positions)

        if len(self.neighbour_positions) > 0:
            # compute voronoi cell

            # Case: delta_ij <= (||p_i - p_j||)/2
            # cell_points are all q within circle with ||q-p_i|| < ||q-p_j|| for all neighbours j
            cell_points = self.find_closest_points_(
                circle_points, self.neighbour_positions)  # tupel-list
            # filter cell points considering encumbrances
            # Case: delta_ij > (||p_i - p_j||)/2
            #  ||q-p_i|| < ||q-p_j~|| for all neighbours j
            cell_points_filtered = self.consider_encumbrances(cell_points)
            # if no points left in cell_points_filtered, return current position as centroid
            if len(cell_points_filtered) == 0:
                cell_points_filtered = [self.robot_position]

            x_cell, y_cell = zip(
                *cell_points_filtered
            )  # unzip tupel-list (* is unpacking operator)

        else:
            # circle_points are voronoi cell
            x_cell, y_cell = zip(
                *circle_points)  # unzip tupel-list (* is unpacking operator)

        x_cell_no_neigh, y_cell_no_neigh = zip(
            *circle_points)  # unzip tupel-list (* is unpacking operator)

        # compute scalar values for weighted centroid
        scalar_values = self.compute_scalar_values(x_cell, y_cell)
        scalar_values_no_neigh = self.compute_scalar_values(
            x_cell_no_neigh, y_cell_no_neigh)

        # make sure everything is numpy array
        x_cell = np.array(x_cell)
        y_cell = np.array(y_cell)
        scalar_values = np.array(scalar_values)
        x_cell_no_neigh = np.array(x_cell_no_neigh)
        y_cell_no_neigh = np.array(y_cell_no_neigh)
        scalar_values_no_neigh = np.array(scalar_values_no_neigh)

        # compute weighted centroids

        c1_x = np.sum(x_cell * scalar_values) / np.sum(scalar_values)
        c1_y = np.sum(y_cell * scalar_values) / np.sum(scalar_values)
        c2_x = np.sum(x_cell_no_neigh *
                      scalar_values_no_neigh) / np.sum(scalar_values_no_neigh)
        c2_y = np.sum(y_cell_no_neigh *
                      scalar_values_no_neigh) / np.sum(scalar_values_no_neigh)

        c1 = np.array([c1_x, c1_y])  # nieghbours
        c2 = np.array([c2_x, c2_y])  # no neighbours

        return c1, c2

    def get_circle_points(self):
        x_center, y_center = self.robot_position

        x_min = x_center - self.radius
        x_max = x_center + self.radius
        y_min = y_center - self.radius
        y_max = y_center + self.radius

        x_coords = np.round(np.arange(x_min, x_max + self.cell_resolution,
                                      self.cell_resolution),
                            decimals=3)  # to avoid floating point issues
        y_coords = np.round(np.arange(y_min, y_max + self.cell_resolution,
                                      self.cell_resolution),
                            decimals=3)

        x, y = np.meshgrid(x_coords, y_coords)

        distances = np.sqrt((x - x_center)**2 + (y - y_center)**2)
        valid_indices = np.where(distances <= self.radius)

        circle_points = list(zip(x[valid_indices],
                                 y[valid_indices]))  # zip makes tuples
        return circle_points

    def find_closest_points_(self, circle_points, occupied_points):
        # all circle_points closer to robot i than to any neighbour
        dists_to_robot = np.linalg.norm(np.array(circle_points) -
                                        self.robot_position,
                                        axis=1)
        dists_to_occupied_points = np.linalg.norm(
            np.array(circle_points)[:, np.newaxis] - np.array(occupied_points),
            axis=2
        )  # shape (num_circle_points, num_neighbours) # table with all distances
        # demand: dist to robot < dist to any neighbour
        valid_indices = np.all(
            dists_to_robot[:, np.newaxis] < dists_to_occupied_points,
            axis=1)  # shape (num_circle_points,) boolean array

        closer_points = np.array(circle_points)[valid_indices].tolist()
        return closer_points  # list of tuples

    def find_closest_points(self, circle_points):
        # all circle_points closer to robot i than to any neighbour
        dists_to_robot = np.linalg.norm(np.array(circle_points) -
                                        self.robot_position,
                                        axis=1)
        dists_to_neighbours = np.linalg.norm(
            np.array(circle_points)[:, np.newaxis] -
            np.array(self.neighbour_positions),
            axis=2
        )  # shape (num_circle_points, num_neighbours) # table with all distances
        # demand: dist to robot < dist to any neighbour
        valid_indices = np.all(
            dists_to_robot[:, np.newaxis] < dists_to_neighbours,
            axis=1)  # shape (num_circle_points,) boolean array

        closer_points = np.array(circle_points)[valid_indices].tolist()
        return closer_points  # list of tuples

    def consider_encumbrances(self, cell_points):

        index = []
        for j, neighbour in enumerate(self.neighbour_positions):
            # encumbrance of neighbours
            # vector from i to j for middle point between i and j
            dx = self.robot_position[0] - neighbour[0]
            dy = self.robot_position[1] - neighbour[1]

            if abs(dx) < 0.001:
                dx = 0.001  # avoid division by zero
            if abs(dy) < 0.001:
                dy = 0.001  # avoid division by zero

            m = dy / dx  # slope

            if abs(m) < 0.001:
                m = 0.001  # avoid division by zero

            # coordinates of middle point
            xm = (self.robot_position[0] + neighbour[0]) / 2
            ym = (self.robot_position[1] + neighbour[1]) / 2

            # length of that vector
            dm = np.linalg.norm(
                [xm - self.robot_position[0], ym - self.robot_position[1]])

            # r < 1/2 dm --> r < 1/4 ||p_i - p_j|| --> delta_ij < 1/2 ||p_i - p_j||
            if dm < self.encumbrance_neighbours[j] + self.encumbrance:
                normal_ij = np.array([dx, dy]) / np.linalg.norm([dx, dy])
                solx = xm + (self.encumbrance_neighbours[j] +
                             self.encumbrance - dm) * normal_ij[0]
                soly = ym + (self.encumbrance_neighbours[j] +
                             self.encumbrance - dm) * normal_ij[1]

                # Geradengleichung für Trennungslinie der Zelle aufstellen und umstellen
                # y = - 1/m * (x - solx) + soly
                # y + 1/m * (x - solx) - soly = 0
                if self.robot_position[1] + 1 / m * (self.robot_position[0] -
                                                     solx) - soly > 0:
                    # Robot i is above the line
                    for k, point in enumerate(cell_points):
                        if point[1] + 1 / m * (point[0] - solx) - soly < 0:
                            index.append(
                                k)  # all points on the wrong side of the line
                else:
                    # Robot i is below the line
                    for k, point in enumerate(cell_points):
                        if point[1] + 1 / m * (point[0] - solx) - soly > 0:
                            index.append(
                                k)  # all points on the wrong side of the line
        cell_points_filtered = [
            point for k, point in enumerate(cell_points) if k not in index
        ]
        return cell_points_filtered

    def consider_barriers(self, cell_points):
        # encumbrance of barriers
        # first we filter out points that are too close to barriers
        # remove coordinates in cell_points that are closer than encumbrance_barriers to any (filtered) barrier

        if len(self.barrier_positions) > 0:
            # GEOMETRIC FILTERING
            # first we filter out points that are outside the basin or inside an obstacle

            cell_points = np.array(cell_points)  # shape (num_cell_points, 2)
            x, y = cell_points[:, 0], cell_points[:, 1]

            # buffer to mask, consisting of robot encumbrance + barrier encumbrance
            buffer = self.encumbrance + self.encumbrance_barriers_float

            # basin-mask Check (all points inside limits are valid)
            basin_mask = (
                (x > self.basin_limits[0][0] + self.encumbrance_barriers_float)
                &
                (x < self.basin_limits[0][1] - self.encumbrance_barriers_float)
                &
                (y > self.basin_limits[1][0] + self.encumbrance_barriers_float)
                & (y < self.basin_limits[1][1] -
                   self.encumbrance_barriers_float))

            # obstacle-mask Check (all points outside limits are valid)
            # ~ is the NOT operator

            obstacle_mask = ~((x > (self.obstacle_limits[0][0] - buffer)) &
                              (x < (self.obstacle_limits[0][1] + buffer)) &
                              (y > (self.obstacle_limits[1][0] - buffer)) &
                              (y < (self.obstacle_limits[1][1] + buffer)))

            valid_mask = basin_mask & obstacle_mask

            cell_points = cell_points[valid_mask]

            # DISTANCE FILTERING
            # now filter out points that are too close to barriers
            barrier_positions = np.array(self.barrier_positions)

            # delta_barriers_robot_position =
            dists_to_barriers = np.linalg.norm(
                np.array(cell_points)[:, np.newaxis] - barrier_positions,
                axis=2)
            safety_margin = self.encumbrance_barriers_float + self.encumbrance
            valid_indices = np.all(dists_to_barriers > safety_margin, axis=1)

            cell_points = np.array(cell_points)[valid_indices]

        return cell_points.tolist()

    def compute_scalar_values(self, x_cell, y_cell):
        x_cell = np.array(x_cell)
        y_cell = np.array(y_cell)

        dists_to_goal = np.linalg.norm(np.column_stack(
            (x_cell - self.goal_position[0], y_cell - self.goal_position[1])),
                                       axis=1)
        scalar_vals = np.exp(-dists_to_goal /
                             self.beta)  # beta is spreading factor rho
        return scalar_vals.tolist()

    def compute_control(self, **kwargs):
        centroid = kwargs.get('centroid', None)
        if centroid is None:
            centroid, _ = self.get_centroid()

        error = centroid - self.robot_position
        u = self.k_p * error  # control input # PROPORTIONAL CONTROLLER
        # TODO maybe upgrade to General COntrol Law
        return u if np.linalg.norm(
            u) <= self.v_max else u / np.linalg.norm(u) * self.v_max

    def move(self):
        x, y = self.robot_position
        u = self.compute_control()
        x_new = x + u[0] * self.dt
        y_new = y + u[1] * self.dt
        return np.array([x_new, y_new])


def applyrules(i, params, beta, current_positions, c1, c2, theta,
               goal_positions, BlueRovs, c1_no_rotation, d2, d4):
    c1_i = np.array(c1[i])  # centroid with neighbours
    current_position_i = np.array(current_positions[i])

    # first condition
    dist_c1_c2 = np.linalg.norm(c1_i - np.array(c2[i]))
    if dist_c1_c2 > d2 and np.linalg.norm(current_position_i -
                                          c1_i) < params["d1"]:
        beta[i] = max(beta[i] - params["dt"], params["beta_min"])
    else:
        beta[i] = beta[i] - params["dt"] * (beta[i] - params["betaD"][i])

    # second condition
    dist_c1_c2_d4 = dist_c1_c2 > d4
    if dist_c1_c2_d4 and np.linalg.norm(current_position_i -
                                        c1_i) < params["d3"]:
        theta[i] = min(theta[i] + params["dt"], np.pi / 2)
    else:
        theta[i] = max(0, theta[i] - params["dt"])

    # third condition
    if theta[i] == np.pi / 2 and np.linalg.norm(current_position_i - np.array(
            c1_no_rotation[i])) > np.linalg.norm(current_position_i - c1_i):
        theta[i] = 0

    # compute the angle and new position
    angle = np.arctan2(goal_positions[i][1] - current_position_i[1],
                       goal_positions[i][0] - current_position_i[0])
    new_angle = angle - theta[i]
    distance = np.sqrt((goal_positions[i][0] - current_position_i[0])**2 +
                       (goal_positions[i][1] - current_position_i[1])**2)
    BlueRovs.goal_positions[i][0] = current_position_i[
        0] + distance * math.cos(new_angle)  # new goalposition x
    BlueRovs.goal_positions[i][1] = current_position_i[
        1] + distance * math.sin(new_angle)  # new goalposition y
    # BlueRovs.destinations[i][0] = current_position


class barriers:

    def __init__(self, cell_resolution, obstacle_limits, pool_limits):
        self.cell_resolution = cell_resolution  # m adjustable kommt aus main.py "dx"
        self.lim_basin_x = np.array(pool_limits[0])
        self.lim_basin_y = np.array(pool_limits[1])
        self.lim_obs_x = np.array(obstacle_limits[0])
        self.lim_obs_y = np.array(obstacle_limits[1])

    def get_walls(self, corners):
        walls = []
        num_corners = len(corners)
        for i in range(num_corners):
            start = corners[i]
            end = corners[(i + 1) %
                          num_corners]  # Wrap around to the first corner
            if start[0] == end[0]:  # Vertical wall (same x-coordinate)
                y_values = np.arange(min(start[1], end[1]),
                                     max(start[1], end[1]) + self.cell_resolution,
                                     self.cell_resolution)
                wall = np.column_stack((np.full_like(y_values,
                                                     start[0]), y_values))
            elif start[1] == end[1]:  # Horizontal wall, same y-coordinate
                x_values = np.arange(min(start[0], end[0]),
                                     max(start[0], end[0]) + self.cell_resolution,
                                     self.cell_resolution)
                wall = np.column_stack(
                    (x_values, np.full_like(x_values, start[1])))
            else:
                raise ValueError(
                    "Walls must be either vertical or horizontal. Please rearrange corners."
                )
            walls.append(wall)
        walls = np.vstack(walls)
        walls = np.round(walls, decimals=3)  # avoid floating point issues
        walls = np.unique(walls, axis=0)  # remove duplicate corner points
        return walls

    def def_barriers(self):

        basin_corners = np.array([[self.lim_basin_x[0], self.lim_basin_y[0]],
                                  [self.lim_basin_x[1], self.lim_basin_y[0]],
                                  [self.lim_basin_x[1], self.lim_basin_y[1]],
                                  [self.lim_basin_x[0], self.lim_basin_y[1]]])
        obstacle_corners = np.array([[self.lim_obs_x[0], self.lim_obs_y[0]],
                                     [self.lim_obs_x[1], self.lim_obs_y[0]],
                                     [self.lim_obs_x[1], self.lim_obs_y[1]],
                                     [self.lim_obs_x[0], self.lim_obs_y[1]]])

        obstacle_walls = self.get_walls(obstacle_corners)
        basin_walls = self.get_walls(basin_corners)

        return np.vstack((obstacle_walls, basin_walls))

    def get_limits(self):
        # return basinlimits, obstacle limits

        return np.array([self.lim_basin_x, self.lim_basin_y
                         ]), np.array([self.lim_obs_x, self.lim_obs_y])


def actions_to_matrix_points(p_start_matrix, actions: list[search.Action]):
    points = [[p_start_matrix[0], p_start_matrix[1]]]
    for action in actions:
        v_direction = action.vector
        x_prev = points[-1][0]
        y_prev = points[-1][1]
        points.append([x_prev + v_direction.x, y_prev + v_direction.y])
    return points


class State(Enum):
    UNSET = auto()
    INIT = auto()
    IDLE = auto()
    MOVE_TO_START = auto()
    NORMAL_OPERATION = auto()


def occupancy_grid_to_matrix(grid: OccupancyGrid):
    data = np.array(grid.data, dtype=np.uint8)
    data = data.reshape(grid.info.height, grid.info.width)
    return data.transpose()


def world_to_matrix(x, y, grid_size):
    return [round(x / grid_size), round(y / grid_size)]


def matrix_index_to_world(x, y, grid_size):
    return [x * grid_size, y * grid_size]


def multiple_matrix_indeces_to_world(points, grid_size):
    world_points = []
    for point in points:
        world_points.append([point[0] * grid_size, point[1] * grid_size])
    return world_points


def compute_discrete_line(x0, y0, x1, y1):
    # calculates the points of a discrete grid that are intersected by the
    # line, defined by its start- and endpoint
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy

    x = x0
    y = y0
    points = []
    while True:
        points.append([int(x), int(y)])
        if x == x1 and y == y1:
            break
        doubled_error = 2 * error
        if doubled_error >= dy:
            if x == x1:
                break
            error += dy
            x += sx
        if doubled_error <= dx:
            if y == y1:
                break
            error += dx
            y += +sy
    return points


class LloydPathPlanner(Node):

    def __init__(self):
        super().__init__(node_name='lloyd_path_planner')
        self.state = State.UNSET
        self.recomputation_required = True
        self.target_viewpoint_index = -1
        self.path_marker: Marker
        self.init_path_marker()
        self.path_marker_pub = self.create_publisher(Marker, '~/marker', 1)
        self.viewpoints: Viewpoints = None
        self.waypoints = []
        self.orientations = []
        self.occupancy_grid: OccupancyGrid = None
        self.occupancy_matrix: np.ndarray = None
        self.progress = -1.0
        self.path_segments = []
        self.path_counter = 0

        self.compute_full_path = False
        self.init_params()

        self.init_clients()
        self.init_services()
        self.grid_map_sub = self.create_subscription(OccupancyGrid,
                                                     'occupancy_grid',
                                                     self.on_occupancy_grid, 1)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                 'vision_pose_cov',
                                                 self.on_pose, 1)
        self.viewpoints_sub = self.create_subscription(Viewpoints,
                                                       'viewpoints',
                                                       self.on_viewpoints, 1)

    def init_params(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pool_limits_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('pool_limits_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('cell_resolution', rclpy.Parameter.Type.DOUBLE),
                ('obstacle_limits_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('obstacle_limits_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('N', rclpy.Parameter.Type.INTEGER),
                ('radius', rclpy.Parameter.Type.DOUBLE), # sensing radius
                ('size', rclpy.Parameter.Type.DOUBLE), # robot size
                ('d1', rclpy.Parameter.Type.DOUBLE),
                ('d3', rclpy.Parameter.Type.DOUBLE),
                ('k', rclpy.Parameter.Type.INTEGER),
                ('betaD', rclpy.Parameter.Type.DOUBLE),
                ('beta_min', rclpy.Parameter.Type.DOUBLE),
                ('encumbrance_barriers', rclpy.Parameter.Type.DOUBLE),
                ('num_steps', rclpy.Parameter.Type.INTEGER),
                ('waiting_time', rclpy.Parameter.Type.INTEGER),
                ('bluerov_id', rclpy.Parameter.Type.INTEGER),

            ],
            # alle Parameter, die für die Lloyd-Initialisierung benötigt werden
        )
        self.pool_limits_x = self.get_parameter('pool_limits_x').value
        self.pool_limits_y = self.get_parameter('pool_limits_y').value
        self.cell_resolution = self.get_parameter('cell_resolution').value
        self.obstacle_limits_x = self.get_parameter('obstacle_limits_x').value
        self.obstacle_limits_y = self.get_parameter('obstacle_limits_y').value
        self.N = self.get_parameter('N').value
        self.radius = self.get_parameter('radius').value
        self.size = self.get_parameter('size').value
        self.d1 = self.get_parameter('d1').value
        self.d3 = self.get_parameter('d3').value
        self.k = self.get_parameter('k').value
        self.betaD = self.get_parameter('betaD').value
        self.beta_min = self.get_parameter('beta_min').value
        self.encumbrance_barriers = self.get_parameter('encumbrance_barriers').value
        self.num_steps = self.get_parameter('num_steps').value
        self.waiting_time = self.get_parameter('waiting_time').value
        

    def init_services(self):
        self.move_to_start_service = self.create_service(
            MoveToStart, '~/move_to_start', self.serve_move_to_start)
        self.start_service = self.create_service(Trigger, '~/start',
                                                 self.serve_start)
        self.stop_service = self.create_service(Trigger, '~/stop',
                                                self.serve_stop)

    def init_clients(self):
        cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.set_path_client = self.create_client(SetPath,
                                                  'path_follower/set_path',
                                                  callback_group=cb_group)
        self.path_finished_client = self.create_client(
            Trigger, 'path_follower/path_finished', callback_group=cb_group)

    def on_pose(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def serve_move_to_start(self, request, response):
        self.state = State.MOVE_TO_START
        print('start_pose: ', request.current_pose)
        print('target_pose: ', request.target_pose)
        self.start_pose = request.target_pose
        self.current_pose = request.current_pose
        # we do not care for collisions while going to the start position
        # in the simulation 'collisions' do not matter. In the lab, we
        # can manually make sure that we avoid collisions, by bringing the
        # vehicle in a safe position manually before starting anything.
        response.success = self.move_to_start(request.current_pose,
                                              request.target_pose)
        return response

    def move_to_start(self, p0: Pose, p1: Pose):
        path_segment = self.compute_simple_path_segment(p0,
                                                        p1,
                                                        check_collision=False)
        request = SetPath.Request()
        request.path = path_segment['path']
        answer = self.set_path_client.call(request)
        if answer.success:
            self.get_logger().info('Moving to start position')
            return True
        else:
            self.get_logger().info(
                'Asked to move to start position. '
                'But the path follower did not accept the new path.')
            return False

    def has_collisions(self, points_2d):
        if not self.occupancy_grid:
            return []
        collision_indices = [
            i for i, p in enumerate(points_2d)
            if self.occupancy_matrix[p[0], p[1]] >= 50
        ]
        return collision_indices

    def is_walkable(self, p0, p1):  # noqa: C901
        # checks if a line from p0 to p1 intersects any occupied fields of the
        # occupancy matrix
        # https://www.gamedeveloper.com/programming/toward-more-realistic-pathfinding
        # http://eugen.dedu.free.fr/projects/bresenham/
        def infeasible_point(x, y):
            return self.occupancy_matrix[x, y] > 0

        x1 = p0[0]
        y1 = p0[1]
        x2 = p1[0]
        y2 = p1[1]
        # i = np.nan            # loop counter
        ystep = np.nan
        xstep = np.nan  # the step on y and x axis
        error = np.nan  # the error accumulated during the increment
        errorprev = np.nan  # *vision the previous value of the error variable
        y = y1
        x = x1  # the line points
        ddy = np.nan
        ddx = np.nan  # compulsory variables: the double values of dy and dx
        dx = x2 - x1
        dy = y2 - y1
        if infeasible_point(x1, y1):  # first point
            return False
        # NB the last point can't be here, because of its previous point
        # (which has to be verified)
        if dy < 0:
            ystep = -1
            dy = -dy
        else:
            ystep = 1
        if dx < 0:
            xstep = -1
            dx = -dx
        else:
            xstep = 1
        ddy = 2 * dy  # work with double values for full precision
        ddx = 2 * dx
        if ddx >= ddy:  # first octant (0 <= slope <= 1)
            # compulsory initialization (even for errorprev, needed when dx==dy)
            error = dx
            errorprev = dx  # start in the middle of the square
            for _ in range(dx):  # do not use the first point (already done)
                x += xstep
                error += ddy
                if error > ddx:  # increment y if AFTER the middle ( > )
                    y += ystep
                    error -= ddx
                    # three cases (octant == right->right-top for directions
                    # below):
                    if error + errorprev < ddx:  # bottom square also
                        if infeasible_point(x, y - ystep):
                            return False
                    elif error + errorprev > ddx:  # left square also
                        if infeasible_point(x - xstep, y):
                            return False
                    else:  # corner: bottom and left squares also
                        if infeasible_point(x, y - ystep):
                            return False
                        if infeasible_point(x - xstep, y):
                            return False
                if infeasible_point(x, y):
                    return False
                errorprev = error
        else:  # the same as above
            error = dy
            errorprev = dy
            for _ in range(dy):
                y += ystep
                error += ddx
                if error > ddy:
                    x += xstep
                    error -= ddy
                    if error + errorprev < ddy:
                        if infeasible_point(x - xstep, y):
                            return False
                    elif error + errorprev > ddy:
                        if infeasible_point(x, y - ystep):
                            return False
                    else:
                        if infeasible_point(x - xstep, y):
                            return False
                        if infeasible_point(x, y - ystep):
                            return False
                if infeasible_point(x, y):
                    return False
                errorprev = error
        if (not y == y2) | (
                not x == x2
        ):  # the last point (y2,x2) has to be the same with the last point of
            # the algorithm
            print('Algorithm did not end at endpoint!')
            return False
        return True

    def straighten_path(self, points):
        # postprocessing, reduces "zig-zag" behavior of A-star by deleting
        # ntermediate points if feasible for collisons
        # see: https://www.gamedeveloper.com/programming/toward-more-realistic-pathfinding
        idxs = [0]
        i = 1
        while i + 1 < len(points):
            if not self.is_walkable(points[idxs[-1]], points[i + 1]):
                idxs.append(i)
            i += 1
        idxs.append(len(points) - 1)
        key_points = [points[idx] for idx in idxs]

        # resample points between key_points
        resampled_points = []
        for i in range(len(key_points) - 1):
            n_resampling = int(
                np.ceil(
                    np.sqrt((key_points[i + 1][0] - key_points[i][0])**2 +
                            (key_points[i + 1][1] - key_points[i][1])**2)))
            resampled_x = np.linspace(key_points[i][0], key_points[i + 1][0],
                                      n_resampling)
            resampled_y = np.linspace(key_points[i][1], key_points[i + 1][1],
                                      n_resampling)
            if i == 0:
                idx_range = range(0, n_resampling)
            else:
                idx_range = range(1, n_resampling)
            resampled_points = resampled_points + [
                [resampled_x[idx], resampled_y[idx]] for idx in idx_range
            ]
        return resampled_points

    def postprocess_path(self,
                         p0: Pose,
                         p1: Pose,
                         points: [tuple],
                         check_collisions=True):
        # straighten path and interpolate yaw
        if check_collisions:
            collision_indices = self.has_collisions(points)
        else:
            collision_indices = []
        if not collision_indices:  # post process path further such that
            # unnecessary zic-zac behaviour is eliminated
            # and shortest straight paths are chosen
            points = self.straighten_path(points)
        # Convert back our matrix/grid_map points to world coordinates. Since
        # the grid_map does not contain information about the z-coordinate,
        # the following list of points only contains the x and y component.
        xy_3d = multiple_matrix_indeces_to_world(points, self.cell_resolution)

        # it might be, that only a single grid point brings us from p0 to p1.
        # in this duplicate this point. this way it is easier to handle.
        if len(xy_3d) == 1:
            xy_3d.append(xy_3d[0])
        z0 = p0.position.z
        z1 = p1.position.z
        z_step = (z1 - z0) / (len(xy_3d) - 1)
        points_3d = [
            Point(x=p[0], y=p[1], z=z0 + i * z_step)
            for i, p in enumerate(xy_3d)
        ]
        # Replace the last point with the exac value stored in p1.position
        # instead of the grid map discretized world coordinate
        points_3d[-1] = p1.position
        # Now we have a waypoint path with the x and y component computed by
        # our path finding algorithm and z is a linear interpolation between
        # the z coordinate of the start and the goal pose.

        # now we need to compute our desired heading (yaw angle) while we
        # follow the waypoints. We choose a not-so-clever approach by
        # keeping the yaw angle from our start pose and only set the yaw
        # angle to the desired yaw angle from the goal pose for the very last
        # waypoint
        q0 = p0.orientation
        _, _, yaw0 = euler_from_quaternion([q0.x, q0.y, q0.z, q0.w])
        q1 = p1.orientation
        _, _, yaw1 = euler_from_quaternion([q1.x, q1.y, q1.z, q1.w])

        # replace the very last orientation with the orientation of our
        # goal pose p1.
        yaws = np.linspace(yaw0, yaw1, len(points_3d))
        orientations = []
        for yaw in yaws:
            q = quaternion_from_euler(0.0, 0.0, yaw)
            orientations.append(Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

        path = Path()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        path.poses = [
            PoseStamped(header=header, pose=Pose(position=p, orientation=q))
            for p, q in zip(points_3d, orientations)
        ]
        return {'path': path, 'collision_indices': collision_indices}

    def compute_simple_path_segment(self,
                                    p0: Pose,
                                    p1: Pose,
                                    check_collision=True):
        p0_2d = world_to_matrix(p0.position.x, p0.position.y, self.cell_resolution)
        p1_2d = world_to_matrix(p1.position.x, p1.position.y, self.cell_resolution)
        # now we should/could apply some sophisticated algorithm to compute
        # the path that brings us from p0_2d to p1_2d. For this dummy example
        # we simply go in a straight line. Not very clever, but a straight
        # line is the shortes path between two points, isn't it?
        line_points_2d = compute_discrete_line(p0_2d[0], p0_2d[1], p1_2d[0],
                                               p1_2d[1])
        if check_collision:
            collision_indices = self.has_collisions(line_points_2d)
        else:
            collision_indices = []

        # Convert back our matrix/grid_map points to world coordinates. Since
        # the grid_map does not contain information about the z-coordinate,
        # the following list of points only contains the x and y component.
        xy_3d = multiple_matrix_indeces_to_world(line_points_2d,
                                                 self.cell_resolution)

        # it might be, that only a single grid point brings us from p0 to p1.
        # in this duplicate this point. this way it is easier to handle.
        if len(xy_3d) == 1:
            xy_3d.append(xy_3d[0])
        z0 = p0.position.z
        z1 = p1.position.z
        z_step = (z1 - z0) / (len(xy_3d) - 1)
        points_3d = [
            Point(x=p[0], y=p[1], z=z0 + i * z_step)
            for i, p in enumerate(xy_3d)
        ]
        # Replace the last point with the exac value stored in p1.position
        # instead of the grid map discretized world coordinate
        points_3d[-1] = p1.position
        # Now we have a waypoint path with the x and y component computed by
        # our path finding algorithm and z is a linear interpolation between
        # the z coordinate of the start and the goal pose.

        # now we need to compute our desired heading (yaw angle) while we
        # follow the waypoints. We choose a not-so-clever approach by
        # keeping the yaw angle from our start pose and only set the yaw
        # angle to the desired yaw angle from the goal pose for the very last
        # waypoint
        q0 = p0.orientation
        _, _, yaw0 = euler_from_quaternion([q0.x, q0.y, q0.z, q0.w])
        q1 = p1.orientation
        _, _, yaw1 = euler_from_quaternion([q1.x, q1.y, q1.z, q1.w])

        # replace the very last orientation with the orientation of our
        # goal pose p1.
        yaws = np.linspace(yaw0, yaw1, len(points_3d))
        orientations = []
        for yaw in yaws:
            q = quaternion_from_euler(0.0, 0.0, yaw)
            orientations.append(Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

        path = Path()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        path.poses = [
            PoseStamped(header=header, pose=Pose(position=p, orientation=q))
            for p, q in zip(points_3d, orientations)
        ]
        return {'path': path, 'collision_indices': collision_indices}

    def reset_internals(self):
        self.target_viewpoint_index = -1
        self.recomputation_required = True
        self.state = State.UNSET

    def serve_start(self, request, response):
        if self.state != State.NORMAL_OPERATION:
            self.get_logger().info('Starting normal operation.')
            self.reset_internals()
        self.state = State.NORMAL_OPERATION
        response.success = True
        return response

    def serve_stop(self, request, response):
        if self.state != State.IDLE:
            self.get_logger().info('Asked to stop. Going to idle mode.')
        response.success = self.do_stop()
        return response

    def do_stop(self):
        self.state = State.IDLE
        if self.path_finished_client.call(Trigger.Request()).success:
            self.reset_internals()
            self.state = State.IDLE
            return True
        return False

    def handle_mission_completed(self):
        self.get_logger().info('Mission completed.')
        if not self.do_stop():
            self.get_logger().error(
                'All waypoints completed, but could not '
                'stop the path_follower. Trying again...',
                throttle_duration_sec=1.0,
            )
            return
        self.state = State.IDLE

    def a_star_segment(self, p0, p1):
        start = world_to_matrix(p0.x, p0.y, self.cell_resolution)
        p_start = search.Position(*start)
        goal = world_to_matrix(p1.x, p1.y, self.cell_resolution)
        p_goal = search.Position(*goal)
        initial_node = search.Node(p_start, 0.0, p_goal, None, [])
        goal_node = search.Node(p_goal, 0.0, p_goal, None, [])
        problem = search.SearchProblem(initial_node, goal_node,
                                       self.occupancy_matrix)
        return search.a_star(problem)

    def compute_new_path(self, viewpoints: Viewpoints):
        original_indices = []
        viewpoint_poses = []
        for i, v in enumerate(viewpoints.viewpoints):
            if not v.completed:
                viewpoint_poses.append(v.pose)
                original_indices.append(i)
        # TODO:
        # 1. generate all viewpoint segments
        # 2. compute distance by a star
        # 3. find the best order
        indices = range(len(viewpoint_poses))
        permutation_iterable = permutations(indices)
        segment_costs = {}
        best_permutation = []
        min_costs = math.inf
        viewpoint_poses.insert(0, self.current_pose)
        for permutation in permutation_iterable:
            costs = 0.0
            indices = list(permutation)
            permutated_original_indices = [
                original_indices[i] for i in indices
            ]
            indices = [0] + [x + 1 for x in indices]
            for i, _ in enumerate(indices):
                if i == 0:
                    continue
                a = indices[i - 1]
                b = indices[i]
                ab = frozenset([a, b])
                if ab not in segment_costs:
                    pa = viewpoint_poses[a].position
                    pb = viewpoint_poses[b].position
                    result = self.a_star_segment(pa, pb)
                    if result is None:
                        self.get_logger().fatal(
                            'Could not find a valid solution for a star '
                            f'between viewpoint {a-1} and {b-1}')
                        raise ValueError(
                            'Could not find a valid solution for a star '
                            f'between viewpoint {a-1} and {b-1}')
                    segment_costs[ab] = result.get_total_costs()
                costs += segment_costs[ab]
            if costs < min_costs:
                best_permutation = list(permutation)
                self.permutated_original_indices = permutated_original_indices
                min_costs = costs

        indices = [0] + [x + 1 for x in best_permutation]
        segments = []
        for i, _ in enumerate(indices):
            if i == 0:
                continue
            a = indices[i - 1]
            b = indices[i]
            pa = viewpoint_poses[a].position
            pb = viewpoint_poses[b].position
            result = self.a_star_segment(pa, pb)
            p_start = world_to_matrix(pa.x, pa.y, self.cell_resolution)
            waypoints = actions_to_matrix_points(p_start,
                                                 result.action_sequence)
            segment = self.postprocess_path(viewpoint_poses[a],
                                            viewpoint_poses[b], waypoints,
                                            False)
            segments.append(segment)
        return segments

    def handle_no_collision_free_path(self):
        self.get_logger().fatal('We have a collision in our current segment!'
                                'Giving up...')
        if self.do_stop():
            self.state = State.IDLE
        else:
            self.state = State.UNSET

    def do_normal_operation(self, viewpoints: Viewpoints):
        n_completed_viewpoints = self.count_completed_viewpoints(viewpoints)
        # we completed our mission!
        if n_completed_viewpoints == len(viewpoints.viewpoints):
            self.handle_mission_completed()
            return
        if not self.recomputation_required:
            # we are still chasing the same viewpoint. Nothing to do.
            return
        # if n_completed_viewpoints == 0:
        #     p = viewpoints.viewpoints[0].pose
        #     if not self.move_to_start(p, p):
        #         self.get_logger().fatal(
        #             'Could not move to first viewpoint. Giving up...'
        #         )
        #         if self.do_stop():
        #             self.state = State.IDLE
        #         else:
        #             self.state = State.UNSET
        #     return
        if self.did_viewpoints_change(viewpoints) or (
                n_completed_viewpoints == 1
                and self.target_viewpoint_index == -1):
            self.target_viewpoint_index = 1
            t_start = time.time()
            self.path_segments = self.compute_new_path(viewpoints)
            t_end = time.time()
            self.get_logger().error(f'Computation took {(t_end-t_start):.3f}')
            self.path_counter = 0
            self.set_new_path(self.path_segments[self.path_counter]['path'])
        self.viewpoints = viewpoints
        # if self.path_segments[self.path_counter]['collision_indices']:
        #     self.handle_no_collision_free_path()
        #     return
        if self.viewpoints.viewpoints[self.permutated_original_indices[
                self.path_counter]].completed:
            self.path_counter += 1
            self.set_new_path(self.path_segments[self.path_counter]['path'])
        return

    def did_viewpoints_change(self, new_viewpoints: Viewpoints):
        if self.viewpoints is None:
            return True
        if len(new_viewpoints.viewpoints) != len(self.viewpoints.viewpoints):
            return True

        def viewpoints_equal(v0: Viewpoint, v1: Viewpoint):
            p0 = v0.pose.position
            p1 = v1.pose.position
            q0 = v0.pose.orientation
            q1 = v1.pose.orientation
            return (p0.x == p1.x and p0.y == p1.y and p0.z == p1.z
                    and q0.w == q1.w and q0.x == q1.x and q0.y == q1.y
                    and q0.z == q1.z)

        for i in range(len(new_viewpoints.viewpoints)):
            if not viewpoints_equal(new_viewpoints.viewpoints[i],
                                    self.viewpoints.viewpoints[i]):
                return True

        return False

    def on_viewpoints(self, msg: Viewpoints):
        self.get_logger().info('Viewpoint received', throttle_duration_sec=1.0)
        if self.state == State.IDLE:
            return
        if self.state == State.UNSET:
            if self.do_stop():
                self.state = State.IDLE
            else:
                self.get_logger().error('Failed to stop.')
            return
        if self.state == State.MOVE_TO_START:
            # nothing to be done here. We already did the setup when the
            # corresponding service was called
            return
        if self.state == State.NORMAL_OPERATION:
            self.do_normal_operation(msg)

    def find_first_uncompleted_viewpoint(self, viewpoints: Viewpoints):
        for i, viewpoint in enumerate(viewpoints.viewpoints):
            if not viewpoint.completed:
                return i
        # This should not happen!
        return -1

    def count_completed_viewpoints(self, viewpoints: Viewpoints):
        counter = 0
        for viewpoint in viewpoints.viewpoints:
            if viewpoint.completed:
                counter += 1
        # This should not happen!
        return counter

    def on_occupancy_grid(self, msg: OccupancyGrid):
        self.occupancy_grid = msg
        self.occupancy_matrix = occupancy_grid_to_matrix(self.occupancy_grid)
        if msg.info.resolution != self.cell_resolution:
            self.get_logger().info(
                'Cell size changed. Recomputation required.')
            self.recomputation_required = True
            self.cell_resolution = msg.info.resolution

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

    def set_new_path(self, path):
        request = SetPath.Request()
        if not path:
            return False
        request.path = path
        self.set_new_path_future = self.set_path_client.call_async(request)
        return True

    def publish_path_marker(self, segments):
        msg = self.path_marker
        world_points = self.segments_to_world_points(segments)
        msg.points = [Point(x=p[0], y=p[1], z=-0.5) for p in world_points]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.path_marker_pub.publish(msg)


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
