# Übergabeparamter:
# - Start- und Zielpositionen der Roboter
# - Raum- und Hindernisabmessungen
# - Roboterabmessungen
# roboter1_start = (0.5, 0.5)
# roboter1_ziel = (1.5, 3.5)
# roboter2_start = (1.5, 0.5)
# roboter2_ziel = (0.5, 3.5)
# barriers

# roboter_radius = 0.25

# in diesem Skript definierte Parameter:
# Sicherheitsabstand zwischen Roboter und Hindernis
# sicherheitsabstand zwischen Robotern
# sicherheitsabstand Roboter und wand

# barriers in jeden Schritt aktualisieren

import numpy as np
import math


class Lloyd:

    def __init__(self, radius, encumbrance_i, step_size, encumbrance_barriers,
                 all_barriers, basin_limits,
                 obstacle_limits):  #ggf noch v_max TODO
        # initialisieren des Klassenobjektes des i-ten Roboters
        self.radius = radius  # same for all robots
        self.encumbrance = encumbrance_i
        self.step_size = step_size
        self.encumbrance_barriers_float = encumbrance_barriers
        self.barriers_unfiltered = all_barriers
        self.basin_limits = np.array(basin_limits)
        self.obstacle_limits = np.array(obstacle_limits)

        self.epsilon = 0.0  # epsilon adaptation parameter for more cautelative/aggressive behaviour

    # wird in jeder Iteration aufgerufen und deklariert die Nachbarpositionen neu
    def aggregate(self, neighbour_positions, beta_i, goal_position_i,
                  final_goal_i, dt, current_position):
        self.neighbour_positions = np.array(
            neighbour_positions
        )  # schon gefiltert, nur nachbarn innerhlab von R_S
        self.beta = beta_i
        self.goal_position = goal_position_i
        self.final_goal = final_goal_i
        self.dt = dt
        self.current_position = np.array(current_position)
        self.filter_barriers()

    def filter_barriers(self):
        # Hindernisse filtern, die innerhalb eines bestimmten Abstands sind 1*r
        # ggf hier noch mal sicherstellen. dass barriers und current_position ein numpy array sind
        if not isinstance(self.barriers_unfiltered, np.ndarray):
            self.barriers_unfiltered = np.array(self.barriers_unfiltered)
        if not isinstance(self.current_position, np.ndarray):
            self.current_position = np.array(self.current_position)

        dist = np.linalg.norm(self.barriers_unfiltered - self.current_position,
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

        if len(self.barrier_positions) > 0:
            # 1 --> initial voronoi cell filtering considering barriers
            cell_points = self.find_closest_points_epsilon(
                circle_points, self.barrier_positions)

            # 2 --> filter out points too close to barriers and all points behind barriers (encumbrance barriers)
            cell_points = self.consider_barriers(cell_points)
            if len(cell_points) == 0:
                cell_points = [self.current_position]
        else:
            cell_points = circle_points

        if len(self.neighbour_positions) > 0:
            # compute voronoi cell

            # 1 --> initial voronoi cell filtering considering neighbours
            # later Case: delta_ij <= (||p_i - n_j||)/2
            # cell_points are all q within circle with ||q-p_i|| < ||q-n_j|| for all neighbours j

            cell_points = self.find_closest_points_epsilon(
                cell_points, self.neighbour_positions)  # tupel-list

            # 2 --> filter cell points considering encumbrances
            # Case: delta_ij > (||p_i - n_j||)/2
            #  ||q-p_i|| < ||q-n_j~|| for all neighbours j
            cell_points = self.consider_encumbrances(cell_points)
            # if no points left in cell_points_filtered, return current position as centroid
            if len(cell_points) == 0:
                cell_points = [self.current_position]

        ## Größe der Zelle als Anhaltspunkt für Engheit der Umgebung
        self.cell_size = len(cell_points) # anzahl der koordinatenpunkte in der zelle

        x_cell, y_cell = zip(
            *cell_points)  # unzip tupel-list (* is unpacking operator)

        x_cell_no_influence, y_cell_no_influence = zip(
            *circle_points)  # unzip tupel-list (* is unpacking operator)
        # no neighbours and no obstacles

        # compute scalar values for weighted centroid
        scalar_values = self.compute_scalar_values(x_cell, y_cell,
                                                   self.goal_position)
        scalar_values_no_influence = self.compute_scalar_values(
            x_cell_no_influence, y_cell_no_influence, self.goal_position)
        scalar_values_no_rotation = self.compute_scalar_values(
            x_cell, y_cell, self.final_goal)
        # print(
        #     f'self.goal_position: {self.goal_position} und self.final_goal: {self.final_goal}'
        # )

        # make sure everything is numpy array
        x_cell = np.array(x_cell)
        y_cell = np.array(y_cell)
        scalar_values = np.array(scalar_values)
        x_cell_no_influence = np.array(x_cell_no_influence)
        y_cell_no_influence = np.array(y_cell_no_influence)
        scalar_values_no_influence = np.array(scalar_values_no_influence)
        scalar_values_no_rotation = np.array(scalar_values_no_rotation)

        # compute weighted centroids

        c1_x = np.sum(x_cell * scalar_values) / np.sum(scalar_values)
        c1_y = np.sum(y_cell * scalar_values) / np.sum(scalar_values)
        c2_x = np.sum(
            x_cell_no_influence *
            scalar_values_no_influence) / np.sum(scalar_values_no_influence)
        c2_y = np.sum(
            y_cell_no_influence *
            scalar_values_no_influence) / np.sum(scalar_values_no_influence)
        c1_norot_x = np.sum(
            x_cell *
            scalar_values_no_rotation) / np.sum(scalar_values_no_rotation)
        c1_norot_y = np.sum(
            y_cell *
            scalar_values_no_rotation) / np.sum(scalar_values_no_rotation)

        c1 = np.array([c1_x, c1_y])  # nieghbours
        c2 = np.array([c2_x, c2_y])  # no neighbours
        c1_norot = np.array([c1_norot_x, c1_norot_y])  # no rotation

        return c1, c2, c1_norot

    def get_circle_points(self):

        x_center, y_center = self.current_position

        x_min = x_center - self.radius
        x_max = x_center + self.radius
        y_min = y_center - self.radius
        y_max = y_center + self.radius

        x_coords = np.round(np.arange(x_min, x_max + self.step_size,
                                      self.step_size),
                            decimals=3)  # to avoid floating point issues
        y_coords = np.round(np.arange(y_min, y_max + self.step_size,
                                      self.step_size),
                            decimals=3)

        x, y = np.meshgrid(x_coords, y_coords)

        distances = np.sqrt((x - x_center)**2 + (y - y_center)**2)
        valid_indices = np.where(distances <= self.radius)

        circle_points = list(zip(x[valid_indices],
                                 y[valid_indices]))  # zip makes tuples
        return circle_points

    def find_closest_points(self, circle_points, occupied_points):
        # all circle_points closer to robot i than to any neighbour
        dists_to_robot = np.linalg.norm(np.array(circle_points) -
                                        self.current_position,
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

    def find_closest_points_epsilon(self, circle_points, occupied_points):
        # all circle_points closer to robot i than to any epsilon adapted neighbourposition
        # epsilon = 0 --> more cautelative behaviour
        # epsilon = 1 --> more aggressive behaviour
        dists_to_robot = np.linalg.norm(np.array(circle_points) -
                                        self.current_position,
                                        axis=1)
        # compute new occupied points with epsilon adaptation
        dists_robot_to_occupied = np.linalg.norm(np.array(occupied_points) -
                                                 self.current_position,
                                                 axis=1)

        for i in range(len(occupied_points)):
            direction = (np.array(occupied_points[i]) -
                         self.current_position) / dists_robot_to_occupied[i]
            occupied_points[i] = occupied_points[
                i] + direction * dists_robot_to_occupied[i] * self.epsilon

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

    def consider_encumbrances(self, cell_points):

        index = []
        for j, neighbour in enumerate(self.neighbour_positions):
            # encumbrance of neighbours
            # vector from i to j for middle point between i and j
            dx = self.current_position[0] - neighbour[0]
            dy = self.current_position[1] - neighbour[1]

            if abs(dx) < 0.001:
                dx = 0.001  # avoid division by zero
            if abs(dy) < 0.001:
                dy = 0.001  # avoid division by zero

            m = dy / dx  # slope

            if abs(m) < 0.001:
                m = 0.001  # avoid division by zero

            # coordinates of middle point
            xm = (self.current_position[0] + neighbour[0]) / 2
            ym = (self.current_position[1] + neighbour[1]) / 2

            # length of that vector
            dm = np.linalg.norm(
                [xm - self.current_position[0], ym - self.current_position[1]])

            # r < 1/2 dm --> r < 1/4 ||p_i - p_j|| --> delta_ij < 1/2 ||p_i - p_j||
            if dm < 2 * self.encumbrance:
                normal_ij = np.array([dx, dy]) / np.linalg.norm([dx, dy])
                solx = xm + (2 * self.encumbrance - dm) * normal_ij[0]
                soly = ym + (2 * self.encumbrance - dm) * normal_ij[1]

                # Geradengleichung für Trennungslinie der Zelle aufstellen und umstellen
                # y = - 1/m * (x - solx) + soly
                # y + 1/m * (x - solx) - soly = 0
                if self.current_position[1] + 1 / m * (
                        self.current_position[0] - solx) - soly > 0:
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

        # GEOMETRIC FILTERING
        # first we filter out points that are outside the basin or inside an obstacle

        cell_points = np.array(cell_points)  # shape (num_cell_points, 2)
        x, y = cell_points[:, 0], cell_points[:, 1]

        # buffer to mask, consisting of robot encumbrance + barrier encumbrance
        buffer = self.encumbrance + self.encumbrance_barriers_float

        # basin-mask Check (all points inside limits are valid)
        basin_mask = (
            (x > self.basin_limits[0][0] + self.encumbrance_barriers_float)
            & (x < self.basin_limits[0][1] - self.encumbrance_barriers_float)
            & (y > self.basin_limits[1][0] + self.encumbrance_barriers_float)
            & (y < self.basin_limits[1][1] - self.encumbrance_barriers_float))

        # obstacle-mask Check (all points outside limits are valid)
        # ~ is the NOT operator

        if np.size(self.obstacle_limits) >= 8:
            for obs in self.obstacle_limits:
                obstacle_mask = ~((x > (obs[0][0] - buffer)) &
                                  (x < (obs[0][1] + buffer)) &
                                  (y > (obs[1][0] - buffer)) &
                                  (y < (obs[1][1] + buffer)))
                basin_mask = basin_mask & obstacle_mask
            valid_mask = basin_mask
        else:
            obstacle_mask = ~((x > (self.obstacle_limits[0][0] - buffer)) &
                              (x < (self.obstacle_limits[0][1] + buffer)) &
                              (y > (self.obstacle_limits[1][0] - buffer)) &
                              (y < (self.obstacle_limits[1][1] + buffer)))

            valid_mask = basin_mask & obstacle_mask

        cell_points = cell_points[valid_mask]

        # DISTANCE FILTERING
        # now filter out points that are too close to barriers
        barrier_positions = np.array(self.barrier_positions)

        # delta_barriers_current_position =
        dists_to_barriers = np.linalg.norm(
            np.array(cell_points)[:, np.newaxis] - barrier_positions, axis=2)
        safety_margin = self.encumbrance_barriers_float + self.encumbrance
        valid_indices = np.all(dists_to_barriers > safety_margin, axis=1)

        cell_points = np.array(cell_points)[valid_indices]

        ###########################################
        # find barrier point with minimum distance from robot position to barriers
        dists_pi_to_barriers = np.linalg.norm(self.current_position - barrier_positions,
                                              axis=1)
        min_dist_index = np.argmin(dists_pi_to_barriers)
        self.min_dist = dists_pi_to_barriers[min_dist_index] # für debugging und auswertung

        ###########################################
        barrier_point_closest = barrier_positions[min_dist_index]

        # now filter out all points that are behind that barrier point (encumbrance barrier)
        index = []

        dx = self.current_position[0] - barrier_point_closest[0]
        dy = self.current_position[1] - barrier_point_closest[1]

        if abs(dx) < 0.001:
            dx = 0.001  # avoid division by zero
        if abs(dy) < 0.001:
            dy = 0.001  # avoid division by zero

        m = dy / dx  # slope

        if abs(m) < 0.001:
            m = 0.001  # avoid division by zero

        # coordinates of middle point
        xm = (self.current_position[0] + barrier_point_closest[0]) / 2
        ym = (self.current_position[1] + barrier_point_closest[1]) / 2

        # length of that vector
        dm = np.linalg.norm(
            [xm - self.current_position[0], ym - self.current_position[1]])

        # r < 1/2 dm --> r < 1/4 ||p_i - p_j|| --> delta_ij < 1/2 ||p_i - p_j||
        if dm < self.encumbrance + self.encumbrance_barriers_float:
            normal_ij = np.array([dx, dy]) / np.linalg.norm([dx, dy])
            solx = xm + (self.encumbrance + self.encumbrance_barriers_float - dm) * normal_ij[0]
            soly = ym + (self.encumbrance + self.encumbrance_barriers_float - dm) * normal_ij[1]

            # Geradengleichung für Trennungslinie der Zelle aufstellen und umstellen
            # y = - 1/m * (x - solx) + soly
            # y + 1/m * (x - solx) - soly = 0
            if self.current_position[1] + 1 / m * (
                    self.current_position[0] - solx) - soly > 0:
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
        cell_points = [
            point for k, point in enumerate(cell_points) if k not in index
        ]

        return cell_points #.tolist() # einbleinden wenn zusatzberechnung weg

    def compute_scalar_values(self, x_cell, y_cell, goal):
        x_cell = np.array(x_cell)
        y_cell = np.array(y_cell)

        dists_to_goal = np.linalg.norm(np.column_stack(
            (x_cell - goal[0], y_cell - goal[1])),
                                       axis=1)
        scalar_vals = np.exp(-dists_to_goal /
                             self.beta)  # beta is spreading factor rho
        return scalar_vals.tolist()

    def compute_error(self, **kwargs):
        centroid = kwargs.get('centroid', None)
        if centroid is None:
            centroid, _, _ = self.get_centroid()

        error = self.current_position - centroid
        return error

    def move(self):
        x, y = self.current_position
        error = self.compute_error()
        x_new = x + error[0] * self.dt
        y_new = y + error[1] * self.dt
        return np.array([x_new, y_new])
    
    def get_minimum_distance_to_barriers(self):
        return self.min_dist # for debugging and evaluation
    
    def get_cell_size(self):
        return self.cell_size # for debugging and evaluation


def applyrules(d1, d2, dt, beta_d, beta_min, beta, current_position, c1, c2,
               theta, final_goal, c1_no_rotation, goal_position):
    c1 = np.array(c1)  # centroid with neighbours
    current_position = np.array(current_position)

    dist_c1_c2 = np.linalg.norm(c1 - np.array(c2))
    alpha = np.pi / 2  # adjustable maximum angle offset
    # first condition for beta and theta update (theta is angle offset, just anpother way to represent rotation)
    if dist_c1_c2 > d2 and np.linalg.norm(current_position - c1) < d1:
        beta = max(beta - dt, beta_min)
        theta = min(theta + dt, alpha)
    # second condition
    else:
        beta = beta - dt * (beta - beta_d)
        theta = max(0, theta - dt)

    # third condition (p_z_temp = Rp_z and ||p_i-c1|| < ||p_i - c1_no_rotation||)
    if theta == alpha and np.linalg.norm(
            current_position -
            np.array(c1_no_rotation)) > np.linalg.norm(current_position - c1):
        theta = 0

    # compute the angle and new position
    angle = np.arctan2(final_goal[1] - current_position[1],
                       final_goal[0] - current_position[0])
    new_angle = angle - theta
    distance = np.linalg.norm((final_goal - current_position))

    goal_position[0] = current_position[0] + distance * math.cos(
        new_angle)  # new goalposition x
    goal_position[1] = current_position[1] + distance * math.sin(
        new_angle)  # new goalposition y
    # BlueRovs.destinations[i][0] = current_position

    return goal_position, beta, theta
