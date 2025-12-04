# zu übergebende Parameter:
# Raumdimensionen
# Hindernisdimensionen und Position
# Roboterdimensionen
# aktuelle Roboterpositionen

# update barriers

# dimensionen der barriers (step_size: 0.01m) (=1cm)
import numpy as np
# from bluerov_simulation import bluerov_simulation # avoid circular import


class barriers:

    def __init__(self, step_size, basin_limits_x, basin_limits_y,
                 obstacle1_limits_x, obstacle1_limits_y):
        self.step_size = step_size  # m adjustable kommt aus main.py "dx"
        self.lim_basin_x = np.array(basin_limits_x) #[0.0, 2.0]
        self.lim_basin_y = np.array(basin_limits_y) # [0.0, 4.0]
        self.lim_obs1_x = np.array(obstacle1_limits_x) # [0.75, 1.25]
        self.lim_obs1_y = np.array(obstacle1_limits_y) # [1.95, 2.05]

        # # für U-förmiges Hindernis:
        # self.lim_u_hori_x = np.array([0.0, 0.75])
        # self.lim_u_hori_y = np.array([1.95, 2.05])
        # self.lim_u_vert_x = np.array([0.75, 0.85])
        # self.lim_u_vert_y = np.array([2.05, 2.75])

    def get_walls(self, corners):
        walls = []
        num_corners = len(corners)
        for i in range(num_corners):
            start = corners[i]
            end = corners[(i + 1) %
                          num_corners]  # Wrap around to the first corner
            if start[0] == end[0]:  # Vertical wall (same x-coordinate)
                y_values = np.arange(min(start[1], end[1]),
                                     max(start[1], end[1]) + self.step_size,
                                     self.step_size)
                wall = np.column_stack((np.full_like(y_values,
                                                     start[0]), y_values))
            elif start[1] == end[1]:  # Horizontal wall, same y-coordinate
                x_values = np.arange(min(start[0], end[0]),
                                     max(start[0], end[0]) + self.step_size,
                                     self.step_size)
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
    
    def get_corners(self, x_lim, y_lim):
        corners = np.array([[x_lim[0], y_lim[0]],
                            [x_lim[1], y_lim[0]],
                            [x_lim[1], y_lim[1]],
                            [x_lim[0], y_lim[1]]])
        return corners


    def def_barriers(self):

        basin_corners = self.get_corners(self.lim_basin_x, self.lim_basin_y)
        basin_walls = self.get_walls(basin_corners)

        # ##### U Hindernis
        # u_obstacle_hori_corners = self.get_corners(self.lim_u_hori_x,
        #                                           self.lim_u_hori_y)
        # u_obstacle_vert_corners = self.get_corners(self.lim_u_vert_x,
        #                                           self.lim_u_vert_y)
        # u_obstacle_hori_walls = self.get_walls(u_obstacle_hori_corners)
        # u_obstacle_vert_walls = self.get_walls(u_obstacle_vert_corners)

        # obstacle_walls = np.vstack((u_obstacle_hori_walls,
        #                                 u_obstacle_vert_walls))

        # ####

        #___________________ Standard Hindernis

        obstacle_corners = self.get_corners(self.lim_obs1_x, self.lim_obs1_y)
        obstacle_walls = self.get_walls(obstacle_corners)
        #___________________



        return np.vstack((obstacle_walls, basin_walls))

    def get_limits(self):
        # return basinlimits, obstacle limits TODO

        ## ________________________Standard Hindernis
        return np.array([self.lim_basin_x, self.lim_basin_y
                         ]), np.array([self.lim_obs1_x, self.lim_obs1_y])
        ## ________________________Standard Hindernis Ende

        # ## U-förmiges Hindernis
        # return np.array([self.lim_basin_x, self.lim_basin_y
        #                  ]), np.array([[self.lim_u_hori_x, self.lim_u_hori_y],
        #                                [self.lim_u_vert_x, self.lim_u_vert_y]])
        # # U-förmiges Hindernis Ende

