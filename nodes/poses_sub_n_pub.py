#!/usr/bin/env python3

import rclpy


# subscribe to all visions_pos_cov topics expliciteley
# and republish them on a common topic with the robot id as part of the message
# (so that the lloyd path planner can use them)

