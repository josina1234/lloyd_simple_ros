#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from hippo_control_msgs.msg import ActuatorSetpoint, PIDDebug, YawTarget
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf_transformations import euler_from_quaternion


class YawController(Node):
    def __init__(self):
        super().__init__(node_name='yaw_controller')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Position tracking for movement direction
        self.last_position = None
        self.current_position = None
        self.movement_direction = 0.0

        self.error_integral = 0.0
        self.got_first_state = False
        self.got_first_setpoint = True  # Changed: always consider we have a setpoint
        self.last_time = self.get_clock().now()
        self.last_yaw = 0.0
        self.last_derror = 0.0

        self.declare_parameters(
            namespace='',
            parameters=[
                ('gains.p', rclpy.Parameter.Type.DOUBLE),
                ('gains.i', rclpy.Parameter.Type.DOUBLE),
                ('gains.d', rclpy.Parameter.Type.DOUBLE),
                ('filter_gain', rclpy.Parameter.Type.DOUBLE),
                ('min_movement_threshold', rclpy.Parameter.Type.DOUBLE),  # Added parameter
                ('init_yaw', rclpy.Parameter.Type.DOUBLE),  # Added parameter
            ],
        )
        param = self.get_parameter('gains.p')
        self.get_logger().info(f'{param.name}={param.value}')
        self.K_p = param.value

        param = self.get_parameter('gains.i')
        self.get_logger().info(f'{param.name}={param.value}')
        self.K_i = param.value

        param = self.get_parameter('gains.d')
        self.get_logger().info(f'{param.name}={param.value}')
        self.K_d = param.value

        param = self.get_parameter('filter_gain')
        self.get_logger().info(f'{param.name}={param.value}')
        self.alpha = param.value

        param = self.get_parameter('min_movement_threshold')
        self.min_movement_threshold = param.value

        param = self.get_parameter('init_yaw')
        self.setpoint = param.value
        self.get_logger().info(f'Initial yaw setpoint: {self.setpoint} rad')
        self.get_logger().info(f'vorheriger setpoint: {math.pi/2} rad')

        # default value for the yaw setpoint
        self.setpoint_timed_out = False  # Changed: always have a valid setpoint
        self.dsetpoint = 0.0
        self.last_setpoint_time = self.get_clock().now()

        self.add_on_set_parameters_callback(self.on_params_changed)

        self.pid_debug_pub = self.create_publisher(
            PIDDebug, '~/pid_debug', qos_profile=1
        )

        self.vision_pose_sub = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic='vision_pose_cov',
            callback=self.on_vision_pose,
            qos_profile=qos,
        )

        self.torque_pub = self.create_publisher(
            msg_type=ActuatorSetpoint, topic='torque_setpoint', qos_profile=1
        )
        
        # Remove setpoint timeout timer - not needed anymore
        self.state_timeout_timer = self.create_timer(0.5, self.on_state_timeout)

    def on_params_changed(self, params):
        param: rclpy.Parameter
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            if param.name == 'gains.p':
                self.K_p = param.value
            elif param.name == 'gains.i':
                self.error_integral = 0.0
                self.K_i = param.value
            elif param.name == 'gains.d':
                self.K_d = param.value
            elif param.name == 'filter_gain':
                self.alpha = param.value
            elif param.name == 'min_movement_threshold':
                self.min_movement_threshold = param.value
            else:
                continue
        return SetParametersResult(successful=True, reason='Parameter set')

    def on_state_timeout(self):
        self.state_timeout_timer.cancel()
        self.get_logger().warn('yaw state timed out. waiting for states.')
        self.last_yaw = None

    # Remove on_setpoint_timeout method - not needed

    def wrap_pi(self, value: float):
        """Normalize the angle to the range [-pi; pi]."""
        if (-math.pi < value) and (value < math.pi):
            return value
        range = 2 * math.pi
        num_wraps = math.floor((value + math.pi) / range)
        return value - range * num_wraps

    # Remove on_setpoint method - not needed

    def calculate_movement_direction(self, current_pos):
        """Calculate movement direction from position changes"""
        if self.last_position is None:
            return None
            
        dx = current_pos[0] - self.last_position[0]
        dy = current_pos[1] - self.last_position[1]
        
        # Only update direction if we moved significantly
        movement_distance = math.sqrt(dx*dx + dy*dy)
        if movement_distance < self.min_movement_threshold:
            return None
            
        # Calculate direction angle and rotate by 90 degrees for robot's x-axis
        direction = math.atan2(dy, dx) #- math.pi/2
        return self.wrap_pi(direction)

    def on_vision_pose(self, msg: PoseWithCovarianceStamped):
        self.state_timeout_timer.reset()
        
        # Extract position
        pos = msg.pose.pose.position
        self.current_position = [pos.x, pos.y, pos.z]
        
        # get the vehicle orientation expressed as quaternion
        q = msg.pose.pose.orientation
        # convert the quaternion to euler angles
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = self.wrap_pi(yaw)
        
        # Calculate movement direction and update setpoint
        movement_dir = self.calculate_movement_direction(self.current_position)
        if movement_dir is not None:
            old_setpoint = self.setpoint
            ############################################
            # self.setpoint = movement_dir
            # CHANGE HERRE FOR ADAPTIVE YAW SETPOINT BASED ON MOVEMENT DIRECTION
            ############################################
            dt = (rclpy.time.Time.from_msg(msg.header.stamp) - self.last_setpoint_time).nanoseconds * 1e-9
            self.dsetpoint = self.wrap_pi(self.setpoint - old_setpoint) / max(dt, 1e-6)
            self.last_setpoint_time = rclpy.time.Time.from_msg(msg.header.stamp)
        #     self.get_logger().info(f'Updated setpoint to movement direction: {self.setpoint:.3f} rad')
        
        # self.get_logger().info(f'Received yaw: {yaw:.3f} rad, setpoint: {self.setpoint:.3f} rad')
        
        if not self.got_first_state:
            self.got_first_state = True
            self.last_time = rclpy.time.Time.from_msg(msg.header.stamp)
            self.last_yaw = yaw
            self.last_position = self.current_position
            return

        timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
        control_output = self.compute_control_output(yaw, timestamp)
        self.publish_control_output(control_output, timestamp)
        
        # Update position history
        self.last_position = self.current_position

    def moving_average_filter(self, derror):
        return self.alpha * derror + (1 - self.alpha) * self.last_derror

    def compute_control_output(self, yaw: float, time_now: rclpy.time.Time):
        dt = (time_now - self.last_time).nanoseconds * 1e-9
        if (
            (not self.got_first_state)
            | (self.last_yaw is None)
            | (dt <= 0.0)
        ):
            self.last_time = time_now
            self.last_yaw = yaw
            return 0.0
        # very important: normalize the angle error!
        error = self.wrap_pi(self.setpoint - yaw)
        self.error_integral += dt * error
        dyaw = self.wrap_pi(yaw - self.last_yaw) / max(dt, 1e-6)
        derror = self.moving_average_filter(self.dsetpoint - dyaw)
        p_component = self.K_p * error
        d_component = self.K_d * derror
        i_component = self.K_i * self.error_integral
        torque = p_component + d_component + i_component

        msg = PIDDebug()
        msg.header.stamp = time_now.to_msg()
        msg.p_term = p_component
        msg.i_term = i_component
        msg.d_term = d_component
        msg.p_gain = self.K_p
        msg.i_gain = self.K_i
        msg.d_gain = self.K_d
        msg.error = error
        msg.derror = derror
        msg.output = torque
        # no integral limits. hashtag YOLO
        msg.i_max = math.nan
        msg.i_min = math.nan
        
        # self.pid_debug_pub.publish(msg)  # Actually publish the debug message

        self.last_time = time_now
        self.last_yaw = yaw
        self.last_derror = derror

        return torque

    def publish_control_output(
        self, control_output: float, timestamp: rclpy.time.Time
    ):
        msg = ActuatorSetpoint()
        msg.header.stamp = timestamp.to_msg()
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False  # yaw is the rotation around the vehicle's z axis

        msg.z = control_output
        self.torque_pub.publish(msg)


def main():
    rclpy.init()
    node = YawController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
