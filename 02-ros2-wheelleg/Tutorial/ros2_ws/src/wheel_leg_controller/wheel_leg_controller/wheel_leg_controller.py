"""
ROS 2 wheel-leg balance controller.

This package owns the control law. ``wheel_leg_sim`` owns MuJoCo and physics.
The two processes communicate only through ROS 2 topics:

    /imu, /joint_states -> this node -> /actuator_cmd
    /cmd_vel, /joy      -> this node (command source)

The simulator advances exactly one 2 ms physics step for each actuator command,
so this controller emits one command for each joint-state sample.
"""

import math
import time

from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from rm_msgs.msg import WheelLegCmd
from sensor_msgs.msg import Imu, JointState, Joy

from .vendor.climb_traj import VelocityRampFilter
from .vendor.lqr_update import lqr_controller
from .vendor.roll_leg_control import RollLegController
from .vendor.vmc_cal import five_links_param


JOINT_NAMES = ('right1', 'right2', 'left1', 'left2', 'rightwheel', 'leftwheel')


class PIDController:
    """
    Small PID copied from the team's original control program.

    The derivative term is an error difference, not divided by dt, because the
    original gains were tuned with that convention. Changing it here would
    silently alter every derivative gain.
    """

    def __init__(self, kp, ki, kd, max_out, max_i, feedforward=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.max_i = max_i
        self.feedforward = feedforward
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, current, target):
        error = target - current
        self.integral = float(np.clip(self.integral + error, -self.max_i, self.max_i))
        derivative = error - self.prev_error
        self.prev_error = error
        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
            + self.feedforward
        )
        return float(np.clip(output, -self.max_out, self.max_out))


def quaternion_to_euler(w, x, y, z):
    """Convert a (w, x, y, z) quaternion to roll, pitch and yaw."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def apply_deadzone(value, deadzone):
    """Return zero inside the joystick deadzone, otherwise keep the raw axis value."""
    if abs(value) <= deadzone:
        return 0.0
    return float(np.clip(value, -1.0, 1.0))


def joystick_velocity(axes, deadzone, linear_scale, angular_scale):
    """Map Xbox-style left/right stick axes to linear and angular velocity."""
    if len(axes) < 4:
        return 0.0, 0.0
    linear = -apply_deadzone(axes[1], deadzone) * linear_scale
    angular = -apply_deadzone(axes[3], deadzone) * angular_scale
    return linear, angular


def rate_limit(current, target, max_delta):
    """Move current toward target by at most max_delta in one control step."""
    if max_delta <= 0.0:
        return float(target)
    return float(current + np.clip(target - current, -max_delta, max_delta))


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class WheelLegController(Node):
    def __init__(self):
        super().__init__('wheel_leg_controller')

        self.declare_parameter('wheel_radius', 0.06)
        self.declare_parameter('target_length', 0.10)
        self.declare_parameter('max_velocity', 0.60)
        self.declare_parameter('leg_torque_limit', 1.80)
        self.declare_parameter('wheel_torque_limit', 2.00)
        self.declare_parameter('roll_kp', 1.0)
        self.declare_parameter('target_length_min', 0.10)
        self.declare_parameter('target_length_max', 0.14)
        self.declare_parameter('leg_length_step', 0.005)
        self.declare_parameter('joy_timeout', 0.50)
        self.declare_parameter('joy_deadzone', 0.10)
        self.declare_parameter('joy_linear_scale', 0.60)
        self.declare_parameter('joy_angular_scale', 2.00)
        self.declare_parameter('angular_acceleration', 6.00)
        self.declare_parameter('yaw_heading_kp', 2.00)
        self.declare_parameter('yaw_heading_limit', 0.60)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.base_length = float(self.get_parameter('target_length').value)
        self.max_velocity = float(self.get_parameter('max_velocity').value)
        self.leg_torque_limit = float(self.get_parameter('leg_torque_limit').value)
        self.wheel_torque_limit = float(self.get_parameter('wheel_torque_limit').value)
        self.target_length_min = float(self.get_parameter('target_length_min').value)
        self.target_length_max = float(self.get_parameter('target_length_max').value)
        self.leg_length_step = float(self.get_parameter('leg_length_step').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        self.joy_deadzone = float(self.get_parameter('joy_deadzone').value)
        self.joy_linear_scale = float(self.get_parameter('joy_linear_scale').value)
        self.joy_angular_scale = float(self.get_parameter('joy_angular_scale').value)
        self.angular_acceleration = float(self.get_parameter('angular_acceleration').value)
        self.yaw_heading_kp = float(self.get_parameter('yaw_heading_kp').value)
        self.yaw_heading_limit = float(self.get_parameter('yaw_heading_limit').value)
        self.dt = 0.002

        dt = self.dt
        self.right_leg = five_links_param(dt=dt)
        self.left_leg = five_links_param(dt=dt)
        self.right_lqr = lqr_controller(dt=dt)
        self.left_lqr = lqr_controller(dt=dt)

        self.left_yaw_pid = PIDController(0.5, 0.0, 1.0, 2.0, 0.0)
        self.right_yaw_pid = PIDController(0.5, 0.0, 1.0, 2.0, 0.0)
        self.left_length_pid = PIDController(700.0, 0.0, 5.0, 150.0, 0.0)
        self.right_length_pid = PIDController(700.0, 0.0, 5.0, 150.0, 0.0)
        self.roll_balancer = RollLegController(
            kp=float(self.get_parameter('roll_kp').value),
            ki=0.0,
            kd=0.0,
            max_out=0.14,
            min_leg=0.10,
            max_leg=0.14,
        )
        self.velocity_filter = VelocityRampFilter(dt=dt, accel_rate=0.8, decel_rate=1.4)

        self._latest_imu = None
        self._latest_joint_names = None
        self._latest_joint_position = None
        self._latest_joint_velocity = None
        self._cmd_vel_v = 0.0
        self._cmd_vel_w = 0.0
        self._joy_last_time = None
        self._joy_axes = []
        self._joy_buttons = []
        self._joy_target_length = self.base_length
        self._active_v = 0.0
        self._active_w = 0.0
        self._active_length = self.base_length
        self._ramped_v = 0.0
        self._ramped_w = 0.0
        self._yaw_setpoint = None
        self._steps = 0

        self.command_pub = self.create_publisher(WheelLegCmd, 'actuator_cmd', 10)
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info(
            f'轮腿控制器已启动: 目标腿长={self.base_length:.3f} m, '
            f'轮半径={self.wheel_radius:.3f} m'
        )
        self.get_logger().info('命令源优先级：/joy（0.5 s 内有效） > /cmd_vel')
        self.get_logger().info('等待 /imu 和 /joint_states；每个关节状态会推进一个仿真步')

    def imu_callback(self, msg):
        self._latest_imu = msg

    def cmd_vel_callback(self, msg):
        self._cmd_vel_v = float(np.clip(
            msg.linear.x, -self.max_velocity, self.max_velocity))
        self._cmd_vel_w = float(np.clip(msg.angular.z, -2.0, 2.0))

    def joy_callback(self, msg):
        now = time.monotonic()
        previous_buttons = self._joy_buttons
        self._joy_axes = list(msg.axes)
        self._joy_buttons = list(msg.buttons)
        self._joy_last_time = now

        button_actions = ((5, self.leg_length_step), (4, -self.leg_length_step))
        for index, delta in button_actions:
            pressed = index < len(self._joy_buttons) and self._joy_buttons[index] != 0
            was_pressed = index < len(previous_buttons) and previous_buttons[index] != 0
            if pressed and not was_pressed:
                self._joy_target_length = float(np.clip(
                    self._joy_target_length + delta,
                    self.target_length_min,
                    self.target_length_max,
                ))
                self.get_logger().info(
                    f'手柄目标腿长: {self._joy_target_length:.3f} m')

    def _update_command_reference(self):
        joy_active = (
            self._joy_last_time is not None
            and time.monotonic() - self._joy_last_time <= self.joy_timeout
            and len(self._joy_axes) >= 4
        )
        if joy_active:
            linear, angular = joystick_velocity(
                self._joy_axes,
                self.joy_deadzone,
                self.joy_linear_scale,
                self.joy_angular_scale,
            )
            self._active_v = float(np.clip(
                linear, -self.max_velocity, self.max_velocity))
            self._active_w = float(np.clip(angular, -2.0, 2.0))
            self._active_length = self._joy_target_length
        else:
            self._active_v = self._cmd_vel_v
            self._active_w = self._cmd_vel_w
            self._active_length = self.base_length

    def joint_callback(self, msg):
        if len(msg.name) != len(msg.position) or len(msg.name) != len(msg.velocity):
            self.get_logger().error('JointState 的 name/position/velocity 长度不一致')
            return

        self._latest_joint_names = msg.name
        self._latest_joint_position = msg.position
        self._latest_joint_velocity = msg.velocity

        if self._latest_imu is None:
            return
        self.step()

    def _joint_value(self, values, name):
        try:
            index = self._latest_joint_names.index(name)
        except ValueError:
            self.get_logger().error(f'JointState 缺少关节 {name}', throttle_duration_sec=1.0)
            return None
        return float(values[index])

    def step(self):
        positions = {
            name: self._joint_value(self._latest_joint_position, name)
            for name in JOINT_NAMES
        }
        velocities = {
            name: self._joint_value(self._latest_joint_velocity, name)
            for name in JOINT_NAMES
        }
        if any(value is None for value in positions.values()):
            return
        if any(value is None for value in velocities.values()):
            return

        imu = self._latest_imu
        w = float(imu.orientation.w)
        x = float(imu.orientation.x)
        y = float(imu.orientation.y)
        z = float(imu.orientation.z)
        roll, pitch, yaw = quaternion_to_euler(w, x, y, z)

        # The original project flips pitch/roll and the corresponding gyro axes.
        # Keep exactly this convention so the signs in the LQR and VMC code stay valid.
        pitch = -pitch
        roll = -roll
        gyro_pitch = -float(imu.angular_velocity.y)
        # The MuJoCo IMU reports body rates in the body frame: x=roll,
        # y=pitch, z=yaw. The yaw PID must use z, otherwise a ramp-induced
        # roll rate is fed back as a yaw command and the robot veers.
        gyro_yaw = float(imu.angular_velocity.z)

        left_wheel_pos = positions['leftwheel']
        right_wheel_pos = positions['rightwheel']
        left_wheel_speed = velocities['leftwheel']
        right_wheel_speed = velocities['rightwheel']
        common_position = (left_wheel_pos - right_wheel_pos) / 2.0
        common_speed = (left_wheel_speed - right_wheel_speed) / 2.0

        self._update_command_reference()
        self._ramped_v = self.velocity_filter.update(self._ramped_v, self._active_v)
        self._ramped_w = rate_limit(
            self._ramped_w,
            self._active_w,
            self.angular_acceleration * self.dt,
        )

        # Hold the heading only for straight-line driving. When the user asks
        # for a turn, the yaw setpoint follows the robot and the normal angular
        # rate command takes over.
        yaw_rate_target = self._ramped_w
        if self._yaw_setpoint is None or self._active_w != 0.0:
            self._yaw_setpoint = yaw
        elif self._active_v != 0.0:
            yaw_error = wrap_angle(self._yaw_setpoint - yaw)
            yaw_rate_target = float(np.clip(
                self.yaw_heading_kp * yaw_error,
                -self.yaw_heading_limit,
                self.yaw_heading_limit,
            ))
        elif self._active_v == 0.0:
            self._yaw_setpoint = yaw

        # While stationary, freeze x_b_set. Then x_b - x_b_set is the position
        # error that makes the LQR pull the robot back instead of letting it drift.
        if not (self._ramped_v == 0.0 and self._active_v == 0.0):
            x_b_set = common_position * self.wheel_radius
            self.left_lqr.x_b_set = x_b_set
            self.right_lqr.x_b_set = x_b_set

        self.left_lqr.v_set = self._ramped_v
        self.right_lqr.v_set = self._ramped_v

        target_left, target_right = self.roll_balancer.compute_leg_lengths(
            current_roll=roll,
            target_roll=0.0,
            base_length=self._active_length,
        )

        left_phi1, left_phi4, left_wheel = self._control_leg(
            leg='left',
            motor1=positions['left1'],
            motor2=positions['left2'],
            five_links=self.left_leg,
            lqr=self.left_lqr,
            yaw_pid=self.left_yaw_pid,
            length_pid=self.left_length_pid,
            pitch=pitch,
            gyro_pitch=gyro_pitch,
            gyro_yaw=gyro_yaw,
            target_w=yaw_rate_target,
            target_length=target_left,
            common_position=common_position,
            common_speed=common_speed,
        )
        right_phi1, right_phi4, right_wheel = self._control_leg(
            leg='right',
            motor1=positions['right1'],
            motor2=positions['right2'],
            five_links=self.right_leg,
            lqr=self.right_lqr,
            yaw_pid=self.right_yaw_pid,
            length_pid=self.right_length_pid,
            pitch=pitch,
            gyro_pitch=gyro_pitch,
            gyro_yaw=gyro_yaw,
            target_w=yaw_rate_target,
            target_length=target_right,
            common_position=common_position,
            common_speed=common_speed,
        )

        command = WheelLegCmd()
        command.right_phi1 = right_phi1
        command.right_phi4 = right_phi4
        command.left_phi1 = left_phi1
        command.left_phi4 = left_phi4
        command.right_wheel = right_wheel
        command.left_wheel = left_wheel
        self.command_pub.publish(command)

        self._steps += 1
        if self._steps % 2500 == 0:
            self.get_logger().info(
                f'已发送 {self._steps} 条力矩指令'
                f'（约 {self._steps * 0.002:.1f} s 仿真时间），'
                f'pitch={math.degrees(pitch):.2f}°，'
                f'v_cmd={self._active_v:.3f}，v_ramp={self._ramped_v:.3f}，'
                f'v_wheel={common_speed * self.wheel_radius:.3f} m/s'
            )

    def _control_leg(
        self,
        leg,
        motor1,
        motor2,
        five_links,
        lqr,
        yaw_pid,
        length_pid,
        pitch,
        gyro_pitch,
        gyro_yaw,
        target_w,
        target_length,
        common_position,
        common_speed,
    ):
        if leg == 'left':
            leg_flag = 1
            lqr_position = common_position
            lqr_speed = common_speed
        else:
            leg_flag = 0
            lqr_position = -common_position
            lqr_speed = -common_speed

        five_links.forward_kinematics_cal(leg_flag, motor1, motor2, pitch, gyro_pitch)
        lqr.states_update(
            leg_flag,
            five_links,
            pitch,
            gyro_pitch,
            lqr_position,
            lqr_speed,
            self.wheel_radius,
        )
        lqr.calc_k_matrix_from_poly(five_links.L_0)

        leg_force = length_pid.compute(five_links.L_0, target_length)
        cos_theta = math.cos(five_links.theta)
        feedforward = 4.0 / cos_theta if abs(cos_theta) > 1e-9 else 0.0
        leg_force = float(np.clip(feedforward + leg_force, -100.0, 100.0))

        pitch_torque = (
            lqr.K[1] * lqr.theta
            + lqr.K[3] * lqr.d_theta
            + lqr.K[5] * (lqr.x_b - lqr.x_b_set)
            + lqr.K[7] * (lqr.v_b_whole - lqr.v_set)
            + lqr.K[9] * lqr.phi
            + lqr.K[11] * lqr.d_phi
        )

        wheel_balance_torque = (
            lqr.K[0] * lqr.theta
            + lqr.K[2] * lqr.d_theta
            + lqr.K[4] * (lqr.x_b - lqr.x_b_set)
            + lqr.K[6] * (lqr.v_b_whole - lqr.v_set)
            + lqr.K[8] * lqr.phi
            + lqr.K[10] * lqr.d_phi
        )
        yaw_torque = yaw_pid.compute(gyro_yaw, target_w)

        torque_1, torque_4 = five_links.VMC_torque_cal(leg_force, pitch_torque)
        torque_1 = float(np.clip(torque_1, -self.leg_torque_limit, self.leg_torque_limit))
        torque_4 = float(np.clip(torque_4, -self.leg_torque_limit, self.leg_torque_limit))

        if leg == 'left':
            wheel_torque = -wheel_balance_torque - yaw_torque
            return -torque_1, -torque_4, float(np.clip(
                wheel_torque, -self.wheel_torque_limit, self.wheel_torque_limit))

        wheel_torque = wheel_balance_torque - yaw_torque
        return torque_1, torque_4, float(np.clip(
            wheel_torque, -self.wheel_torque_limit, self.wheel_torque_limit))


def main(args=None):
    rclpy.init(args=args)
    node = WheelLegController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
