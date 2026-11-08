#!/usr/bin/env python3
"""
Drive the real controller through the optional command-tracking tests.

Modes:
  cmd_vel  - hold 0.3 m/s until wheel odometry travels 3 m, then stop.
  joy      - send a synthetic Xbox-style /joy command instead of /cmd_vel.
  priority - publish neutral /joy plus /cmd_vel; verify joy wins, then verify
             fallback to /cmd_vel after the joy timeout.

Simulation time is counted from /joint_states (one sample = 2 ms), not wall
time, so results are independent of MuJoCo rendering speed.
"""

import argparse
import csv
import json
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rm_msgs.msg import WheelLegCmd
from sensor_msgs.msg import Imu, JointState, Joy

from wheel_leg_controller.wheel_leg_controller import quaternion_to_euler


DT = 0.002
WHEEL_RADIUS = 0.06


class BonusEvaluator(Node):
    def __init__(self, mode, speed, distance, priority_time, recovery_time,
                 max_time, output):
        super().__init__('bonus_evaluator')
        self.mode = mode
        self.speed = speed
        self.distance_target = distance
        self.priority_time = priority_time
        self.recovery_time = recovery_time
        self.max_time = max_time
        self.output = output

        self.imu = None
        self.command = None
        self.samples = []
        self.start_position = None
        self.start_time = None
        self.stop_time = None
        self.priority_start_position = None
        self.priority_hold_start_position = None
        self.priority_probe_displacement = None
        self.priority_hold_displacement = None
        self.fallback_time_sim = None
        self.fallback_time_wall = None
        self.last_joy_wall_time = None
        self.wall_start = time.monotonic()
        self.max_distance = 0.0
        self.termination = 'distance_reached'
        self.finished = False
        self.last_log_second = -1

        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.create_subscription(WheelLegCmd, 'actuator_cmd', self.command_callback, 10)
        self.create_timer(0.05, self.publish_command)
        self.get_logger().info(
            f'加分项验收 mode={mode}, speed={speed:.2f} m/s, '
            f'distance={distance:.2f} m'
        )

    def imu_callback(self, msg):
        self.imu = msg

    def command_callback(self, msg):
        self.command = msg

    def publish_command(self):
        if self.finished:
            return
        sim_t = len(self.samples) * DT
        linear = 0.0
        angular = 0.0
        joy_linear_axis = 0.0
        send_joy = False

        if self.mode == 'cmd_vel':
            if sim_t >= 1.0:
                linear = self.speed
        elif self.mode == 'joy':
            send_joy = True
            if sim_t >= 1.0:
                joy_linear_axis = -self.speed / 0.60
        elif self.mode == 'priority':
            if sim_t < self.priority_time:
                send_joy = True
                linear = self.speed
                joy_linear_axis = 0.0
            else:
                linear = self.speed

        if self.start_time is not None and sim_t >= self.start_time:
            if self.stop_time is not None:
                linear = 0.0
                joy_linear_axis = 0.0
                if self.mode == 'joy':
                    send_joy = True
                if sim_t >= self.stop_time + self.recovery_time:
                    self.finish()
                    return
            elif self.distance >= self.distance_target:
                linear = 0.0
                joy_linear_axis = 0.0
                if self.mode == 'joy':
                    send_joy = True
                self.stop_time = sim_t
                self.get_logger().info(
                    f'达到 {self.distance:.3f} m，开始停车恢复')
            elif sim_t >= self.max_time:
                self.termination = 'timeout'
                self.finish()
                return

        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.twist_pub.publish(twist)

        if send_joy:
            joy = Joy()
            joy.axes = [0.0, float(joy_linear_axis), 0.0, 0.0]
            joy.buttons = []
            self.joy_pub.publish(joy)
            self.last_joy_wall_time = time.monotonic()

    @property
    def distance(self):
        if self.start_position is None or not self.samples:
            return 0.0
        last = self.samples[-1]['wheel_position_m']
        return abs(last - self.start_position)

    def joint_callback(self, msg):
        if self.imu is None or self.finished:
            return
        names = list(msg.name)
        position = dict(zip(names, msg.position))
        if 'leftwheel' not in position or 'rightwheel' not in position:
            return

        roll, pitch, yaw = quaternion_to_euler(
            self.imu.orientation.w,
            self.imu.orientation.x,
            self.imu.orientation.y,
            self.imu.orientation.z,
        )
        pitch = -pitch
        roll = -roll
        common_position = (position['leftwheel'] - position['rightwheel']) / 2.0
        wheel_position = common_position * WHEEL_RADIUS
        sim_t = len(self.samples) * DT
        wall_t = time.monotonic() - self.wall_start

        if self.mode == 'priority' and self.priority_start_position is None:
            self.priority_start_position = wheel_position
        if (self.mode == 'priority' and sim_t >= 1.0
                and self.priority_hold_start_position is None):
            self.priority_hold_start_position = wheel_position

        drive_enabled = (
            self.mode in ('cmd_vel', 'joy') and sim_t >= 1.0
        ) or (
            self.mode == 'priority' and sim_t >= self.priority_time
        )
        if drive_enabled and self.start_time is None:
            self.start_time = sim_t
            self.start_position = wheel_position
            if (self.mode == 'priority'
                    and self.priority_hold_start_position is not None):
                self.priority_hold_displacement = abs(
                    wheel_position - self.priority_hold_start_position)
            self.get_logger().info(
                f'驱动开始: t={sim_t:.2f} s, x={wheel_position:.3f} m')

        if self.mode == 'priority' and self.start_time is not None:
            if self.last_joy_wall_time is not None:
                since_joy = time.monotonic() - self.last_joy_wall_time
                # Sample just before the 0.5 s wall-clock joy timeout. A value
                # near zero means the neutral /joy message kept overriding
                # /cmd_vel even though /cmd_vel asked for 0.3 m/s.
                if (self.priority_probe_displacement is None
                        and 0.35 <= since_joy < 0.50):
                    self.priority_probe_displacement = self.distance
            if self.fallback_time_sim is None and self.distance >= 0.01:
                self.fallback_time_sim = sim_t
                if self.last_joy_wall_time is not None:
                    self.fallback_time_wall = (
                        time.monotonic() - self.last_joy_wall_time)

        if int(sim_t) != self.last_log_second and self.start_time is not None:
            self.last_log_second = int(sim_t)
            self.get_logger().info(
                f't={sim_t:.1f} s, x={self.distance:.2f} m, '
                f'pitch={math.degrees(pitch):.2f}°')

        self.samples.append({
            't': sim_t,
            'wall_t': wall_t,
            'pitch_deg': math.degrees(pitch),
            'roll_deg': math.degrees(roll),
            'yaw_deg': math.degrees(yaw),
            'gyro_x': float(self.imu.angular_velocity.x),
            'gyro_y': float(self.imu.angular_velocity.y),
            'gyro_z': float(self.imu.angular_velocity.z),
            'wheel_position_m': wheel_position,
            'distance_m': self.distance,
            'right_wheel_cmd': (
                self.command.right_wheel if self.command is not None else 0.0),
            'left_wheel_cmd': (
                self.command.left_wheel if self.command is not None else 0.0),
            'right_phi1_cmd': (
                self.command.right_phi1 if self.command is not None else 0.0),
            'right_phi4_cmd': (
                self.command.right_phi4 if self.command is not None else 0.0),
            'left_phi1_cmd': (
                self.command.left_phi1 if self.command is not None else 0.0),
            'left_phi4_cmd': (
                self.command.left_phi4 if self.command is not None else 0.0),
        })
        self.max_distance = max(self.max_distance, self.distance)
        if abs(pitch) > math.radians(45.0):
            self.termination = 'pitch_limit'
            self.finish()
            return

    def finish(self):
        if self.finished:
            return
        self.finished = True
        if not self.samples or self.start_time is None:
            summary = {
                'mode': self.mode,
                'termination': 'no_data',
                'success': False,
            }
            print(json.dumps(summary, ensure_ascii=False, indent=2))
            rclpy.shutdown()
            return
        drive_samples = [
            sample for sample in self.samples
            if self.start_time is not None and sample['t'] >= self.start_time
        ]
        pitch_values = [abs(sample['pitch_deg']) for sample in drive_samples]
        wheel_torques = [
            abs(sample[key]) for sample in drive_samples
            for key in ('right_wheel_cmd', 'left_wheel_cmd')
        ]
        leg_torques = [
            abs(sample[key]) for sample in drive_samples
            for key in ('right_phi1_cmd', 'right_phi4_cmd',
                        'left_phi1_cmd', 'left_phi4_cmd')
        ]
        distance_reached = self.max_distance >= self.distance_target - 0.05
        priority_success = True
        if self.mode == 'priority':
            priority_success = (
                self.priority_probe_displacement is not None
                and self.priority_probe_displacement < 0.05
                and self.fallback_time_sim is not None
                and self.fallback_time_wall is not None
                and 0.40 <= self.fallback_time_wall <= 2.00
                and distance_reached
            )
        summary = {
            'mode': self.mode,
            'requested_speed_m_s': self.speed,
            'distance_m': round(self.distance, 3),
            'max_distance_m': round(self.max_distance, 3),
            'drive_duration_s': round((self.stop_time or drive_samples[-1]['t'])
                                      - self.start_time, 3),
            'max_abs_pitch_deg': max(pitch_values),
            'max_abs_wheel_torque_nm': max(wheel_torques, default=0.0),
            'max_abs_leg_torque_nm': max(leg_torques, default=0.0),
            'final_pitch_deg': drive_samples[-1]['pitch_deg'],
            'priority_hold_displacement_m': self.priority_hold_displacement,
            'fallback_probe_displacement_m': self.priority_probe_displacement,
            'fallback_detected_after_priority_start_s_sim': (
                None if self.fallback_time_sim is None
                else round(self.fallback_time_sim - self.priority_time, 3)),
            'fallback_detected_after_last_joy_s_wall': (
                None if self.fallback_time_wall is None
                else round(self.fallback_time_wall, 3)),
            'termination': self.termination,
            'success': (
                self.termination == 'distance_reached'
                and distance_reached
                and priority_success
            ),
        }
        if self.output:
            with open(self.output, 'w', newline='', encoding='utf-8') as handle:
                writer = csv.DictWriter(handle, fieldnames=self.samples[0].keys())
                writer.writeheader()
                writer.writerows(self.samples)
            with open(self.output + '.json', 'w', encoding='utf-8') as handle:
                json.dump(summary, handle, ensure_ascii=False, indent=2)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('cmd_vel', 'joy', 'priority'),
                        default='priority')
    parser.add_argument('--speed', type=float, default=0.30)
    parser.add_argument('--distance', type=float, default=3.0)
    parser.add_argument('--priority-time', type=float, default=2.0)
    parser.add_argument('--recovery-time', type=float, default=2.0)
    parser.add_argument('--max-time', type=float, default=30.0)
    parser.add_argument('--output', default='')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = BonusEvaluator(
        args.mode,
        args.speed,
        args.distance,
        args.priority_time,
        args.recovery_time,
        args.max_time,
        args.output,
    )
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
