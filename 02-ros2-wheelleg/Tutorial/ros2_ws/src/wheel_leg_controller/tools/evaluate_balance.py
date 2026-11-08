#!/usr/bin/env python3
"""
Measure a stationary balance run without participating in control.

Run this in a third terminal after starting wheel_leg_sim and
wheel_leg_controller. It subscribes to the same topics, writes a CSV, and prints
a JSON summary after the requested simulation duration.
"""

import argparse
import csv
import json
import math

import rclpy
from rclpy.node import Node
from rm_msgs.msg import WheelLegCmd
from sensor_msgs.msg import Imu, JointState

from wheel_leg_controller.vendor.vmc_cal import five_links_param
from wheel_leg_controller.wheel_leg_controller import quaternion_to_euler


DT = 0.002
WHEEL_RADIUS = 0.06


class BalanceMonitor(Node):
    def __init__(self, duration, settle, output):
        super().__init__('balance_monitor')
        self.duration = duration
        self.settle = settle
        self.output = output
        self.left_leg = five_links_param(dt=DT)
        self.right_leg = five_links_param(dt=DT)
        self.imu = None
        self.samples = []
        self.commands = []
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.create_subscription(WheelLegCmd, 'actuator_cmd', self.command_callback, 10)
        self.get_logger().info(
            f'开始测量：目标 {duration:.1f} s 仿真时间，前 {settle:.1f} s 作为稳定期'
        )

    def imu_callback(self, msg):
        self.imu = msg

    def command_callback(self, msg):
        self.commands.append((
            msg.right_phi1, msg.right_phi4,
            msg.left_phi1, msg.left_phi4,
            msg.right_wheel, msg.left_wheel,
        ))

    def joint_callback(self, msg):
        if self.imu is None:
            return

        names = list(msg.name)
        position = dict(zip(names, msg.position))
        velocity = dict(zip(names, msg.velocity))
        if any(name not in position for name in (
                'right1', 'right2', 'left1', 'left2', 'rightwheel', 'leftwheel')):
            return

        roll, pitch, _ = quaternion_to_euler(
            self.imu.orientation.w,
            self.imu.orientation.x,
            self.imu.orientation.y,
            self.imu.orientation.z,
        )
        pitch = -pitch
        roll = -roll
        gyro_pitch = -self.imu.angular_velocity.y

        self.left_leg.forward_kinematics_cal(
            1, position['left1'], position['left2'], pitch, gyro_pitch)
        self.right_leg.forward_kinematics_cal(
            0, position['right1'], position['right2'], pitch, gyro_pitch)

        t = len(self.samples) * DT
        common_position = (position['leftwheel'] - position['rightwheel']) / 2.0
        self.samples.append({
            't': t,
            'pitch_deg': math.degrees(pitch),
            'roll_deg': math.degrees(roll),
            'left_L0': self.left_leg.L_0,
            'right_L0': self.right_leg.L_0,
            'wheel_common_m': common_position * WHEEL_RADIUS,
            'left_wheel_speed': velocity['leftwheel'],
            'right_wheel_speed': velocity['rightwheel'],
        })

        if t >= self.duration:
            self.finish()

    def finish(self):
        settled = [sample for sample in self.samples if sample['t'] >= self.settle]
        if not settled:
            self.get_logger().error('没有任何稳定期样本')
            rclpy.shutdown()
            return

        pitch_values = [sample['pitch_deg'] for sample in settled]
        leg_values = [
            value for sample in settled
            for value in (sample['left_L0'], sample['right_L0'])
        ]
        summary = {
            'duration_s': self.samples[-1]['t'],
            'settle_s': self.settle,
            'samples': len(self.samples),
            'max_abs_pitch_deg': max(abs(value) for value in pitch_values),
            'rms_pitch_deg': math.sqrt(sum(value * value for value in pitch_values)
                                       / len(pitch_values)),
            'final_pitch_deg': pitch_values[-1],
            'min_leg_length_m': min(leg_values),
            'max_leg_length_m': max(leg_values),
            'final_wheel_common_m': settled[-1]['wheel_common_m'],
            'max_abs_leg_torque_nm': (
                max(abs(value) for command in self.commands for value in command[:4])
                if self.commands else None
            ),
            'max_abs_wheel_torque_nm': (
                max(abs(value) for command in self.commands for value in command[4:])
                if self.commands else None
            ),
        }
        if self.output:
            with open(self.output, 'w', newline='', encoding='utf-8') as handle:
                writer = csv.DictWriter(handle, fieldnames=self.samples[0].keys())
                writer.writeheader()
                writer.writerows(self.samples)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--settle', type=float, default=5.0)
    parser.add_argument('--output', default='/tmp/balance_metrics.csv')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = BalanceMonitor(args.duration, args.settle, args.output)
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
