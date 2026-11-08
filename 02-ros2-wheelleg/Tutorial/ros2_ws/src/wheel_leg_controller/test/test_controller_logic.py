"""Unit tests for the controller's small, ROS-independent logic."""

import math
from pathlib import Path

from wheel_leg_controller.wheel_leg_controller import (
    apply_deadzone,
    joystick_velocity,
    PIDController,
    quaternion_to_euler,
    rate_limit,
)


def test_identity_quaternion_has_zero_euler_angles():
    roll, pitch, yaw = quaternion_to_euler(1.0, 0.0, 0.0, 0.0)
    assert math.isclose(roll, 0.0, abs_tol=1e-12)
    assert math.isclose(pitch, 0.0, abs_tol=1e-12)
    assert math.isclose(yaw, 0.0, abs_tol=1e-12)


def test_quaternion_rotation_about_y_axis_is_pitch():
    angle = math.radians(30.0)
    roll, pitch, yaw = quaternion_to_euler(
        math.cos(angle / 2.0), 0.0, math.sin(angle / 2.0), 0.0)
    assert math.isclose(roll, 0.0, abs_tol=1e-12)
    assert math.isclose(pitch, angle, abs_tol=1e-12)
    assert math.isclose(yaw, 0.0, abs_tol=1e-12)


def test_pid_output_and_integral_are_clipped():
    controller = PIDController(kp=10.0, ki=10.0, kd=0.0, max_out=2.0, max_i=0.5)
    assert controller.compute(current=0.0, target=1.0) == 2.0
    assert math.isclose(controller.integral, 0.5)


def test_joystick_deadzone_and_velocity_mapping():
    assert apply_deadzone(0.10, 0.10) == 0.0

    linear, angular = joystick_velocity(
        axes=[0.0, -0.50, 0.0, -0.50],
        deadzone=0.10,
        linear_scale=0.60,
        angular_scale=2.00,
    )
    assert math.isclose(linear, 0.30)
    assert math.isclose(angular, 1.00)


def test_rate_limit_respects_per_step_delta():
    assert rate_limit(0.0, 1.0, 0.02) == 0.02
    assert rate_limit(0.0, -1.0, 0.02) == -0.02
    assert rate_limit(0.5, 0.51, 0.02) == 0.51


def test_controller_does_not_import_mujoco():
    source = Path(__file__).parents[1] / 'wheel_leg_controller' / 'wheel_leg_controller.py'
    text = source.read_text(encoding='utf-8')
    assert 'import mujoco' not in text
    assert 'from mujoco' not in text
