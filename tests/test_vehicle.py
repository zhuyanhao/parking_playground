"""Test everything in vehicle.py."""

from ..vehicle import *
import pytest

def test_vehicle_properties():
    """Test if invalid values are captured by VehicleProperties class."""
    # negative wheelbase
    with pytest.raises(ValueError):
        VehicleProperties(
            wheelbase_m = -1.0,
            front_wheel_angle_limit_rad = 1.0,
            rear_wheel_angle_limit_rad = 1.0
        )

    # negative front wheel angle
    with pytest.raises(ValueError):
        VehicleProperties(
            wheelbase_m = 1.0,
            front_wheel_angle_limit_rad = -1.0,
            rear_wheel_angle_limit_rad = 1.0
        )

    # negative rear wheel angle
    with pytest.raises(ValueError):
        VehicleProperties(
            wheelbase_m = 1.0,
            front_wheel_angle_limit_rad = 1.0,
            rear_wheel_angle_limit_rad = -1.0
        )

def test_vehicle_kinematics_straight_line():
    """Test kinematics of vehicle when front_wheel_angle = rear_wheel_angle."""
    initial_state = VehicleState(
        x_m = 1.0,
        y_m = 5.0,
        yaw_rad = -math.pi / 2.0
    )
    properties = VehicleProperties(
        wheelbase_m = 3.0,
        front_wheel_angle_limit_rad = math.pi,
        rear_wheel_angle_limit_rad = math.pi
    )
    # drifting to the right by 5m
    input = VehicleInput(
        distance_moved_m = initial_state.x_m,
        front_wheel_angle_rad = -math.pi / 2.0,
        rear_wheel_angle_rad = -math.pi / 2.0
    )
    new_state = initial_state.step(properties, input)
    epsilon = 1.0e-6
    assert new_state.x_m == pytest.approx(0.0, epsilon)
    assert new_state.y_m == pytest.approx(initial_state.y_m, epsilon)
    assert new_state.yaw_rad == pytest.approx(initial_state.yaw_rad, epsilon)

def test_vehicle_kinematics_front_wheel_steer_only():
    """Test kinematics of vehicle when front wheel is steered only."""
    initial_state = VehicleState(
        x_m = 0.0,
        y_m = 0.0,
        yaw_rad = math.pi / 2.0
    )
    properties = VehicleProperties(
        wheelbase_m = 3.0,
        front_wheel_angle_limit_rad = math.pi / 4.0,
        rear_wheel_angle_limit_rad = math.pi / 4.0
    )
    wheel_angle = -math.pi / 8.0
    radius = math.fabs(properties.wheelbase_m / math.tan(wheel_angle))
    input = VehicleInput(
        distance_moved_m = math.pi * radius / 2.0,
        front_wheel_angle_rad = wheel_angle,
        rear_wheel_angle_rad = 0.0
    )
    new_state = initial_state.step(properties, input)
    epsilon = 1.0e-6
    assert new_state.x_m == pytest.approx(radius, epsilon)
    assert new_state.y_m == pytest.approx(radius, epsilon)
    assert new_state.yaw_rad == pytest.approx(0.0, epsilon)