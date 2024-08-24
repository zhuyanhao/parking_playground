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
        