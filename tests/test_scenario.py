"""Test everything in scenario.py."""

from ..scenario import *

import math
import pytest

def test_scenario_parameters():
    """Test if invalid values are captured by ParkingScenarioParameters class."""

    # negative num_rows
    with pytest.raises(ValueError):
        ParkingScenarioParameters(
            num_rows = -1, 
            num_cols = 3,
            grid_size_m = 0.04
        )

    # negative num_cols
    with pytest.raises(ValueError):
        ParkingScenarioParameters(
            num_rows = 3, 
            num_cols = -1,
            grid_size_m = 0.04
        )

    # negative grid_size
    with pytest.raises(ValueError):
        ParkingScenarioParameters(
            num_rows = 3, 
            num_cols = 3,
            grid_size_m = -0.04
        )

def test_collision_detection():
    """Test ParkingScenario.in_collision function."""
    
    scenario_params = ParkingScenarioParameters(
        num_rows = 4,
        num_cols = 4,
        grid_size_m = 1.0
    )
    ego_geometry = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
    scenario = ParkingScenario(scenario_params, ego_geometry)

    # map is empty, no collision
    ego_state1 = VehicleState(x_m = 0.0, y_m = 0.0, yaw_rad = 0.0)
    assert not scenario.in_collision(ego_state1)

    # add a parked car to the lower right corner
    parked_car = ParkedCar(bounding_box_m = Polygon([(3, 3), (3, 4), (4, 4), (4, 3)]))
    scenario.add_object(parked_car)

    # parked car added but not in collision
    assert not scenario.in_collision(ego_state1)

    # ego moved to the lower right corner and hence in collision
    ego_state2  = VehicleState(x_m  = 2.0, y_m  = 2.0, yaw_rad = math.pi / 16.0)
    assert scenario.in_collision(ego_state2)
