"""Test everything in planner.py."""

from ..planner import *

import pytest

def test_planning_goal_pose():
    """Check if invalid tolerance is captured upon initialization."""
    # negative x tolerance
    with pytest.raises(ValueError):
        PlanningGoalPose(
            goal = VehicleState(),
            tolerance = VehicleState(x_m = -1.0, y_m = 1.0, yaw_rad = 1.0)
        )

    # negative y tolerance
    with pytest.raises(ValueError):
        PlanningGoalPose(
            goal = VehicleState(),
            tolerance = VehicleState(x_m = 1.0, y_m = -1.0, yaw_rad = 1.0)
        )

    # negative yaw tolerance
    with pytest.raises(ValueError):
        PlanningGoalPose(
            goal = VehicleState(),
            tolerance = VehicleState(x_m = 1.0, y_m = 1.0, yaw_rad = -1.0)
        )