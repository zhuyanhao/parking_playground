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

def test_goal_region_check():
    """Check if PlanningGoalPose.in_goal_region is correctly implemented."""

    epsilon = 1e-3
    goal = PlanningGoalPose(
        goal = VehicleState(),
        tolerance = VehicleState(x_m = epsilon, y_m = epsilon, yaw_rad = epsilon)
    )

    assert goal.in_goal_region(goal.goal)

    # x not in range
    invalid_x = VehicleState(
        x_m = goal.goal.x_m + 2 * epsilon,
        y_m = goal.goal.y_m,
        yaw_rad = goal.goal.yaw_rad
    )
    assert not goal.in_goal_region(invalid_x)

    # y not in range
    invalid_y = VehicleState(
        x_m = goal.goal.x_m,
        y_m = goal.goal.y_m + 2 * epsilon,
        yaw_rad = goal.goal.yaw_rad
    )
    assert not goal.in_goal_region(invalid_x)

    # yaw not in range
    invalid_yaw = VehicleState(
        x_m = goal.goal.x_m,
        y_m = goal.goal.y_m,
        yaw_rad = goal.goal.yaw_rad + 2 * epsilon
    )
    assert not goal.in_goal_region(invalid_x)