"""Defines the interface and data structure used by the planner."""

from .vehicle import VehicleState, VehicleNode, VehicleProperties
from .scenario import ParkingScenario

from dataclasses import dataclass
from typing import Optional
import abc
from matplotlib import axes

class PlanningStartPose(VehicleState):
    """The planning start pose of the vehicle."""
    pass

@dataclass
class PlanningGoalPose:
    """The planning goal pose of the vehicle."""
    goal: VehicleState
    tolerance: VehicleState

    def __post_init__(self) -> None:
        """Check if the tolerance is non-negative."""
        if self.tolerance.x_m < 0 or self.tolerance.y_m < 0 or self.tolerance.yaw_rad < 0:
            raise ValueError('Tolerance must be non negative.')
        
    def in_goal_region(self, state: VehicleState) -> bool:
        """Check if the state is in the goal region."""
        return abs(state.x_m - self.goal.x_m) <= self.tolerance.x_m and \
            abs(state.y_m - self.goal.y_m) <= self.tolerance.y_m and \
            abs(state.yaw_rad - self.goal.yaw_rad) <= self.tolerance.yaw_rad
        
class Planner(abc.ABC):
    """The interface for a planner."""

    @abc.abstractmethod
    def plan(self, ego: VehicleProperties, start: VehicleState, goal: PlanningGoalPose, scenario: ParkingScenario) -> Optional[VehicleNode]:
        """Given the scenario and start & goal pose, return the last node of the optimal path if it exists."""
        return None
    
    @abc.abstractmethod
    def render(self, ax: axes.Axes, ego: VehicleProperties) -> None:
        """Render the planning process and result."""
        pass