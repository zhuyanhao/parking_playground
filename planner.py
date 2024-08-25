"""Defines the interface and data structure used by the planner."""

from .vehicle import VehicleState, VehicleNode
from .scenario import ParkingScenario

from dataclasses import dataclass
from typing import Optional
import abc

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
        
class Planner(abc.ABC):
    """The interface for a planner."""

    @abc.abstractmethod
    def plan(self, start: VehicleState, goal: PlanningGoalPose, scenario: ParkingScenario) -> Optional[VehicleNode]:
        """Given the scenario and start & goal pose, return the last node of the optimal path if it exists."""
        return None
    
    @abc.abstractmethod
    def render(self, scenario) -> None:
        """Render the planning process and result."""
        pass