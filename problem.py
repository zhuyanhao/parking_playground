"""Defines the planning problem."""

from .planner import Planner, PlanningStartPose, PlanningGoalPose
from .scenario import ParkingScenario
from .vehicle import VehicleNode, VehicleProperties

from dataclasses import dataclass
from typing import Optional
import matplotlib.pyplot as plt

@dataclass(frozen=True)
class PlanningProblem:
    """The planning problem.
    
    This is where user should aggregate all ingredients, run the planner and retrieve the result.
    """
    ego: VehicleProperties
    scenario: ParkingScenario
    planner: Planner
    start_pose: PlanningStartPose
    goal_pose: PlanningGoalPose

    def solve(self) -> Optional[VehicleNode]:
        """Solves the planning problem."""
        return self.planner.plan(self.ego, self.start_pose, self.goal_pose, self.scenario)
    
    def render(self) -> None:
        """Render the scenario, the planning process and the solution."""
        plt.figure()
        ax = plt.gca()
        self.scenario.render(ax)
        self.planner.render(ax, self.ego)
        plt.show()
