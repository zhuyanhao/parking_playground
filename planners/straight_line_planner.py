"""A simple planner that can only sample forward motion."""

from .. import *

from typing import List
import numpy as np
from shapely import transform, Polygon
import math

class StraightLinePlanner(Planner):
    """A sample planner that simply steps froward until:

        1. collision detected
        2. exceed the maximum number of steps
    """
    def __init__(self, ds: float, max_num_steps: int) -> None:
        self._ds = ds
        self._max_num_steps = max_num_steps

    def plan(self, ego: VehicleProperties, start: VehicleState, goal: PlanningGoalPose, scenario: ParkingScenario) -> Optional[VehicleNode]:
        """Given the scenario and start & goal pose, return the last node of the optimal path if it exists."""
        
        num_iter = 0
        self._expanded_nodes: List[VehicleNode] = [VehicleNode(state = start)]

        while num_iter < self._max_num_steps and not goal.in_goal_region(self._expanded_nodes[-1].state):
            num_iter += 1
            next_state = self._expanded_nodes[-1].state.step(
                property = ego,
                input = VehicleInput(self._ds, 0.0, 0.0)
            )

            if scenario.in_collision(next_state, ego.geometry):
                break

            self._expanded_nodes.append(
                VehicleNode(
                    state = next_state,
                    parent = self._expanded_nodes[-1]
                )
            )

        return self._expanded_nodes[-1] if goal.in_goal_region(self._expanded_nodes[-1].state) else None
    
    def render(self, ax: axes.Axes, ego: VehicleProperties) -> None:
        """Render the planning process and result."""
        for idx, node in enumerate(self._expanded_nodes):
            
            if idx % 10 == 0 or idx == len(self._expanded_nodes) - 1:
                def transform_polygon(data: np.ndarray) -> np.ndarray:
                    """Transform the polygon from ego frame to world frame."""
                    data_T = data.T
                    augmented_data = np.vstack([data_T, np.ones((1, data_T.shape[1]))])
                    transformed_augmented_data = node.state.to_matrix().dot(augmented_data)[:-1, :]
                    return transformed_augmented_data.T

                transformed_ego_geometry = transform(ego.geometry, transform_polygon)
                ax.plot(*transformed_ego_geometry.exterior.xy, 'b')
