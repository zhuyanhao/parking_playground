"""Implement the object that defines a parking scenario."""

from .objects import *
from .vehicle import VehicleState
from .objects import convert_index_to_xy

from dataclasses import dataclass
from typing import List
import numpy as np
from shapely import Polygon, transform

@dataclass(frozen=True)
class ParkingScenarioParameters:
    """Defines the parameters of a parking scenario."""
    num_rows: int
    num_cols: int
    grid_size_m: float

    def __post_init__(self):
        """Check if all parameters are set correctly."""
        if self.num_rows < 1 or self.num_cols < 1:
            raise ValueError('The number of rows and columns must be positive.')
        
        if self.grid_size_m < 0.:
            raise ValueError('The grid size must be non-negative.')

class ParkingScenario:
    """Defines a parking scenario."""

    def __init__(self, params: ParkingScenarioParameters, ego_geometry: Polygon) -> None:
        """Initialize the scenario."""

        self._grid_size_m = params.grid_size_m
        # x-axis: row direction, y-axis: column direction
        self._map = np.zeros((params.num_rows, params.num_cols), dtype=int)
        self._objects: List[ParkingObject] = []
        self._ego_geometry: Polygon = ego_geometry
    
    def add_object(self, object: ParkingObject) -> None:
        """Add an object to the scenario and update its map."""
        self._objects.append(object)
        object.update_map(self._map, self._grid_size_m)

    def in_collision(self, ego_state: VehicleState) -> bool:
        """Check if the vehicle is in collision with any other objects.
        
        Args:
            ego_state: the current ego state/pose at which the collision is checked.
            
        Returns:
            True if in collision and False otherwise.
        """
        def transform_polygon(data: np.ndarray) -> np.ndarray:
            """Transform the polygon from ego frame to world frame."""
            data_T = data.T
            augmented_data = np.vstack([data_T, np.ones((1, data_T.shape[1]))])
            transformed_augmented_data = ego_state.to_matrix().dot(augmented_data)[:-1, :]
            return transformed_augmented_data.T

        transformed_ego_geometry = transform(self._ego_geometry, transform_polygon)

        num_rows, num_cols = self._map.shape
        for row in range(num_rows):
            for col in range(num_cols):
                cell_x, cell_y = convert_index_to_xy(row, col, self._grid_size_m)
                
                if self._map[row][col] > ObjectType.OBJECT_NO_OVERLAPPING_ALLOWED and transformed_ego_geometry.contains(Point(cell_x, cell_y)):
                    return True
        
        return False