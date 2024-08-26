"""Contains all objects that could possibly be present in a parking scenario."""

import abc
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Tuple
import numpy as np
from shapely import Point, Polygon
from matplotlib import axes

class ObjectType(IntEnum):
    """Enumeration of all possible object types.
    
    Objects with larger enum value will overwrite ones with smaller enum value.
    This allows us to render overlapping objects (parking spot vs parked cars) on
    one map.
    """
    ## Objects that doesn't need to be considered for collision avoidance.
    UNKNOWN = 0
    PARKING_SPOT = 1

    ## Objects that should must be considered for collision avoidance.
    OBJECT_NO_OVERLAPPING_ALLOWED = 100  # A placeholder for objects that are not allowed to overlap.
    CAR = 101

class ParkingObject(abc.ABC):
    """Abstract class for all objects that could be present in a parking scenario."""

    @abc.abstractmethod
    def get_object_type(self) -> ObjectType:
        """Return the type of the object."""
        return ObjectType.UNKNOWN

    @abc.abstractmethod
    def update_map(self, parking_map: np.ndarray, grid_size_m: float) -> None:
        """Update the semantic value of each grid cell in the given map.
        
        Args:
            map: a 2d map provided by ParkingScenario object.
            grid_size_m: the grid size of the map.
        """
        return
    
    @abc.abstractmethod
    def render(self, ax: axes.Axes) -> None:
        """Render the current object on the given matplotlib axes."""
        return

def convert_index_to_xy(row: int, col: int, grid_size_m: float) -> Tuple[float, float]:
    """Convert row and column index into the x&y coordinate of the grid center."""
    cell_x = (row + 0.5) * grid_size_m
    cell_y = (col + 0.5) * grid_size_m

    return cell_x, cell_y

@dataclass(frozen=True)
class ParkedCar(ParkingObject):
    """Object representing a parked car."""
    
    bounding_box_m: Polygon  # in world/map frame
    render_color = 'r'

    def get_object_type(self) -> ObjectType:
        """Return the type of the object."""
        return ObjectType.CAR

    def update_map(self, parking_map: np.ndarray, grid_size_m: float) -> None:
        """Update the semantic value of each grid cell in the given map.
        
        Args:
            map: a 2d map provided by ParkingScenario object.
            grid_size_m: the grid size of the map.
        """
        num_rows, num_cols = parking_map.shape
        for row in range(num_rows):
            for col in range(num_cols):
                cell_x, cell_y = convert_index_to_xy(row, col, grid_size_m)
                
                if self.bounding_box_m.contains(Point(cell_x, cell_y)):
                    parking_map[row][col] = max(parking_map[row][col], self.get_object_type())

    def render(self, ax: axes.Axes) -> None:
        """Render parked car on the given axis."""
        ax.fill(*self.bounding_box_m.exterior.xy, color=self.render_color)