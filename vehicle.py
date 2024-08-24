"""Defines properties, states, inputs and anything relevant to vehicle."""

from dataclasses import dataclass
from typing import Optional, List
import math

@dataclass(frozen=True)
class VehicleProperties:
    """Defines all physical properties of an vehicle.
    
    The values set here affects the dynamics as well as the search space
    of the planner.
    """
    wheelbase_m: float = 2.5
    front_wheel_angle_limit_rad: float = math.radians(30)
    rear_wheel_angle_limit_rad: Optional[float] = None

    def __post_init__(self):
        """Check if all properties are set correctly.
        
        Raises:
            ValueError if any of the property is invalid.
        """
        if (self.wheelbase_m < 0.0):
            raise ValueError('Wheelbase must be positive.')
        
        if (self.front_wheel_angle_limit_rad < 0.0):
            raise ValueError('Front wheel angle limit must be positive.')
        
        if (self.rear_wheel_angle_limit_rad is not None and self.rear_wheel_angle_limit_rad < 0.0):
            raise ValueError('Rear wheel angle limit must be positive.')
        
@dataclass(frozen=True)
class VehicleState:
    """Defines the state of an vehicle that supports rear wheel steering."""
    x_m: float = 0.0  # X coordinate of the control point (center of rear axle) in world frame.
    y_m: float = 0.0  # Y coordinate of the control point (center of rear axle) in world frame.
    yaw_rad: float = 0.0  # Yaw angle of the vehicle in world frame.

@dataclass(frozen=True)
class VehicleInput:
    """Defines the input of an vehicle that supports rear wheel steering."""
    distance_moved_rad: float = 0.0
    front_wheel_angle_rad: float = 0.0
    rear_wheel_angle_rad: Optional[float] = None

@dataclass(frozen=False)  # This class is made mutable as the children node will be updated in MCTS.
class VehicleNode:
    """Defines a node in the search graph."""
    state: VehicleState
    input: Optional[VehicleInput]
    parent: Optional['VehicleNode'] = None
    children: Optional[List['VehicleNode']] = None
