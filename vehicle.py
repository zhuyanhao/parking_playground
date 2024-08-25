"""Defines properties, states, inputs and anything relevant to vehicle."""

from dataclasses import dataclass
from typing import Optional, List
import math
import numpy as np

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
class VehicleInput:
    """Defines the input of an vehicle that supports rear wheel steering."""
    distance_moved_m: float = 0.0
    front_wheel_angle_rad: float = 0.0
    rear_wheel_angle_rad: float = 0.0

@dataclass
class SE2:
    """Rigid Transformation in 2D space."""
    x_m: float = 0.0  # X coordinate of the control point in world frame.
    y_m: float = 0.0  # Y coordinate of the control point in world frame.
    yaw_rad: float = 0.0  # Yaw angle in world frame.

    def to_matrix(self) -> np.ndarray:
        """Returns a matrix representation of the SE2 transformation."""

        return np.array([
            [np.cos(self.yaw_rad), -np.sin(self.yaw_rad), self.x_m],
            [np.sin(self.yaw_rad), np.cos(self.yaw_rad), self.y_m],
            [0, 0, 1]
        ])
    
    @staticmethod
    def create_from_matrix(matrix: np.ndarray) -> 'SE2':
        """Creates a SE2 from a matrix representation."""

        if matrix.shape!= (3, 3):
            raise ValueError('Matrix must be 3x3.')

        if not np.allclose(matrix[2], [0., 0., 1.], atol=1e-3):
            raise ValueError('Matrix must be homogeneous.')
        
        return SE2(
            x_m=matrix[0][2].item(), 
            y_m=matrix[1][2].item(), 
            yaw_rad=np.arctan2(matrix[1][0], matrix[0][0]).item()
        )

class VehicleState(SE2):
    """Defines the state of an vehicle. The control point is at the center of rear axle."""

    def step(self, property: VehicleProperties, input: VehicleInput) -> 'VehicleState':
        """Forward simulate a kinematic bicycle model that supports rear wheel steering."""
        clamped_input = VehicleInput(
            distance_moved_m = input.distance_moved_m,
            front_wheel_angle_rad = np.clip(input.front_wheel_angle_rad, -property.front_wheel_angle_limit_rad, property.front_wheel_angle_limit_rad).item(),
            rear_wheel_angle_rad = np.clip(input.rear_wheel_angle_rad, -property.rear_wheel_angle_limit_rad, property.rear_wheel_angle_limit_rad).item(),
        )

        # first compute the instantaneous curvature of the vehicle
        kappa = (math.tan(clamped_input.front_wheel_angle_rad) - math.tan(clamped_input.rear_wheel_angle_rad)) / property.wheelbase_m
    
        # if kappa ~= 0, then the vehicle is moving along a straight line
        if math.isclose(kappa, 0.0, abs_tol=1e-6):
            motion_wrt_ego = SE2(
                x_m = input.distance_moved_m * math.cos(clamped_input.front_wheel_angle_rad),
                y_m = input.distance_moved_m * math.sin(clamped_input.front_wheel_angle_rad),
                yaw_rad = 0.0
            )
            state = SE2.create_from_matrix(self.to_matrix().dot(motion_wrt_ego.to_matrix()))
            state.__class__ = VehicleState  # cast to child class
            return state
        
        # if kappa != 0, compute vehicle motion by instantaneous rotation center
        ego2center = SE2(
            x_m = -math.tan(clamped_input.rear_wheel_angle_rad) / kappa,
            y_m = 1.0 / kappa,
            yaw_rad = -math.pi / 2.0 + clamped_input.rear_wheel_angle_rad
        )
        rotation = SE2(
            x_m = 0.0,
            y_m = 0.0,
            yaw_rad = clamped_input.distance_moved_m * kappa
        )
        state = SE2.create_from_matrix(self.to_matrix().dot(ego2center.to_matrix()).dot(rotation.to_matrix()).dot(np.linalg.inv(ego2center.to_matrix())))
        state.__class__ = VehicleState   # cast to child class
        return state

@dataclass(frozen=False)  # This class is made mutable as the children node will be updated in MCTS.
class VehicleNode:
    """Defines a node in the search graph."""
    state: VehicleState
    input: Optional[VehicleInput]
    parent: Optional['VehicleNode'] = None
    children: Optional[List['VehicleNode']] = None
