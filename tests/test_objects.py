"""Test different types of objects defined in objects.py"""

from ..objects import *

def test_parked_car():
    """Check if the map is properly updated by ParkedCar object."""
    n = 4
    parking_map = np.zeros((n, n), dtype=int)
    grid_size = 1.0
    car = ParkedCar(bounding_box_m=Polygon([(2, 0), (2, 2), (4, 2), (4, 0)]))
    car.update_map(parking_map, grid_size)
    expected_map = np.array(
        [[  0,   0,   0,   0],
        [  0,   0,   0,   0],
        [101, 101,   0,   0],
        [101, 101,   0,   0]],
    )

    assert (parking_map == expected_map).all()