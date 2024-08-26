from ..straight_line_planner import *

def test_straight_line_planner():
    # plan a path from [10.0, 10.0] to [10.0, 55.0]
    ego = VehicleProperties(
        wheelbase_m = 3.0,
        geometry = Polygon([(-1, -1), (-1, 1), (3, 1), (3, -1)])
    )
    scenario = ParkingScenario(
        params = ParkingScenarioParameters(
            num_rows = 100,
            num_cols = 100,
            grid_size_m = 1.0
        )
    )
    planner = StraightLinePlanner(ds = 0.2, max_num_steps = 1000)
    start_pose = PlanningStartPose(x_m = 10.0, y_m = 10.0, yaw_rad = math.pi / 2.0)
    epsilon = 0.15
    goal_pose = PlanningGoalPose(
        goal = VehicleState(x_m = 10.0, y_m = 55.0, yaw_rad = math.pi / 2.0),
        tolerance = VehicleState(x_m = epsilon, y_m = epsilon, yaw_rad = epsilon)
    )
    problem = PlanningProblem(
        ego = ego,
        scenario = scenario,
        planner = planner,
        start_pose = start_pose,
        goal_pose = goal_pose
    )
    
    sol1 = problem.solve()
    assert sol1 is not None
    problem.render()  # uncomment to visualize the solution

    parked_car = ParkedCar(bounding_box_m = Polygon([(9, 40), (9, 46), (11, 46), (11, 40)]))
    scenario.add_object(parked_car)
    sol2 = problem.solve()
    assert sol2 is None
    # problem.render()   # uncomment to visualize the solution