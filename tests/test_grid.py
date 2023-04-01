from robot_sim.grid import AABB, OccupancyGrid


def test_free_grid_has_no_occupied_cells():
    grid = OccupancyGrid(width=5, height=5, resolution=1.0)
    assert not grid.occupied.any()


def test_obstacle_marks_covered_cells_occupied():
    grid = OccupancyGrid(width=5, height=5, resolution=1.0,
                          obstacles=[AABB(1, 1, 3, 3)])
    assert grid.is_free((0, 0))
    assert not grid.is_free((1, 1))
    assert not grid.is_free((2, 2))
    assert not grid.is_free((3, 3))
    assert grid.is_free((4, 4))


def test_world_to_cell_and_back_round_trips_to_cell_center():
    grid = OccupancyGrid(width=10, height=10, resolution=0.5, origin=(-5, -5))
    cell = grid.world_to_cell(0.2, -1.3)
    x, y = grid.cell_to_world(*cell)
    # round-tripped center should be within one cell of the original point
    assert abs(x - 0.2) <= grid.resolution
    assert abs(y - (-1.3)) <= grid.resolution


def test_out_of_bounds_cell_is_not_free():
    grid = OccupancyGrid(width=2, height=2, resolution=1.0)
    assert not grid.is_free((-1, 0))
    assert not grid.is_free((0, 100))
