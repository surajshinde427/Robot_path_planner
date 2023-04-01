from robot_sim.grid import AABB, OccupancyGrid
from robot_sim.planner import astar


def test_finds_straight_path_on_empty_grid():
    grid = OccupancyGrid(width=5, height=5, resolution=1.0)
    path = astar(grid, (0, 0), (4, 0))
    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (4, 0)


def test_path_routes_around_obstacle():
    # wall at column x=2 covering rows 0-3, leaving row 4 as a gap to route through
    grid = OccupancyGrid(width=5, height=5, resolution=1.0,
                          obstacles=[AABB(2, 0, 2, 3)])
    path = astar(grid, (0, 2), (4, 2))
    assert path is not None
    for cell in path:
        assert grid.is_free(cell)
    assert (2, 4) in path  # must pass through the gap


def test_returns_none_when_goal_is_walled_off():
    grid = OccupancyGrid(width=5, height=5, resolution=1.0,
                          obstacles=[AABB(2, 0, 2, 4)])  # full-height wall, blocks all rows
    path = astar(grid, (0, 2), (4, 2))
    # the wall spans the whole grid height, so no path should exist
    assert path is None


def test_returns_none_when_start_or_goal_is_occupied():
    grid = OccupancyGrid(width=3, height=3, resolution=1.0,
                          obstacles=[AABB(1, 1, 1, 1)])
    assert astar(grid, (1, 1), (2, 2)) is None
    assert astar(grid, (0, 0), (1, 1)) is None


def test_start_equals_goal_returns_single_cell_path():
    grid = OccupancyGrid(width=3, height=3, resolution=1.0)
    assert astar(grid, (1, 1), (1, 1)) == [(1, 1)]
