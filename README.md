# robot-sim

A small differential-drive robot simulator: plan a path across an arena full of
obstacles with A*, then drive it in PyBullet. A hobby project, callback to Robocon.

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
python main.py                 # opens the PyBullet GUI (needs a display)
python main.py --headless      # runs with p.DIRECT, saves PNG frames to out/
python main.py --arena maze    # pick a different hardcoded arena
```

## Test

```bash
pytest tests/
```

## Layout

- `robot_sim/grid.py` — occupancy grid: world coords <-> grid cells, obstacle rasterization
- `robot_sim/planner.py` — A* search over the grid
- `robot_sim/controller.py` — unicycle kinematics, waypoint-following controller
- `robot_sim/world.py` — PyBullet scene: ground, obstacles, robot body, camera
- `robot_sim/visualize.py` — matplotlib top-down preview of grid + path
- `robot_sim/simulation.py` — wires everything into a run loop
- `main.py` — CLI entry point

## Roadmap

- Swap the kinematic robot body for a full URDF diff-drive (e.g. `pybullet_data`'s `husky.urdf`)
- RRT / RRT* as an alternative global planner
- Reactive local avoidance (DWA or potential fields) for obstacles unknown at planning time
