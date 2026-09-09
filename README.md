# SLAM Playground

`slam_playground` is a ROS 2 Jazzy learning project for simulating a moving
robot, generating idealized 2D laser scans, and building an occupancy grid.

The current implementation is an odometry-based mapper, not a complete SLAM
system. It uses the robot pose from simulated odometry to place laser endpoints
in a grid. It does not estimate or correct the pose from sensor observations.

## What is built

The `moving_robot.launch.py` launch file starts three nodes:

| Executable | Node name | Responsibility |
| --- | --- | --- |
| `robot_motion` | `robot_motion_node` | Publishes exact odometry for a robot following a circle and broadcasts the robot TF frames. |
| `laser_simulator` | `laser_simulator` | Casts 360 idealized rays against the walls of a square room. |
| `simple_mapper` | `simple_mapper` | Uses odometry and scan endpoints to accumulate occupied cells in an occupancy grid. |

The simulated robot moves on a circle with a radius of 1.5 m. The laser scans
an 8 m by 8 m square room at 5 Hz with a range of 0.1 m to 8.0 m. The mapper
publishes a 12 m by 12 m grid at 0.05 m per cell.

### ROS interfaces

| Topic | Type | Publisher | Subscriber | Purpose |
| --- | --- | --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | `robot_motion_node` | `laser_simulator`, `simple_mapper` | Exact simulated robot pose. |
| `/scan` | `sensor_msgs/msg/LaserScan` | `laser_simulator` | `simple_mapper` | Simulated 360-degree laser scan. |
| `/scan_rays` | `visualization_msgs/msg/Marker` | `laser_simulator` | — | Visualization of every tenth laser ray. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `simple_mapper` | — | Accumulated occupied scan endpoints. |

The implemented TF tree is:

```text
odom -> base_link -> laser_frame
```

`odom -> base_link` follows the simulated motion. `base_link -> laser_frame`
is static with zero translation and rotation. The occupancy grid is expressed
in `odom`; there is no `map` TF frame.

The mapper marks valid scan endpoints as occupied (`100`). Unobserved cells
remain unknown (`-1`). It does not mark free space along a ray.

## Build

Requirements:

- Ubuntu 24.04 and ROS 2 Jazzy
- `colcon` and `rosdep`
- `uv`
- Python 3.12

ROS dependencies are declared in `src/slam_playground/package.xml`. Additional
Python runtime and development dependencies are declared in `pyproject.toml`
and pinned reproducibly in `uv.lock`. The uv environment uses the system Python
interpreter so that it remains compatible with the Python packages supplied by
ROS 2, while uv-installed packages remain isolated from the global Python
installation.

From the repository root, install the ROS dependencies and create the local uv
environment once:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
export UV_PROJECT_ENVIRONMENT=slam_env
uv venv --python /usr/bin/python3 --system-site-packages slam_env
touch slam_env/COLCON_IGNORE
uv sync --locked
```

Activate the environment and build the ROS package:

```bash
source slam_env/bin/activate
source /opt/ros/jazzy/setup.bash
python /usr/bin/colcon build --symlink-install --packages-select slam_playground
source install/setup.bash
```

Invoking the system `colcon` script through the active environment's `python`
is intentional. It makes the installed ROS executables use `slam_env` and
therefore gives the nodes access to the dependencies installed by uv.

Do not install project dependencies with system-wide `pip`, `pip --user`, or
`uv pip --system`. Add or remove non-ROS Python dependencies with `uv add` and
`uv remove`; both commands update `pyproject.toml`, `uv.lock`, and `slam_env`.
Set `UV_PROJECT_ENVIRONMENT=slam_env` in the current shell before using these
commands. After pulling dependency changes, run `uv sync --locked` again.

## Run

In each new terminal, activate the uv environment before sourcing ROS 2 and the
built workspace:

```bash
export UV_PROJECT_ENVIRONMENT=slam_env
source slam_env/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Then launch the demo:

```bash
ros2 launch slam_playground moving_robot.launch.py
```

To run the nodes separately, source the environment in each terminal and start
the odometry publisher first:

```bash
ros2 run slam_playground robot_motion
ros2 run slam_playground laser_simulator
ros2 run slam_playground simple_mapper
```

For RViz 2, use `odom` as the fixed frame and add `/map`, `/scan`,
`/scan_rays`, and TF displays. No RViz configuration is included.

Useful inspection commands:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo odom base_link
```

## Tests

The package includes the standard ament Flake8, PEP 257, and copyright test
wrappers:

```bash
python /usr/bin/colcon test --packages-select slam_playground
python /usr/bin/colcon test-result --verbose
```

The copyright test is explicitly skipped. The Flake8 and PEP 257 tests pass.
No functional tests currently exercise the simulation or mapper.

## What remains to do

The tracked project work is maintained in [TODO.md](TODO.md). The main gaps are
functional tests, a less idealized sensor and motion model, free-space mapping,
and the pose-estimation and correction components required for actual 2D SLAM.

## Repository layout

```text
.
|-- README.md
|-- TODO.md
|-- pyproject.toml
|-- uv.lock
+-- src/slam_playground/
    |-- package.xml
    |-- setup.py
    |-- launch/moving_robot.launch.py
    |-- slam_playground/moving_robot/
    |   |-- robot_motion_node.py
    |   |-- laser_simulator_node.py
    |   +-- simple_mapper_node.py
    +-- test/
```

The generated `slam_env` directory contains the local Python environment and
is not committed. Its `COLCON_IGNORE` marker prevents `colcon` from inspecting
the environment as part of the workspace. `UV_PROJECT_ENVIRONMENT` tells uv to
manage this directory instead of its default `.venv` path.

## License

Apache License 2.0. See
[`src/slam_playground/LICENSE`](src/slam_playground/LICENSE).
