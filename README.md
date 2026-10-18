# Autonomous Drone Swarm 

This repository (`swarm_ws`) contains both the core ROS 2 simulation/API package (`swarm/`) and standalone Python mission scripts (`mission/`).

Because the `swarm` package is built once with `--symlink-install`, you can write, edit, and run mission scripts inside `mission/` directly using `python3 <script>.py` **without ever needing to run `colcon build` or edit `setup.py` again**.

## 1. Repository Structure

```text
/root/ardupilot_ws/src/swarm_ws/
├── mission/                   # Standalone mission & teleop scripts (ignored by colcon)
│   ├── COLCON_IGNORE
│   └── teleop.py
└── swarm/                     # ROS 2 package (built once via colcon --symlink-install)
    ├── config/
    │   └── swarm.yaml         # Default swarm parameters (num_drones, y_spacing)
    ├── launch/
    │   └── swarm.launch.py    # Launches Gazebo, ArduPilot SITL, MAVROS, & LiDAR bridges
    ├── models/                # Custom drone (iris_with_gimbal_custom + 2D LiDAR) & maze models
    ├── swarm/
    │   ├── __init__.py
    │   ├── drone_api.py       # Core Drone & Swarm Python API + 2D OccupancyGrid mapper
    │   └── drone.py
    ├── worlds/                # artemis_world.sdf (normal) & world_with_maze.sdf (maze)
    ├── package.xml
    └── setup.py
```

## 2. First-Time Setup 

Run this block once inside your Docker container (`~/ardupilot_ws`) to clean up any old swarm or mission directories, clone this repository into `src/swarm_ws`, and build the workspace:

```bash
# 1. Remove old folders and stale build/install caches
cd /root/ardupilot_ws/src
rm -rf swarm mission swarm_ws
cd /root/ardupilot_ws
rm -rf build/swarm install/swarm build/artemis install/artemis

# 2. Clone this repository into src/swarm_ws
cd /root/ardupilot_ws/src
git clone https://github.com/asvin24/swarm_ws.git swarm_ws
touch swarm_ws/mission/COLCON_IGNORE

# 4. Build once with --symlink-install and source the workspace
cd /root/ardupilot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select swarm
source ~/.bashrc
source /root/ardupilot_ws/install/setup.bash
```

### Pulling Updates Later

Because `--symlink-install` links directly to the source files, pulling new changes does not require rebuilding:

```bash
cd /root/ardupilot_ws/src/swarm_ws
git pull
```

## 3. Launching the Simulation

Use `swarm.launch.py` to start Gazebo Harmonic, spawn N drones in a straight line along the Y-axis, start ArduCopter SITL instances, launch MAVROS, and bridge the 2D LiDAR topics.

### Launch Arguments

| Argument | Default | Options / Description |
| --- | --- | --- |
| `world` | `normal` | `normal` (`artemis_world.sdf`) or `maze` (`world_with_maze.sdf`) |
| `num_drones` | `1` | Integer (1, 2, 3, ...) — spawns drones in a line along Y |
| `y_spacing` | `2.0` | Distance in meters between spawned drones along the Y-axis |

### Example Launch Commands

Single drone in the normal world (default):

```bash
ros2 launch swarm swarm.launch.py
```

Single drone in the maze world:

```bash
ros2 launch swarm swarm.launch.py world:=maze
```

Two drones in the maze world spaced 3 meters apart:

```bash
ros2 launch swarm swarm.launch.py world:=maze num_drones:=2 y_spacing:=3.0
```

**Note:** Wait until you see `[drone0.mavros.mavros]: CON: Got HEARTBEAT, connected. FCU: ArduPilot` in the launch terminal before starting your mission script.

### Cleaning Up Stuck Processes

If Gazebo or SITL is closed abruptly, clear any leftover background processes before relaunching:

```bash
pkill -9 -f arducopter; pkill -9 -f sim_vehicle; pkill -9 -f mavros_node; pkill -9 -f "gz sim"
```
