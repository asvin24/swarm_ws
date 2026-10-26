# Swarm Workspace (`swarm_ws`)

ROS 2 swarm package for ArduPilot drones. The same API, launch structure and mission scripts run on both:

- **Simulation** on a PC (ArduPilot SITL + Gazebo Harmonic), and
- **Real hardware** on a Raspberry Pi connected to a Cube Orange+.

## Repositories

| Repo | Purpose | Needed for |
|---|---|---|
| [`asvin24/swarm_ws`](https://github.com/asvin24/swarm_ws) | Swarm API, launch files, mission scripts | Sim **and** real drone |
| [`asvin24/ardupilot_gazebo`](https://github.com/asvin24/ardupilot_gazebo) (branch `swarm_sim`) | Gazebo plugin, `iris_with_gimbal_custom` model, maze/artemis worlds | Sim only |
| [ArduPilot](https://github.com/ArduPilot/ardupilot) | SITL (`sim_vehicle.py`) | Sim only |

`ardupilot_gazebo` is a fork of the official ArduPilot repo and is **separate** from this one. The Raspberry Pi only needs `swarm_ws`.

## Workspace layout

```
~/ardupilot_ws/
└── src/
    ├── swarm_ws/            # this repo
    │   ├── swarm/           # ROS 2 package (API, launch files)
    │   └── mission/         # standalone mission scripts (ignored by colcon)
    ├── ardupilot_gazebo/    # sim only (branch: swarm_sim)
    └── ardupilot/           # sim only (SITL)
```

---

## Part A: Simulation setup (PC / Docker)

### Prerequisites

- Ubuntu 22.04 (or a Docker container based on it)
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- MAVROS for ROS 2 Humble
- ArduPilot SITL

Install MAVROS if you don't have it:

```bash
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

Install the libraries needed to build the Gazebo plugin:

```bash
sudo apt install libgz-sim8-dev rapidjson-dev libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

### 1. Clone the repositories

```bash
mkdir -p ~/ardupilot_ws/src
cd ~/ardupilot_ws/src

git clone https://github.com/asvin24/swarm_ws.git
git clone -b swarm_sim https://github.com/asvin24/ardupilot_gazebo.git
```

### 2. Install ArduPilot SITL

```bash
cd ~/ardupilot_ws/src
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

Make sure `sim_vehicle.py` is on your `PATH`. If it isn't, add this line to `~/.bashrc`:

```bash
export PATH=$HOME/ardupilot_ws/src/ardupilot/Tools/autotest:$PATH
```

### 3. Build the Gazebo plugin

```bash
cd ~/ardupilot_ws/src/ardupilot_gazebo
export GZ_VERSION=harmonic
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

### 4. Set Gazebo environment variables

```bash
echo 'export GZ_VERSION=harmonic' >> ~/.bashrc
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
source ~/.bashrc
```

### 5. Build the ROS 2 package

```bash
cd ~/ardupilot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select swarm
source install/setup.bash
```

### 6. Launch the simulation

```bash
ros2 launch swarm sim.launch.py world:=maze
```

Run this in a terminal where you have sourced `/opt/ros/humble/setup.bash` and `~/ardupilot_ws/install/setup.bash`.

---

## Part B: Raspberry Pi setup (real drone)

The Pi only needs `swarm_ws`. No Gazebo or SITL files are required.

### Prerequisites

- Raspberry Pi running Ubuntu 22.04 with [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- MAVROS installed (same commands as in Part A)
- Cube Orange+ connected to the Pi over USB (shows up as `/dev/ttyACM0`)

### 1. Clone the repository

```bash
mkdir -p ~/ardupilot_ws/src
cd ~/ardupilot_ws/src
git clone https://github.com/asvin24/swarm_ws.git
```

### 2. Build the package

```bash
cd ~/ardupilot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select swarm
```

### 3. Enable hardware wall-clock mode

This disables `use_sim_time` in `drone_api.py`, which is required on real hardware.

```bash
echo 'export SWARM_REAL_HW=1' >> ~/.bashrc
echo 'source ~/ardupilot_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 4. Give your user access to the serial port

```bash
sudo usermod -aG dialout $USER
```

Log out and back in (or reboot) for this to take effect.

### 5. Launch MAVROS and the TF tree

```bash
ros2 launch swarm real.launch.py drone_id:=0 fcu_url:=/dev/ttyACM0:921600
```

For additional drones, change `drone_id` (and `fcu_url` if the port differs).

---

## Mission scripts

Standalone scripts live in `mission/` (for example `square.py`, `teleop.py`, `boilerplate_auto.py`). They are not built by colcon (`mission/COLCON_IGNORE`). With the simulation or real launch file running, run a script from a sourced terminal:

```bash
cd ~/ardupilot_ws/src/swarm_ws/mission
python3 square.py
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `ros2 launch swarm ...` says package not found | Run `source ~/ardupilot_ws/install/setup.bash` |
| Gazebo can't find the ArduPilot plugin or models | Check `echo $GZ_SIM_SYSTEM_PLUGIN_PATH` points to `ardupilot_gazebo/build`, and that you cloned the `swarm_sim` branch |
| `sim_vehicle.py: command not found` | Add `ardupilot/Tools/autotest` to your `PATH` (Part A, step 2) |
| Drone/time-sync issues on the Pi | Confirm `echo $SWARM_REAL_HW` prints `1` |
| `Permission denied: /dev/ttyACM0` | Add yourself to `dialout` (Part B, step 4) and log in again |
| Git push asks for a password | Use a GitHub **personal access token** instead of your password |

## Notes for contributors

- Never commit build artifacts or SITL runtime files (`build/`, `install/`, `log/`, `*.bin`, `*.tlog`, and so on). The provided `.gitignore` covers them.
- Gazebo models and worlds belong in the `ardupilot_gazebo` fork (branch `swarm_sim`), not in this repo.