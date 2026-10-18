# Arty-Part2

# Recent Additions:

```
sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm96-5
pip install PyGeodesy
```

Instructions for Setup:

## Ubuntu, ROS and Gazebo

#### 1.) Install Ubuntu 22.04.4 LTS from https://releases.ubuntu.com/jammy/ 
#### 2.) Update all packages using the command `sudo apt update && sudo apt upgrade`
#### 3.) Install ROS2 Humble (Ubuntu Binary Install) using this link https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html and Install all 3 packages using the command `sudo apt install ros-humble-desktop ros-humble-ros-base ros-dev-tools`
#### 4.) Add `source /opt/ros/humble/setup.bash` to your ~/.bashrc file
#### 5.) Follow the instructions [here](https://gazebosim.org/docs/garden/install_ubuntu) to install Gazebo Garden
#### 6.) Add the following line to your bashrc `export GZ_VERSION=garden`

## Ardupilot Install

#### 1.) First, make a directory called `ardupilot_ws` in your Home directory and make a `src` directory inside ardupilot_ws.
#### 2.) Ensure you have git installed. You can get it using `sudo apt install git`. Run the following commands.
#### 3.) `cd ~/ardupilot_ws/src`
#### 4.) `git clone https://github.com/ArduPilot/ardupilot/ --recurse-submodules`
#### 5.) `cd ardupilot`
#### 6.) `touch COLCON_IGNORE`
#### 7.) `git init`
#### 8.) `git checkout fd1bcc6`

Note that fd1bcc6 is the commit hash for release 4.5.4 of ArduCopter. You can refer to the Ardupilot Github for different releases.


## Ardupilot Setup

#### 1.) `sudo apt-get update && sudo apt-get install git && sudo apt-get install gitk git-gui`
#### 2.) `~/ardupilot_ws/src/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y`
#### 3.) Check your ~/.bashrc file. You will find a line that looks **something like** this that should have automatically been added

```source /home/user/ardupilot_ws/src/ardupilot/Tools/completion/completion.bash```

#### The following sections will vary slightly from user to user so DO NOT blindly copy paste commands

#### 4.) You also need to add ardupilot/Tools/autotest to path. Do that by adding the following line to your ~/.bashrc

```export PATH=$PATH:/home/user/ardupilot_ws/src/ardupilot/Tools/autotest```

### This line will change based on what your user account is called. If my user account is called artemis, then instead of `/home/user/...` I would need to write `/home/artemis/...` instead.

#### 5.) cd into the ardupilot directory using `cd ~/ardupilot_ws/src/ardupilot` 

#### 6.) waf (which is an executable) should be present. Test it out by using `./waf configure` and `./waf clean`

## MAVROS Install 

#### 1.) Run the following commands `sudo apt install ros-humble-mavros-msgs ros-humble-mavros-extras ros-humble-mavros`
#### 2.) `wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh`
#### 3.) `chmod u+x install_geographiclib_datasets.sh`
#### 4.) `sudo ./install_geographiclib_datasets.sh`

## REBOOT YOUR MACHINE

## ROS2 with MAVROS and SITL 

#### 1.) If you have followed all instructions correctly, you should be able to run ardupilot's SITL now. Try running this command `sim_vehicle.py -v ArduCopter --console --map`
#### 2.) You should see a line that looks something like 

```
SIM_VEHICLE: Run MavProxy
SIM_VEHICLE: "mavproxy.py" "--out" "127.0.0.1:14550" "--master" "tcp:127.0.0.1:5760" "--sitl" "127.0.0.1:5501" "--map" "--console"

```

#### 3.) The --out parameter specifies the UDP stream for MAVLINK which is where we will attach MAVROS
#### 4.) Open a new terminal, and run the following command. `ros2 launch mavros apm.launch fcu_url:=udp://:14550@`
#### 5.) You can now use `ros2 topic list` to see all available topics.
#### 6.) Play around with this. Using Mavproxy, make the drone move around. You can echo some ros2 topics and see position and acceleration data being sent on ros topics.

## Ardupilot Gazebo

#### 1.) Ardupilot Gazebo is a plugin to run simulations using Gazebo. You should already have Gazebo Garden installed if you have followed the instructions properly.
#### 2.) cd into your src directory. `cd ~/ardupilot_ws/src`
#### 3.) `git clone https://github.com/ArduPilot/ardupilot_gazebo`
#### 4.) `cd ardupilot_gazebo`
#### 5.) `git init && git checkout 0eaa3c6`
#### 6.) Run the following commands

```
sudo rosdep init
rosdep update
sudo apt update
sudo apt-get install ros-humble-ros-gzgarden
sudo apt install libgz-sim7-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
sudo bash -c 'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list'
rosdep update
cd ~/ardupilot_ws
rosdep resolve gz-garden
rosdep install --from-paths src --ignore-src -y
cd ~/ardupilot_ws/src/ardupilot_gazebo
touch COLCON_IGNORE
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

#### 7.) Add the following lines to your ~/.bashrc and Remember to replace the username. If my username is  instead of user, /home/user/ needs to be changed to /home/artemis/ in the following commands.

```
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/user/ardupilot_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH

export GZ_SIM_RESOURCE_PATH=/home/user/ardupilot_ws/src/ardupilot_gazebo/models:/home/user/ardupilot_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

#### 8.) For running simulations, add one final line to your bashrc
```
export GZ_SIM_RESOURCE_PATH=/home/user/ardupilot_ws/src/Arty-Part2/models:/home/user/ardupilot_ws/src/Arty-Part2/worlds:$GZ_SIM_RESOURCE_PATH
```

## Running the simulations

#### 1.) `cd ~/ardupilot_ws`
#### 2.) You can now build the artemis package using `colcon build --packages-select artemis`
#### 3.) `source install/setup.bash`
#### 4.) `ros2 launch artemis gazebo.launch.py` 

## This should launch MAVROS, Ardupilot SITL and `ros_gz_bridge` to stream image to a `/camera/image` ros2 topic.
## You can now manually fly around the simulation area.
