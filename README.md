<h1 align="center">UAV Autonomous Navigation <br> Pick and Place</h1>
<p align="center">
  <img src="https://github.com/MarcoMustacchi/UAV_Pick_and_Place/blob/master/VitaranaDrone.png" width="1440">
</p>

## Setup 
Ubuntu 20.04 with ROS Noetic using [Terminator](https://gnome-terminator.org/) as shell

## Table of contents
- Installation
- Notes: Wind
- Notes: Aruco Marker
- Notes: Gazebo
- How to setup PX4 toolchain development environment

## Installation
Assuming you have Ubuntu 20.04 installed, ROS Noetic installed and PX4 development environment set

##### Create a ROS workspace
```bash
mkdir -p ~/Desktop/catkin_ws/src
cd ~/Desktop/catkin_ws/src
catkin_init_workspace
```

##### Build the workspace
```bash
cd ~/Desktop/catkin_ws/
catkin build
```

##### Install and build package dependency "gazebo_ros_link_attacher"
```bash
cd ~/Desktop/catkin_ws/src
git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git
catkin build gazebo_ros_link_attacher
```

##### Clone repository
```bash
git clone https://github.com/MarcoMustacchi/UAV_Pick_and_Place.git
```

##### Move the repository content in the current directory
```bash
mv UAV_Pick_and_Place/* .
```

##### Remove directory and all its content
```bash
rm -rf UAV_Pick_and_Place
```

##### Build the packages
```bash
catkin build aruco_detector link_attacher topic_recorder 
```

##### Build the remaining package "offb" (it depends on "aruco_detector" pkg and "link_attacher" pkg)
```bash
catkin build offb
```

### Preliminary Step
##### Remember to add 
```bash
source /opt/ros/noetic/setup.bash
source ~/Desktop/catkin_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=~/Desktop/catkin_ws/src/offb/gazebo_models:$GAZEBO_MODEL_PATH
```
##### at the end of your **bashrc** file
```bash
gedit ~/.bashrc
```

#### How to start simulation
```bash
roslaunch offb custom_mavros_posix_sitl.launch
```
```bash
roslaunch offb nodes_simulation.launch
```

##### If you want to record the simulation using rosbag
```bash
roslaunch offb custom_mavros_posix_sitl.launch
```
```bash
roslaunch topic_recorder start_recording.launch
```
```bash
roslaunch offb nodes_simulation.launch
```

##### Define Setpoint for mission
```bash
rosparam set /uav_setpoint "{x: 3.0, y: 0.0, z: 2}"
```

##### you can see the result of the simulation using rosbag file with Plotjuggler
```bash
rosrun plotjuggler plotjuggler
```

## Notes: Wind
##### Added in .world file the wind from 30s to 35s in simulation time to test the PX4 controller

## Notes: Aruco Marker
##### Using the following website to get the texture.svg 
[https://chev.me/arucogen/](https://chev.me/arucogen/)
##### which then I need to convert as texture.png

## Notes: Gazebo
### Understanding Gazebo Model Path
##### Gazebo uses several default directories to search for models, which may not always be explicitly listed in the `GAZEBO_MODEL_PATH` environment variable. Here’s why you see additional model paths in the Gazebo interface that aren't shown when you echo the `GAZEBO_MODEL_PATH`:

1. **Default Paths in Gazebo:**
- **`~/.gazebo/models`**: This is the user's local model directory. Gazebo automatically checks this directory for models, even if it’s not included in the `GAZEBO_MODEL_PATH`.
- **`/usr/share/gazebo-11/models`**: This is a system-wide directory where Gazebo installs its default models. It’s also included by default, regardless of what’s set in the `GAZEBO_MODEL_PATH`.
- **`/opt/ros/noetic/share/.../models`**: When using ROS with Gazebo, certain ROS packages (like `turtlebot3_gazebo`) include their own models. These models are automatically included in Gazebo’s model search paths by ROS integration scripts, even if they’re not in the `GAZEBO_MODEL_PATH`.

2. **Implicit Paths**:
- Gazebo has certain hardcoded paths that it checks for models, even if these paths are not included in the `GAZEBO_MODEL_PATH` variable. These paths are typically the standard installation directories like `/usr/share/gazebo-11/models`.

3. **GAZEBO_MODEL_PATH**:
- The `GAZEBO_MODEL_PATH` environment variable is typically used to extend Gazebo’s default search paths with additional directories, particularly for custom models or models that are not installed in the default locations.

#### Viewing All Active Model Paths
##### If you want to see all the directories Gazebo is searching for models, including the defaults, you can do the following:
**In the Gazebo GUI**:
- Open Gazebo.
- Click on `Insert` in the left panel to see all available models. Hovering over or inspecting the models should show their respective paths.

#### **Finding the Current Gazebo Path**
##### You can check the current Gazebo paths by inspecting these environment variables:
- **`GAZEBO_MODEL_PATH`**: This is where Gazebo looks for model files.
- **`GAZEBO_RESOURCE_PATH`**: This is where Gazebo looks for world files, meshes, and other resources.
- **`GAZEBO_PLUGIN_PATH`**: This is where Gazebo looks for plugins.

##### To check the values of these environment variables, use the `echo` command:
```bash
echo $GAZEBO_MODEL_PATH
echo $GAZEBO_RESOURCE_PATH
echo $GAZEBO_PLUGIN_PATH
```

#### **Temporarily changing the Gazebo Path**
##### You can temporarily change the path by exporting the environment variable in your terminal session:
```bash
export GAZEBO_MODEL_PATH=/your/custom/path:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=/your/custom/resource/path:$GAZEBO_RESOURCE_PATH
export GAZEBO_PLUGIN_PATH=/your/custom/plugin/path:$GAZEBO_PLUGIN_PATH
```
##### To permanently change the path, add the export commands to your shell’s configuration file (`~/.bashrc`, `~/.bash_profile`, `~/.zshrc`, etc.), depending on which shell you use.

### Understanding Gazebo models
##### Gazebo uses the model's internal name to differentiate instances, and if two models have the same name, only one of them will be loaded.
You can add a `<name>` tag to each instance of the `small_box` model in your world file to give them unique names. 


## How to setup PX4 toolchain development environment
```bash
sudo apt update
sudo apt upgrade
mkdir ~/src
cd ~/src
git clone https://github.com/PX4/Firmware.git --recursive
cd ~/src/Firmware
bash ./Tools/setup/ubuntu.sh
```
##### Reboot computer
```bash
sudo reboot
```
##### Install dependencies
```bash
https://raw.githubusercontent.com/ktelegenov/scripts/main/ubuntu_sim_ros_noetic.sh
bash ubuntu_sim_ros_noetic.sh
```
##### this will create catwkin_ws workspace with already all packages dependencies
##### Close the terminal and open it again
##### Navigate to PX4 firmware folder and build it
```bash
cd ~/src/Firmware
git submodule update --init --recursive
DONT_RUN=1 make px4_sitl_default gazebo
```
##### Add PX4 firmware to ROS path and source it
```bash
source ~/src/Firmware/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/src/Firmware ~/src/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/Firmware/Tools/simulation/gazebo-classic/sitl_gazebo-classic
```
##### make sure to add the above inside the .bashrc file if you want to run it everytime from the terminal.\
```bash
roslaunch px4 multi_uav_mavros_sitl.launch
```
