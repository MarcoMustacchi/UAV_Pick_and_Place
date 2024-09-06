<p align="center">
  <img src="https://github.com/MarcoMustacchi/MarcoMustacchi.github.io/blob/main/assets/img/icons/UniPD_logo.svg" width="150">
</p>

<h1 align="center">Computer Vision - Laboratory Activities <br> UniPd</h1>

## Setup 
Ubuntu 22.04 with ROS Noetic using [Terminator](https://gnome-terminator.org/) as shell

## Installation

##### Create a ROS workspace

```bash
mkdir -p ~/Desktop/catkin_ws/src
cd ~/~/Desktop/catkin_ws/src
catkin_init_workspace
```

##### Build the workspace
```bash
cd ~/Desktop/catkin_ws/
catkin build
```

##### Setting up a ROS package from Git
```bash
cd ~/Desktop/catkin_ws/src
```

##### Clone repository
```bash
git clone https://github.com/MarcoMustacchi/IntelligentRoboticsLabs.git
```

##### Install packages dependencies "Apriltag" and "Apriltag_ros"

```bash
git clone https://github.com/AprilRobotics/apriltag.git
```

```bash
git clone https://github.com/AprilRobotics/apriltag_ros.git
```

##### Navigate to catkin workspace
```bash
cd ~/Desktop/catkin_ws
```

##### Build the dependencies Packages
```bash
catkin build 
```

## Preliminary Step
Remember to add 
```bash
source /opt/ros/noetic/setup.bash
source ~/Desktop/catkin_ws/devel/setup.bash
```
at the end of your **bashrc** file
```bash
gedit ~/.bashrc
```



## Gazebo Models

you can either define the full model directly in the world file

or

you cab create the model with model.sdf and model.config and then simply include the model in the world file



#### Note:

Gazebo uses the model's internal name to differentiate instances, and if two models have the same name, only one of them will be loaded.

You can add a `<name>` tag to each instance of the `small_box` model in your world file to give them unique names. 



## Define Setpoint for mission

```bash
rosparam set /uav_setpoint "{x: 3.0, y: 0.0, z: 2}"
```



## Aruco Marker

Using the following website to get the texture.svg 

https://chev.me/arucogen/

which then i need to convert as texture.png



## Useful Linux commands

Open the current path in the terminal

```bash
nautilus .
```



## Understanding Gazebo Model Path

Gazebo uses several default directories to search for models, which may not always be explicitly listed in the `GAZEBO_MODEL_PATH` environment variable. Here’s why you see additional model paths in the Gazebo interface that aren't shown when you echo the `GAZEBO_MODEL_PATH`:

1. **Default Paths in Gazebo:**

   - **`~/.gazebo/models`**: This is the user's local model directory. Gazebo automatically checks this directory for models, even if it’s not included in the `GAZEBO_MODEL_PATH`.
   - **`/usr/share/gazebo-11/models`**: This is a system-wide directory where Gazebo installs its default models. It’s also included by default, regardless of what’s set in the `GAZEBO_MODEL_PATH`.
   - **`/opt/ros/noetic/share/.../models`**: When using ROS with Gazebo, certain ROS packages (like `turtlebot3_gazebo`) include their own models. These models are automatically included in Gazebo’s model search paths by ROS integration scripts, even if they’re not in the `GAZEBO_MODEL_PATH`.

2. **Implicit Paths**:

   - Gazebo has certain hardcoded paths that it checks for models, even if these paths are not included in the `GAZEBO_MODEL_PATH` variable. These paths are typically the standard installation directories like `/usr/share/gazebo-11/models`.

3. **GAZEBO_MODEL_PATH**:

   - The `GAZEBO_MODEL_PATH` environment variable is typically used to extend Gazebo’s default search paths with additional directories, particularly for custom models or models that are not installed in the default locations.

   ### Viewing All Active Model Paths

   If you want to see all the directories Gazebo is searching for models, including the defaults, you can do the following:

   **In the Gazebo GUI**:

   - Open Gazebo.
   - Click on `Insert` in the left panel to see all available models. Hovering over or inspecting the models should show their respective paths.

### **Finding the Current Gazebo Path**

You can check the current Gazebo paths by inspecting these environment variables:

- **`GAZEBO_MODEL_PATH`**: This is where Gazebo looks for model files.
- **`GAZEBO_RESOURCE_PATH`**: This is where Gazebo looks for world files, meshes, and other resources.
- **`GAZEBO_PLUGIN_PATH`**: This is where Gazebo looks for plugins.

To check the values of these environment variables, use the `echo` command:

```bash
echo $GAZEBO_MODEL_PATH
echo $GAZEBO_RESOURCE_PATH
echo $GAZEBO_PLUGIN_PATH
```

### **Temporarily changing the Gazebo Path**

You can temporarily change the path by exporting the environment variable in your terminal session:

```bash
export GAZEBO_MODEL_PATH=/your/custom/path:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=/your/custom/resource/path:$GAZEBO_RESOURCE_PATH
export GAZEBO_PLUGIN_PATH=/your/custom/plugin/path:$GAZEBO_PLUGIN_PATH
```

To permanently change the path, add the export commands to your shell’s configuration file (`~/.bashrc`, `~/.bash_profile`, `~/.zshrc`, etc.), depending on which shell you use.
