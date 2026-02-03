# GelSight Mini ROS2 Wrapper
This package provides a ROS2 wrapper for the GelSight Mini tactile sensor, built on top of the [GelSight Robotics SDK](https://github.com/gelsightinc/gsrobotics.git)
.
## Setting up Your Environment
This setup has been tested with ROS2 Humble and Ubuntu 22.04.

This setup uses the simplest structure to avoid modifying anything inside the official gsrobotics repository while continuing to benefit from their future updates:
tactile_ws/
└── src/
    └── gelsight_mini_ros2/
        ├── gsrobotics/      # The official GelSight SDK repository
        └── gsmini_ros2/     # The ROS2 wrapper package
        └── sensing_utils/   # OPTIONAL: a repo used for easy usb camera integration and other things

You can set this up manually or follow the step-by-step instructions below.


1. Create a ROS2 workspace
If you don't already have a workspace, create one:

```bash
mkdir -p ~/projects/tactile_ws/src
```


2. Clone this repository
```bash
cd ~/projects/tactile_ws/src
git clone https://github.com/FabPrez/gelsight_mini_ros2.git
```


3. Import dependencies inside the gelsight_mini_ros2 folder:
The .repos file automatically downloads the official gsrobotics repository (and any other dependencies):
```bash
cd ~/projects/tactile_ws/src/gelsight_mini_ros2
vcs import < gsmini.repos
```

4. Install package dependencies
```bash
cd ~/projects/tactile_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

5. Build the workspace
```bash
colcon build --symlink-install
source install/setup.bash
```

6. create a virtual environment and install all the requiriments for the [GelSight Robotics SDK](https://github.com/gelsightinc/gsrobotics.git)
7. use the path of the virtual environemnt as an hard coded path for the launch file "simple_pub_hardcoded.launch.py"

## Path Configuration
To avoid touching the gsrobotics package, paths to reference that folder have been made relative where possible. However, relative paths don't always work reliably, so you can directly set the gsrobotics path in the file gsmini_img_pub.py.

Simply edit the path variable in that file to point to your local gsrobotics folder:
```bash
# Example - adjust to your workspace structure
GSROBOTICS_PATH = "/home/yourusername/projects/tactile_ws/src/gelsight_mini_ros2/gsrobotics"
```
