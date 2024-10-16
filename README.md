# Project 1 - Indoor Environment Simulation

This ROS2 package simulates an indoor environment based on the first floor of The University of Alabama Alpha Chi Omega Sorority House 

## Package Structure
- **worlds/**: Contains the Webots `.wbt` world file used in the simulation.
- **launch/**: Contains the `simulation_launch.py` file to launch the simulation.
- **CMakeLists.txt** and **package.xml**: Standard ROS2 package files for building and installing the package.

## Prerequisites

Ensure you have ROS2 installed and sourced (tested with ROS2 Humble). You will also need Webots and `webots_ros2` installed.

1. Install Webots if you haven't already:
    ```bash
    sudo apt update
    sudo apt install webots
    ```

2. Install the `webots_ros2` package:
    ```bash
    sudo apt install ros-humble-webots-ros2
    ```

## Installation

1. Clone the repository:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/ggkarabas/CS460_Project1Submission project1
    ```

2. Build the package:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

3. Source the workspace:
    ```bash
    source install/setup.bash
    ```

## Running the Simulation

To run the simulation, use the following command:

```bash
ros2 launch project1 simulation_launch.py
```
## Opening/Closing Doors

To open or close the doors, simply click on the Importable Externproto dropdown menu and change the position of the 'Door' Node from 0 to 1. 0 = closed, 1 = open. 

