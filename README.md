# Project 1 - Indoor and Outdoor Environment Simulations

This GitHub repository contains ROS2 packages for simulating both an indoor and an outdoor environment. The indoor environment simulates the first floor of The University of Alabama Alpha Chi Omega Sorority House, while the outdoor environment represents a customizable natural outdoor scene.

## Package Structure

- **project1/**: Contains files for the indoor environment simulation.
    - **worlds/**: Indoor Webots `.wbt` world file.
    - **launch/**: `simulation_launch.py` to launch the indoor simulation.
- **project2/**: Contains files for the outdoor environment simulation.
    - **worlds/**: Outdoor Webots `.wbt` world file.
    - **launch/**: `outdoor_simulation_launch.py` to launch the outdoor simulation.
- **CMakeLists.txt** and **package.xml**: Standard ROS2 package files for building and installing each package.

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
    git clone https://github.com/ggkarabas/CS460_Project1Submission.git
    ```

2. Build the packages:

    ```bash
    cd ~/ros2_ws
    colcon build
    ```

3. Source the workspace:

    ```bash
    source install/setup.bash
    ```

## Running the Simulations

### Indoor Simulation

To run the indoor simulation, use the following command:

```bash

ros2 launch project1 simulation_launch.py
```

# Opening/Closing Doors

To open or close the doors in the indoor simulation:

1. Open the `.wbt` world file in Webots.
2. In the Importable Externproto dropdown menu, locate the 'Door' node.
3. Change the position of the 'Door' Node from 0 to 1:
    - 0 = closed
    - 1 = open

# Outdoor Simulation

To run the outdoor simulation, use the following command:

```bash
ros2 launch project2 outdoor_simulation_launch.py
```

# Adjusting Lighting for Nighttime

To simulate nighttime in the outdoor environment:

1. Open the `outdoorworld_gabriellekarabas.wbt` file in Webots.
2. Locate the `DirectionalLight` node.
3. Reduce the luminosity value to create a dimmer, nighttime effect.

# Additional Information

## Modifying World Files

Both the indoor and outdoor `.wbt` files can be customized further:

- **Indoor Simulation**: Modify room layouts, furniture, and other elements to suit your simulation needs.
- **Outdoor Simulation**: Adjust terrain, add objects, or modify vegetation for different scenarios.

## Troubleshooting

If you encounter issues:

- **Ensure dependencies are installed**: Confirm that Webots and `webots_ros2` are installed.
- **Source the workspace**: Run `source install/setup.bash` to ensure your terminal recognizes the new packages.
- **Verify file paths**: Confirm that world files and launch files are correctly placed in their respective directories.

# Repository Structure

```
multi_simulations/
├── project1/          # Indoor simulation package
│   ├── worlds/        # World files for indoor simulation
│   ├── launch/        # Launch files for indoor simulation
│   ├── CMakeLists.txt
│   └── package.xml
├── project2/          # Outdoor simulation package
│   ├── worlds/        # World files for outdoor simulation
│   ├── launch/        # Launch files for outdoor simulation
│   ├── CMakeLists.txt
│   └── package.xml
└── README.md          # Project documentation
```
