# Multi-Robot WAAM Path Planning

<div width="100%" align="center">
    <img width="32%" src="/assets/nozzle_sim.gif">
    <img width="32%" src="/assets/puzzle_sim.gif">
    <img width="32%" src="/assets/square_sim.gif">
</div>

<div width="100%" align="center">
    <img width="32%" src="/assets/nozzle_path.png">
    <img width="32%" src="/assets/puzzle_path.png">
    <img width="32%" src="/assets/square_path.png">
</div>

## Project Overview

This project focuses on path planning for multi-robot **Wire Arc Additive Manufacturing (WAAM)**. The system enables two or more industrial robots to collaboratively manufacture a single part by generating coordinated deposition paths.

By applying multi-robot systems to WAAM, manufacturing time can be significantly reduced through parallelized deposition, and large-scale part fabrication becomes feasible.

The key features of this implementation are as follows:

- Integration of an efficient scheduling algorithm
- Validation of robot accessibility (workspace reachability)
- Implementation of dwelling time between layers
- Visualization for planning validation
- Synchronization mechanisms to coordinate robotic operations
- Automatic RAPID code generation for simulation in ABB RobotStudio

## Background & Reference

Multi-robot WAAM was first successfully demonstrated by [Oak Ridge National Laboratory (ORNL)](https://www.sciencedirect.com/science/article/pii/S2772369023000634?ref=pdf_download&fr=RR-2&rr=96b4d22519d6e375), where they used three 6-axis ABB robots to fabricate large and complex parts. This project is built upon insights from ORNL’s open-source work on multi-robot additive manufacturing.

In particular, this project utilizes and extends functionality from [Pyrobopath](https://github.com/alexarbogast/pyrobopath), a Python package designed for demonstrating multi-robot path planning in additive manufacturing. Pyrobopath includes:

- Centralized bead-level scheduling system
- Well-structured data models and algorithms for paths, contours, events, and scheduling
- Collision detection using [bounding box method](https://pyrobopath.readthedocs.io/en/latest/users/examples/python_examples.html) and the [python-fcl](https://github.com/BerkeleyAutomation/python-fcl) library

For more details, refer to the Pyrobopath repository and documentation.

## Directory Structure

```shell
./
 ├── multi-robot.py   # Main code for path planning
 │
 ├── path/
 │   └── gcode/       # Location for .gcode path files
 │   └── manual/      # Location for .txt path files
 │        └── path_gen_script/   # Script for generating manual .txt paths
 │
 └── rapid_gen/       # Location for RAPID code output
```

## Requirements

Before running the program, ensure that the following prerequisites are installed:

```bash
pip install numpy
pip install matplotlib
pip install gcodeparser
pip install pyrobopath
```

This program has been developed and tested using Ubuntu 20.04 and Pyrobopath 0.2.8

## Usage

The following shows how to run the examples included in the repository. First, clone the repository using the following command:

```bash
git clone https://github.com/lalywr2000/Multi-Robot_WAAM.git
```

### - Example 1: nozzle aaa


**Gazebo Simulator and Teleoperation Controller are for local environment. While this controller can simultaneously control physical PiRacer, if the purpose is solely simulation, the setup on Raspberry Pi can be disregarded, and only the local environment is utilized.**


```bash
# Local
cd simulation_ws
colcon build

source install/local_setup.bash
ros2 launch sim sim.launch.py
```













## How to Use
- Installation: The software is compatible with ABB RobotStudio for verification and simulation. Make sure to set up your robot system and integrate the provided code for path planning and scheduling.

- Visualization: Use the built-in visualization tool to monitor the deposition process in real-time and adjust parameters as needed.

## Future Work
Further optimization of scheduling algorithms for even better efficiency.

Expansion to handle more complex multi-robot setups.

Continuous integration with new hardware and robotic systems.




-
-
-
-
-
GCODE_MODE = False
# True: input gcode path   # False: input manual path
PREPROCESSING_MODE = False
# True: preprocessing      # False: skip preprocessing
SCHEDULING_MODE = True
# True: scheduling         # False: visualizing

ALGORITHM_MODE = 1
# 0: sequential
# 1: distance priority

GCODE_PATH  = "./path/gcode/square.gcode"
MANUAL_PATH = "./path/manual/nozzle.txt"

ROBOT_REACHABLE_R  = 2500.0  # [mm]
SUBSTRATE_SIZE     = 500.0   # [mm]
MAX_CONTOUR_LENGTH = 300.0   # [mm]
DWELLING_TIME      = 0.0



GCODE_MODE = False
# True: input gcode path   # False: input manual path
PREPROCESSING_MODE = True
# True: preprocessing      # False: skip preprocessing
SCHEDULING_MODE = True
# True: scheduling         # False: visualizing

ALGORITHM_MODE = 1
# 0: sequential
# 1: distance priority

GCODE_PATH  = "./path/gcode/square.gcode"
MANUAL_PATH = "./path/manual/puzzle.txt"

ROBOT_REACHABLE_R  = 2500.0  # [mm]
SUBSTRATE_SIZE     = 500.0   # [mm]
MAX_CONTOUR_LENGTH = 300.0   # [mm]
DWELLING_TIME      = 0.0




GCODE_MODE = True
# True: input gcode path   # False: input manual path
PREPROCESSING_MODE = True
# True: preprocessing      # False: skip preprocessing
SCHEDULING_MODE = True
# True: scheduling         # False: visualizing

ALGORITHM_MODE = 1
# 0: sequential
# 1: distance priority

GCODE_PATH  = "./path/gcode/square.gcode"
MANUAL_PATH = "./path/manual/nozzle.txt"

ROBOT_REACHABLE_R  = 2500.0  # [mm]
SUBSTRATE_SIZE     = 500.0   # [mm]
MAX_CONTOUR_LENGTH = 300.0   # [mm]
DWELLING_TIME      = 0.0



-
-
-
-
-
-
# ABB RobotStudio validation with RAPID
<div width="100%" align="center"><img src="/robotstudio_sim.gif" align="center" width="100%"></div>
<div width="100%" align="center"><img src="/robotstudio_img.png" align="center" width="100%"></div>
-
-


## References
Arbogast, Alex, et al. "Strategies for a scalable multi-robot large scale wire arc additive manufacturing system.“
Additive Manufacturing Letters 8 (2024): 100183.


future work
-> offline to online planning
-> minimize build time (formulation for optimization problem)
-> leveling distribution of heat on the part
-> digital twin
-> better allocation algorithm
