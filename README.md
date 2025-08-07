# Multi-Robot WAAM Path Planning

<div width="100%" align="center">
    <img width="32%" src="/nozzle_sim.gif">
    <img width="32%" src="/puzzle_sim.gif">
    <img width="32%" src="/square_sim.gif">
</div>

<div width="100%" align="center">
    <img width="32%" src="/nozzle_path.png">
    <img width="32%" src="/puzzle_path.png">
    <img width="32%" src="/square_path.png">
</div>


## Multi-Robot WAAM Path Planning
Project Overview
This project focuses on the development of a Multi-Robot Path Planning system for Wire Arc Additive Manufacturing (WAAM). The goal is to enhance the efficiency and scalability of WAAM by integrating multiple robotic arms for collaborative 3D printing tasks. This approach aims to increase the printable volume and parallel deposition capability, while ensuring the effective scheduling, collision avoidance, and synchronization of robots.

## Key Features
- Multi-Robot Coordination: The system supports multiple robotic arms working in parallel to deposit material, effectively increasing the build volume and speed.

- Path Planning & Scheduling: Efficient scheduling algorithms are applied at the bead level for each robot. The system ensures proper robot placement, path collision avoidance, and accessibility range considerations.

- Collision Detection: Collision detection is implemented by using bounding boxes to monitor interactions between robotic arms during the deposition process.

- Real-Time Visualization: A visualization tool is integrated to provide real-time feedback on the deposition process, helping to optimize the workflow and ensure quality control.

- Automatic Code Conversion: The system features automatic conversion of RAPID code for verification in ABB RobotStudio, allowing seamless integration with existing robot control systems.

- Flexible Deposition Scenarios: The system can handle different deposition setups, including dual and triple-robot deposition configurations.

## Benefits of Multi-Robot Systems in WAAM
- Parallel Deposition Capability: Multiple robots working simultaneously can deposit material faster, significantly reducing production time.

- Larger Printable Volume: The use of multiple robots increases the total build volume, allowing for the creation of larger parts.

- Efficient Scheduling: By carefully coordinating the robots' actions, the system minimizes idle times and optimizes the overall manufacturing process.

- Enhanced Flexibility: The system is adaptable to a variety of deposition scenarios, making it suitable for different manufacturing requirements.

## Research Background
This work is inspired by the research conducted by Oak Ridge National Laboratory (ORNL, USA), which demonstrated the potential of multi-robot-based WAAM systems. Their approach utilized a custom-developed path planning algorithm, and this project builds upon their successful implementation, adding new capabilities such as centralized task scheduling and advanced collision detection.

## Development Progress
- Scheduling & Path Planning: Algorithms are developed to schedule robot actions and plan paths based on the build structure, with careful consideration of cooling times (dwell) and robot accessibility.

- Verification in RobotStudio: The system has been verified using ABB RobotStudio, where robot operations and collision detection were tested in virtual environments to ensure safe and accurate execution.

- Real-World Testing
The system has been tested in real-world scenarios, demonstrating its effectiveness in multi-robot synchronization and its ability to adapt to various deposition tasks, such as the creation of complex parts like rocket nozzles.

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
-
-

---
below contents are not related to this project, just for md format references


This repository provides tools and examples for controlling and experimenting with **PiRacer model vehicle** in Gazebo. The Gazebo model, designed to match the hardware specifications of PiRacer, can be controlled via ROS2 topic communication. The included teleoperation example is a simple demonstration using the WASD keys on the keyboard, allowing simultaneous control of both the real PiRacer and the PiRacer in Gazebo.

The idea is to utilize this repository as a template for **digital twin** research. By equipping PiRacer with sensors such as odometer, IMU, LiDAR, etc., and implementing closed-loop feedback control, it would be possible to more accurately replicate the behavior of the real-world PiRacer in Gazebo.

| ![Throttle](/images/throttle.gif) | ![Steering](/images/steering.gif) |
|:---:|:---:|
| Throttle | Steering |

## ROS2 Packages

```shell
./
 ├── simulation_ws/src/
 │   └── sim        # Description and launch file for PiRacer model
 │
 └── teleoperation_ws/src/
     └── teleop     # Remote control example including a controller and a receiver
```

## Requirements

Before using this package, ensure the following prerequisites are installed:

| Device           | Software/Framework                                                                 | Functionality              |
|------------------|------------------------------------------------------------------------------------|----------------------------|
| **Local**        | [Gazebo 11](https://fdeantoni.medium.com/ros2-dev-with-gazebo-11-3f1795bba33)      | Simulation environment     |
| **Local**        | [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) | Robotics middleware        |
| **Local**        | [pygame](https://pypi.org/project/pygame/)                                         | Keyboard input for control |
| **Raspberry Pi** | [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) | Robotics middleware        |
| **Raspberry Pi** | [piracer](https://github.com/twyleg/piracer_py)                                    | Physical PiRacer control   |

This package has been developed and tested on both local machine and Raspberry Pi 4, using Ubuntu 20.04.

## Usage

**Gazebo Simulator and Teleoperation Controller are for local environment. While this controller can simultaneously control physical PiRacer, if the purpose is solely simulation, the setup on Raspberry Pi can be disregarded, and only the local environment is utilized.**

### 0. Package Configuration

Change the `gazebo_model_path` in the **package.xml** located in `simulation_ws/src/sim` to suit your local environment.

### 1. Launch Gazebo Simulator

```bash
# Local
cd simulation_ws
colcon build

source install/local_setup.bash
ros2 launch sim sim.launch.py
```

You should see the PiRacer model inside the Gazebo simulation window.

### 2. Run Teleoperation Receiver

```bash
# Raspberry Pi
cd teleoperation_ws
colcon build

source install/local_setup.bash
ros2 run teleop receiver
```

### 3. Run Teleoperation Controller

```bash
# Local
cd teleoperation_ws

source install/local_setup.bash
ros2 run teleop controller
```

When you run the controller, a small pygame window like the following will appear.

<div width="100%" align="center"><img src="/images/controller.png" align="center" width="30%"></div>

Click on this window and press the WASD keys on the keyboard. Observe the movement of both Gazebo PiRacer and physical PiRacer.

## Note

The communication between controller node and receiver node is machine-to-machine. Make sure that local machine and Raspberry Pi are connected to the same WLAN. If connection is not successful, disable the firewall using the following command.

```bash
# Local & Raspberry Pi
sudo ufw disable
```

## Explore

Design various experiments and developments using the Gazebo PiRacer. This package provides an example with the PiRacer camera. Check the view of the PiRacer's camera using **RViz2**. For instance, you can use this image data to implement autonomous driving in the simulation.

<div width="100%" align="center"><img src="/images/camera.png" align="center" width="70%"></div>

## References

- [Waveshare PiRacer](https://www.waveshare.com/wiki/PiRacer_AI_Kit)
- [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html)
- [Gazebo](https://classic.gazebosim.org/)
