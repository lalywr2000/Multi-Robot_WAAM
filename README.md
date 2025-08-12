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
- Data models and algorithms for paths, contours, events, and scheduling
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

To run individual examples, you need to configure the variables at the top of the `multi-robot.py` file as follows. 

| Example            | [nozzle](/assets/nozzle_sim.gif) | [puzzle](/assets/puzzle_sim.gif) | [square](/assets/square_sim.gif) |
|:-------------------|:--------------------------------:|:--------------------------------:|:--------------------------------:|
| GCODE_MODE         | False                            | False                            | True                             |
| PREPROCESSING_MODE | False                            | True                             | True                             |
| SCHEDULING_MODE    | True                             | True                             | True                             |
| ALGORITHM_MODE     | 1                                | 1                                | 1                                |
| GCODE_PATH         | N/A                              | N/A                              | path/gcode/square.gcode          |
| MANUAL_PATH        | path/manual/nozzle.txt           | path/manual/puzzle.txt           | N/A                              |
| ROBOT_REACHABLE_R  | 2500.0                           | 2500.0                           | 2500.0                           |
| SUBSTRATE_SIZE     | 500.0                            | 500.0                            | 500.0                            |
| MAX_CONTOUR_LENGTH | N/A                              | 300.0                            | 300.0                            |
| DWELLING_TIME      | 0.0                              | 0.0                              | 0.0                              |

After modifying and saving the changes, run the example using the following command:

```bash
python3 multi-robot.py
```

## ABB RobotStudio validation with RAPID

[RobotStudio](https://new.abb.com/products/robotics/software-and-digital/robotstudio) is ABB's commercial software, implemented in the RAPID code format, and supports simulation and Offline Programming (OLP). By utilizing this virtual test environment, it is possible to validate whether path following is feasible without collisions, considering the robot layout and the movement of joints. 

<div width="100%" align="center">
    <img width="51%" src="/assets/robotstudio_sim.gif">
    <img width="48%" src="/assets/robotstudio_img.png">
</div>

-> layout position rotation modelname substrate size position color weldgunnum wobjnum

rapid code formulation:



```shell
MODULE Module1
    CONST robtarget Target_1:=[[0,0,100],[0,0,1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_2:=[[500,0,100],[0,0,1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_3:=[[500,500,100],[0,0,1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    ...     <--- (copy & paste ROB1_target here)

    VAR syncident sync1;
    VAR syncident sync2;
    PERS tasks all_tasks{2}:=[["T_ROB1"],["T_ROB2"]];
    
    PROC mhome()
        MoveJ Target_1\ID:=1,v500,fine,Weldgun1\WObj:=Workobject_1;
    ENDPROC
    
    PROC main()
        ConfJ\Off;
        ConfL\Off;
        AccSet 100,100;
    
        WaitSyncTask sync1,all_tasks;
        SyncMoveOn sync2,all_tasks;
        mhome;
        SyncMoveOff sync2;
    
        MoveL Target_1,v1000,fine,Weldgun1\WObj:=Workobject_1;
        MoveL Target_2,v1000,fine,Weldgun1\WObj:=Workobject_1;
        MoveL Target_3,v1000,fine,Weldgun1\WObj:=Workobject_1;
        ...     <--- (copy & paste ROB1_move here)
    ENDPROC
ENDMODULE
```

## future work

- Offline to online planning
- Minimize build time (formulation for optimization problem)
- Considering heat leveling
- Digital twin
- Further optimization of scheduling algorithms for even better efficiency.
