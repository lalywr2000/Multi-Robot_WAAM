# PiRacer Simulation with ROS2 and Gazebo

<div width="100%" align="center">
    <img width="49%" src="/images/design.png">
    <img width="49%" src="/images/model.png">
</div>

---

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
