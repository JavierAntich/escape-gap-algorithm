# escape-gap-algorithm
Nowadays, there are many tools for facilitating robot programming. Among these tools, two of the most well-known are MRPT (Mobile Robot Programming Toolkit, https://www.mrpt.org/) and ROS (Robot Operating System, http://www.ros.org/). MRPT provides a set of well-tested applications and libraries covering data structures and algorithms employed in common robotics research areas.

This GitHub repository contains the C++ source code that implements a novel algorithm called Escape Gap (EG). This algorithm allows a robot both to safely navigate through narrow spaces and to escape from large obstacles, even when these obstacles have a shape that makes very difficult to find a way out. Moreover, all this is done in accordance with the spirit of the reactive/sense-act paradigm, which means that all decisions about the robot’s actions are merely made using local information of the environment.

EG has been integrated into the MRPT application named Holonomic Navigator Demo (https://www.mrpt.org/list-of-mrpt-apps/application-holonomic-navigator-demo/). This application provides a friendly Graphical User Interface to test the EG algorithm in simulation.

![The EG algorithm in action](./imgs/screenshot.png?raw=true "The EG algorithm successfully moves the robot from the green square to the red star")
