# Wheelchair Simulation

Wheelchair simulation with MPC-DCBF algorithm applied and tested. 

## Enviroment

Ubuntu 20.04

ROS Noetic

## Installation

`cd catkin_ws/src`

`git clone https://github.com/Shared-Wheelchair/wheelchair_sim.git`

`cd ../ && catkin_make`

## Usage

Open the wheelchair sim and rvo for pedestrain simulation.

`roslaunch wheelchair_gazebo wheelchair_dynamic.launch `

Dynamic crossing test.

`roscd wheelchair_gazebo/`

`bash dynamic_test_crossing.sh`
