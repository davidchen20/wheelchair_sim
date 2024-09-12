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

Open the wheelchair sim and launch the gazebo simulation environment

`roslaunch wheelchair_gazebo wheelchair_lab.launch `

Open wheelchair navigation

`roslaunch wheelchair_navigation wheelchair_navigation.launch`
