# Starting the simulation and finding your first coding task

This repository targets ROS Noetic. The existing workspace must be built before
launching. On this machine `/home/surik-wheelchair/catkin_ws/devel/setup.bash`
already exists; no rebuild is needed for launch-file or YAML changes.

## Run the lab baseline

In terminal 1:


```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch wheelchair_gazebo wheelchair_lab.launch model:=waffle
```

In terminal 2:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch wheelchair_navigation wheelchair_navigation.launch model:=waffle
```

Gazebo shows the physical simulation. RViz shows the robot, map, and ROS data.
The navigation launches use the user's recorded map-frame AMCL estimate for the
original lab spawn: x=5.150847450733085, y=6.446909230418239,
yaw=3.139226658721223 radians. This is an initialization estimate, not a calibrated
Gazebo-world-to-map transform; verify laser alignment after restarting.
For other starting locations, pass `initial_pose_x:=... initial_pose_y:=...
initial_pose_a:=...` (heading in radians). Use RViz's **2D Pose Estimate** to
correct localization if needed. Set RViz's Fixed Frame to `map`.

Before trying a goal, confirm that the robot and laser returns align with the
map. RViz's **2D Nav Goal** sends a position and heading to navigation. Ordinary
navigation is not yet a docking implementation.

Stop navigation with Ctrl-C in terminal 2, then stop Gazebo with Ctrl-C in
terminal 1. Avoid starting duplicate navigation or simulation launches.

## Inspect the running system

In a third terminal, source the workspace as above, then run:

```bash
rosnode list
rostopic list
rostopic info /cmd_vel
rostopic echo -n 1 /odom
rostopic hz /scan
rosparam get /move_base/base_local_planner
rosrun rqt_graph rqt_graph
```

`rostopic hz` runs until Ctrl-C. `rqt_graph` is optional and requires the
corresponding ROS GUI package.

## Trace one command through the source

1. `wheelchair/wheelchair_gazebo/launch/wheelchair_lab.launch` starts Gazebo,
   expands the robot Xacro, and spawns the wheelchair.
2. `wheelchair/wheelchair_description/urdf/wheelchair.gazebo.xacro` configures
   the differential-drive plugin: `cmd_vel` in, `odom` out.
3. `wheelchair/wheelchair_navigation/launch/wheelchair_navigation.launch`
   starts lidar conversion, the map server, AMCL, and includes `move_base.launch`.
4. `wheelchair/wheelchair_navigation/launch/move_base.launch` selects
   `mpc_ros/MPCPlannerROS` and loads costmap/controller configuration.
5. `move_base_planner/mpc-ros/mpc_ros/src/mpc_plannner_ros.cpp` implements
   `setPlan()` and `computeVelocityCommands()` inside the move_base process.

## First maneuvering coding milestone

Create a separate `wheelchair_maneuvering` package rather than editing the MPC
solver first. Begin with a command selector:

```text
existing shared controller -> /shared/cmd_vel
future maneuver controller -> /maneuver/cmd_vel
command selector           -> /cmd_vel
```

Its first acceptance criteria are: only the selected source is forwarded;
switching modes outputs zero; stale commands output zero; old commands are not
replayed after switching back. Check timestamps using a monotonic clock for
command freshness so a paused simulation cannot keep a command valid forever.
Remap every existing velocity publisher before enabling the selector.

Next add a maneuver action server that accepts a target pose and wraps the
`move_base` action. Add cancellation, result reporting, and stopped-motion
verification. Establish a TEB baseline in a separate launch configuration before
implementing a custom search planner. A planner generates the route; a controller
tracks it; the maneuver server owns the task and mode transitions.

## Bus scenario differences

`wheelchair_bus.launch` currently loads `wheelchair.urdf.xacro`, whereas the lab
loads `wheelchair_1.urdf.xacro` with Velodyne. Verify the sensor topics before
reusing the lab navigation launch. The bus requires its own matching map and
initial localization pose; the lab defaults do not apply.
