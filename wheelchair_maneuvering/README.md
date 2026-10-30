# First maneuvering experiment

This action client does the same job as RViz's **2D Nav Goal**: it sends a
position and heading to the running move_base server. It does not implement
exclusive control, a custom planner, or independent stopped-motion verification.
Use it in simulation after localization and ordinary navigation work.

## Run

Keep the simulator and TEB navigation running in their existing terminals.
In another terminal:

```bash
source ~/catkin_ws/devel/setup.bash
rospack find wheelchair_maneuvering
rosrun wheelchair_maneuvering send_maneuver_goal.py --help
```

The package can be discovered in the existing source workspace without a rebuild.
The CMake file also installs the script when the workspace is next built.

Choose a clear target in the map. To capture its coordinates without moving the
chair, use RViz's **Publish Point** tool (not **2D Nav Goal**) while this runs:

```bash
rostopic echo -n 1 /clicked_point
```

Its `point.x` and `point.y` are map coordinates when RViz's Fixed Frame is `map`.
Heading is absolute, counterclockwise: 0 degrees = map +x, 90 = map +y,
180 = map -x, -90 = map -y. It is not a relative turn command.

Preview a pose without connecting to ROS or commanding motion:

```bash
rosrun wheelchair_maneuvering send_maneuver_goal.py --x 2.767 --y 6.261 --yaw-deg 0 --dry-run
```

To execute it, remove `--dry-run` only after checking the target is appropriate
for the chair's current location and surroundings. These example coordinates
are from an earlier lab trial; they are not a universal docking location.

Ctrl+C requests cancellation of this script's own goal. The fallback cancel
command, which cancels all goals on move_base, is:

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}'
```

## Read the script

1. `arguments()` reads x, y, and heading from the command line.
2. `MoveBaseGoal` holds the map-frame pose; sine/cosine encode heading as a quaternion.
3. `SimpleActionClient` connects to `/move_base`.
4. `send_goal()` starts navigation and registers a completion callback.
5. `done()` receives the terminal action status. Status 3 means navigation success.
6. The shutdown hook and `finally` block request cancellation if interrupted.

Next steps are command ownership, a maneuver task interface, and checking actual
stopped motion before handing control back. This client alone does not disable
shared control or prevent another client from replacing its goal.
