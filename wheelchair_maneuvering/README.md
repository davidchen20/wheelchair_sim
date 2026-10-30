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

## Exclusive control experiment

Stop the old navigation launch first. Keep Gazebo running. In its place run:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch wheelchair_maneuvering exclusive_maneuvering.launch
```

Correct localization if the chair is no longer at its original spawn. This launch
runs TEB navigation and the command selector. It starts in `stopped` mode.
Check `rostopic info /cmd_vel`: its only publisher must be `/command_selector`.
`move_base` must publish on `/maneuver/cmd_vel`, never directly on `/cmd_vel`.

Then send a suitable goal with exclusive control:

```bash
rosrun wheelchair_maneuvering send_maneuver_goal.py --x 2.767 --y 6.261 --yaw-deg 0 --exclusive
```

The script acquires a unique control token and publishes its heartbeat. A second
client cannot acquire control while a maneuver owns it. The selector blocks shared
commands during maneuver mode, publishes zero for 0.25 seconds during handover,
and outputs zero if selected commands are older than 2 wall-clock seconds. If the
client heartbeat disappears for 2 seconds, it switches to stopped even if
move_base continues sending commands. Unique tokens prevent an older client
from releasing or renewing a newer client's lease. Ctrl+C cancels this client's
goal and releases control. Completion also releases to `stopped`, not shared.

Monitor mode with:

```bash
rostopic echo /control_mode
```

To immediately select stopped mode manually:

```bash
rosservice call /control/enable_maneuver "data: false"
```

The client then cancels its goal after observing loss of ownership. To explicitly
enable shared mode after a maneuver:

```bash
rosservice call /control/enable_shared
```

This does not start a shared controller. When integrating the real shared-control
node, remap its output to `/shared/cmd_vel` at launch (for example
`/cmd_vel:=/shared/cmd_vel`). Every controller must go through the selector.
The exclusive launch uses TEB, which does not blend `/user_cmd` into commands.
Do not replace it with the existing shared MPC plugin without separately blocking
its user-input blending for autonomous operation.

All nodes with access to the ROS graph can still publish directly to `/cmd_vel`:
this is application-level command selection, not access control. Selector process
failure also requires a downstream motor command watchdog for deployment; this
simulation experiment does not provide one. Avoid issuing RViz goals during an
exclusive run, because they use the same move_base action server.

Offline gate checks (no master or motion required):

```bash
python3 wheelchair_maneuvering/tests/test_command_gate.py
```
