# !/bin/bash
rosrun rvo_ros set_goals_client default -5 0 -3.54 -3.54 3.54 -3.54 5 0 3.54 3.54 -3.54 3.54

roslaunch local_planner local_planner.launch

# rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: -3.54, y: 3.54, z: 0.0}, orientation: {w: 1.0}}}'

# rosrun local_planner path_recorder.py

# sleep 10

# rosrun rvo_ros set_goals_client default 8 1 9 1 8 2 9 2 8 3 9 3 1 1 1 2 2 1 2 2 1 3

# sleep 10

# rosrun rvo_ros set_goals_client circle 4 4 4 0
# rosrun rvo_ros set_goals_client default -5.24 12.96 -6.82 12.96 8.05 -12.70 6.49 -12.76

# sleep 10

# rosrun rvo_ros set_goals_client random 6.91 8.76 -11.96 12.82
# rosrun rvo_ros set_goals_client random -7.18 8.66 -13.36 11.32
# rosrun rvo_ros set_goals_client random -8 8 -5 5
# rosrun rvo_ros set_goals_client default 4 3 5 5 6 3 4 6 6 4 6 5 4 5 4 4 4 7 6 6 6 7   # H