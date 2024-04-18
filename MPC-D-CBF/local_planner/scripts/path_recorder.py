#!/usr/bin/env python3

import numpy as np
import rospy
import tf
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import ModelStates
import csv


class Path_Recorder():
    def __init__(self):
        self.N = 10
        self.rate = rospy.Rate(50)

        self.model_state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_state_cb)
        self.last_pose = [0, 0]
        self.curr_pose = [0, 0]
        self.curr_ang = 3*np.pi / 4
        self.last_ang = 3*np.pi / 4
        self.distance = 0
        self.ang_diff = 0
        self.dis_thrd = 0.05
        self.init = False
        self.start_time = rospy.Time.now()
        # self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
    def get_min_distance(self, states):

        robot_index = states.name.index("wheelchair")

        min_distance = np.inf
        for i in range(len(states.name)):
            if(i != robot_index):
                distance2obs = np.sqrt( (states.pose[robot_index].position.x-states.pose[i].position.x) ** 2 + (states.pose[robot_index].position.y - states.pose[i].position.y) ** 2 )
                if(distance2obs < min_distance):
                    min_distance = distance2obs

        return min_distance
    
    
    def model_state_cb(self, states):
        cur_time = rospy.Time.now()
        time_stamp = cur_time.to_sec() - self.start_time.to_sec()

        robot_index = states.name.index("wheelchair")

        min_distance = self.get_min_distance(states)
        # [x, y, ang, time_diff, linear_velocity, ang_velocity, min_distance]
        if(not self.init):
            self.last_pose = [states.pose[robot_index].position.x, states.pose[robot_index].position.y, 3*np.pi / 4, time_stamp,0,0, min_distance]
            self.curr_pose = [states.pose[robot_index].position.x, states.pose[robot_index].position.y, 3*np.pi / 4, time_stamp,0,0, min_distance]

            self.init = True
            with open('/home/yfx/jackal_ws/src/MPC-D-CBF/local_planner/robot_pose_remake/robot_pose_tmp.csv', 'a', newline='') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow(self.curr_pose)
            return
        
        self.curr_pose = [states.pose[robot_index].position.x, states.pose[robot_index].position.y, 3*np.pi / 4, time_stamp,0,0, min_distance]

        x_diff = self.curr_pose[0]-self.last_pose[0]
        y_diff = self.curr_pose[1]-self.last_pose[1]
        self.distance = np.sqrt(x_diff ** 2 + y_diff ** 2)
        self.ang_diff = np.arctan2(y_diff, x_diff) - self.last_pose[2]

        self.curr_pose[2] = np.arctan2(y_diff, x_diff)
        self.curr_pose[4] = self.distance / (self.curr_pose[3] - self.last_pose[3])
        self.curr_pose[5] = self.ang_diff / (self.curr_pose[3] - self.last_pose[3])

        if(self.distance > self.dis_thrd):
            with open('/home/yfx/jackal_ws/src/MPC-D-CBF/local_planner/robot_pose_remake/robot_pose_tmp.csv', 'a', newline='') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow(self.curr_pose)
            self.last_pose = self.curr_pose

        return


if __name__ == '__main__':
    rospy.init_node('path_recorder')
    path_recorder = Path_Recorder()
    rospy.spin()


