#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import csv

def plot_traj(file_path, name):
    dv = np.loadtxt(file_path, delimiter=',')
    dv = dv[:, :2]
    rot_ang = -np.pi / 4
    dv_after = np.matmul(np.array([[np.cos(rot_ang), -np.sin(rot_ang)],
                                   [np.sin(rot_ang), np.cos(rot_ang)]]), dv.transpose())
    x = dv_after[1, :]
    y = dv_after[0, :]
    if(name=="Ours"):
        plt.plot(x,y, linestyle="-", linewidth=2.0, color="r", label="Ours")
    else:
        plt.plot(x,y, linestyle="--", label = name)

def eval_data(file_path, name):
    dv = np.loadtxt(file_path, delimiter=',')

    # Traj length
    dv_pose = dv[:, :2]
    traj_length = 0
    for i in range(1, dv_pose.shape[0]):
        traj_length += np.sqrt((dv_pose[i, 0]-dv_pose[i-1, 0])**2 + (dv_pose[i, 1]-dv_pose[i-1, 1])**2)
    print("{0}: Trajectory length is {1}\n".format(name, traj_length))

    # Time 
    dv_time = dv[:, 3]
    con_time = dv_time[-1]-dv_time[0]
    print("{0}: Time is {1}\n".format(name, con_time))

    # Min distance
    dv_min_dis = dv[:, -1]
    min_dis = np.min(dv_min_dis)
    print("{0}: Min Distance is {1}\n".format(name, min_dis))

    # Linear Speed Var
    dv_linear_vel = dv[1:, 4]
    linear_vel_var = np.var(dv_linear_vel)
    print("{0}: Linear vel var is {1}\n".format(name, linear_vel_var))

    # Ang Speed var
    dv_ang_vel = dv[1:, 5]
    ang_vel_var = np.var(dv_ang_vel)
    print("{0}: Ang vel var is {1}\n".format(name, ang_vel_var))

if __name__ == '__main__':

    dv = np.loadtxt('../robot_pose_remake/reference.csv', delimiter=',')
    rot_ang = -np.pi / 4
    dv_after = np.matmul(np.array([[np.cos(rot_ang), -np.sin(rot_ang)],
                                   [np.sin(rot_ang), np.cos(rot_ang)]]), dv[:2,:].transpose())
    x = dv_after[1, :]
    y = dv_after[0, :]
    plt.plot(x,y, linestyle="--", label = "Reference")

    experiment_name = "aggressive"

    eval_data("../robot_pose_remake/{0}_experiment/{1}_mpc.csv".format(experiment_name, experiment_name), "MPC")
    eval_data("../robot_pose_remake/{0}_experiment/{1}_wo_share.csv".format(experiment_name, experiment_name), "MPC-DCBF")
    eval_data("../robot_pose_remake/{0}_experiment/{1}_socialmpc.csv".format(experiment_name, experiment_name), "Social-MPC")
    eval_data("../robot_pose_remake/{0}_experiment/{1}_w_share.csv".format(experiment_name, experiment_name), "Ours")


    plot_traj("../robot_pose_remake/{0}_experiment/{1}_mpc.csv".format(experiment_name, experiment_name), "MPC")
    plot_traj("../robot_pose_remake/{0}_experiment/{1}_wo_share.csv".format(experiment_name, experiment_name), "MPC-DCBF")
    plot_traj("../robot_pose_remake/{0}_experiment/{1}_socialmpc.csv".format(experiment_name, experiment_name), "Social-MPC")
    plot_traj("../robot_pose_remake/{0}_experiment/{1}_w_share.csv".format(experiment_name, experiment_name), "Ours")

    plt.legend() 

    plt.xlim(-5, 5)
    plt.ylim(-4, 4)
    plt.ylabel('y')
    plt.xlabel('x')
    plt.show()



