#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32MultiArray, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.msg import Path
import threading
import numpy as np
from visualization_msgs.msg import Marker
import casadi as ca
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def distance_global(c1, c2):
    return np.sqrt((c1[0] - c2[0]) * (c1[0] - c2[0]) + (c1[1] - c2[1]) * (c1[1] - c2[1]))


class Local_Planner():
    def __init__(self):
        self.replan_period = rospy.get_param('/local_planner/replan_period', 0.05)

        self.z = 0
        # self.N = 25 #original
        self.N = 35 #zbedit4
        self.goal_state = np.zeros([self.N, 3])
        self.user_input = Twist()
        self.current_vel_time = 0
        self.last_vel_time = rospy.Time.now()
        self.time_step = rospy.Time.now()
        
        self.curr_state = None
        self.global_path = None

        self.last_input = []
        self.last_state = []
        self.mpc_success = None

        self.curr_pose_lock = threading.Lock()
        self.global_path_lock = threading.Lock()
        self.obstacle_lock = threading.Lock()

        self.tf_listener = tf.TransformListener()

        self.ob = []

        path_topic = rospy.get_param('/local_planner/global_path')

        self.__timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb)

        self.__sub_curr_state = rospy.Subscriber('/curr_state', Float32MultiArray, self.__curr_pose_cb, queue_size=10) #original

        self.__sub_obs = rospy.Subscriber('/obs_predict_pub', Float32MultiArray, self.__obs_cb, queue_size=10) #original

        # self.__sub_curr_state = rospy.Subscriber('/curr_state', Float32MultiArray, self.__curr_pose_cb, queue_size=14) #zbedit4

        # self.__sub_obs = rospy.Subscriber('/obs_predict_pub', Float32MultiArray, self.__obs_cb, queue_size=14) #zbedit4

        self.__sub_goal = rospy.Subscriber(path_topic, Path, self.__global_path_cb, queue_size=25) #original

        self.__sub_user = rospy.Subscriber('user_cmd', Twist, self.__user_cmd_cb, queue_size=25) #original

        # self.__sub_goal = rospy.Subscriber(path_topic, Path, self.__global_path_cb, queue_size=35) #zbedit4

        # self.__sub_user = rospy.Subscriber('user_cmd', Twist, self.__user_cmd_cb, queue_size=35) #zbedit4

        self.__pub_local_path_vis = rospy.Publisher('/pub_path_vis', Marker, queue_size=10) #original

        self.__pub_local_path = rospy.Publisher('/local_path', Path, queue_size=10) #original

        self.__pub_user_path = rospy.Publisher('/user_path', Path, queue_size=10) #original

        self.__pub_local_plan = rospy.Publisher('/local_plan', Float32MultiArray, queue_size=10) #original

        self.__pub_start = rospy.Publisher('/cmd_move', Bool, queue_size=10) #original

        # self.__pub_local_path_vis = rospy.Publisher('/pub_path_vis', Marker, queue_size=14) #zbedit4

        # self.__pub_local_path = rospy.Publisher('/local_path', Path, queue_size=14) #zbedit4

        # self.__pub_user_path = rospy.Publisher('/user_path', Path, queue_size=14) #zbedit4

        # self.__pub_local_plan = rospy.Publisher('/local_plan', Float32MultiArray, queue_size=14) #zbedit4

        # self.__pub_start = rospy.Publisher('/cmd_move', Bool, queue_size=14) #zbedit4

        

    def __replan_cb(self, event):
        if self.choose_goal_state():
            # add angle information
            for i in range(self.N - 1):
                y_diff = self.goal_state[i+1, 1]-self.goal_state[i, 1]
                x_diff = self.goal_state[i+1, 0]-self.goal_state[i, 0]
                if x_diff != 0 and y_diff != 0:
                    self.goal_state[i, 2] = np.arctan2(y_diff, x_diff)
                elif i != 0:
                    self.goal_state[i, 2] = self.goal_state[i-1, 2]
                else:
                    self.goal_state[i, 2] = 0

            self.goal_state[-1, 2] = self.goal_state[-2, 2]

            states_sol, input_sol = self.MPC_ellip()

            cmd_move = Bool()
            cmd_move.data = distance_global(self.curr_state, self.global_path[-1]) > 0.1
            self.__pub_start.publish(cmd_move)
            self.__publish_local_plan(input_sol, states_sol)

    def __curr_pose_cb(self, data):
        self.curr_pose_lock.acquire()
        self.curr_state = np.zeros(3)
        self.curr_state[0] = data.data[0]
        self.curr_state[1] = data.data[1]
        self.curr_state[2] = data.data[2]

        self.curr_pose_lock.release()

    def __obs_cb(self, data):
        # rospy.loginfo("zbcheck: in __obs_cb") #zbedit4 for print
        self.obstacle_lock.acquire()
        self.ob = []
        # size = int(len(data.data) / 5) //original
        # size = int(len(data.data) / 10) #zbedit4
        size = int(len(data.data) / 7) #zbedit4try
        # rospy.loginfo("len(data.data): %.1f" % (len(data.data))) #zbedit4 for print
        # for q in range(0,14):
        #     rospy.loginfo("data.data[%.1f]: %.1f" % (q, data.data[q])) #zbedit4 for print
        # rospy.loginfo("size: %.1f" % (size)) #zbedit4 for print
        for i in range(size):
            # self.ob.append(np.array(data.data[5*i:5*i+5])) //original
            # self.ob.append(np.array(data.data[10*i:10*i+10])) #zbedit4
            self.ob.append(np.array(data.data[7*i:7*i+7])) #zbedit4try
        self.obstacle_lock.release()

    def __global_path_cb(self, path):
        self.global_path_lock.acquire()
        size = len(path.poses)
        if size > 0:
            self.global_path = np.zeros([size, 3])
            for i in range(size):
                self.global_path[i, 0] = path.poses[i].pose.position.x
                self.global_path[i, 1] = path.poses[i].pose.position.y
        self.global_path_lock.release()

    def __publish_local_plan(self, input_sol, state_sol):
        local_path = Path()
        local_plan = Float32MultiArray()
        local_path_vis = Marker()
        local_path_vis.type = Marker.LINE_LIST
        local_path_vis.scale.x = 0.05
        local_path_vis.color.g = local_path_vis.color.b = local_path_vis.color.a = 1.0

        local_path_vis.header.stamp = rospy.Time.now()
        local_path.header.stamp = rospy.Time.now()

        local_path.header.frame_id = "map"
        local_path_vis.header.frame_id = "map"

        for i in range(self.N):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = state_sol[i, 0]
            this_pose_stamped.pose.position.y = state_sol[i, 1]
            this_pose_stamped.pose.position.z = self.z
            this_pose_stamped.pose.orientation.x = 0
            this_pose_stamped.pose.orientation.y = 0
            this_pose_stamped.pose.orientation.z = 0
            this_pose_stamped.pose.orientation.w = 1
            this_pose_stamped.header.seq = i
            this_pose_stamped.header.stamp = rospy.Time.now()
            this_pose_stamped.header.frame_id = "map"
            local_path.poses.append(this_pose_stamped)
            for j in range(2):
                if len(input_sol[i]) != 0:
                    local_plan.data.append(input_sol[i][j])

            pt = Point()
            pt.x = state_sol[i, 0]
            pt.y = state_sol[i, 1]
            pt.z = self.z

            color = ColorRGBA()
            color.r = 1
            color.g = 0.82
            color.b = 0.1
            color.a = 1

            p1 = Point()
            p2 = Point()
            p3 = Point()
            p4 = Point()
            if i < self.N-1:
                x_diff = state_sol[i+1, 0]-state_sol[i, 0]
                y_diff = state_sol[i+1, 1]-state_sol[i, 1]
                if x_diff != 0 and y_diff != 0:
                    theta = np.arctan2(y_diff, x_diff)
                else:
                    theta = 0
                w = 0.7
                l = 0.92
                p1.z = pt.z-0.01
                p1.x = 0.5*(l*np.cos(theta)-w*np.sin(theta)) + pt.x
                p1.y = 0.5*(l*np.sin(theta)+w*np.cos(theta)) + pt.y
                p2.z = pt.z-0.01
                p2.x = 0.5*(-l*np.cos(theta)-w*np.sin(theta)) + pt.x
                p2.y = 0.5*(-l*np.sin(theta)+w*np.cos(theta)) + pt.y
                p3.z = pt.z-0.01
                p3.x = 0.5*(-l*np.cos(theta)+w*np.sin(theta)) + pt.x
                p3.y = 0.5*(-l*np.sin(theta)-w*np.cos(theta)) + pt.y
                p4.z = pt.z-0.01
                p4.x = 0.5*(l*np.cos(theta)+w*np.sin(theta)) + pt.x
                p4.y = 0.5*(l*np.sin(theta)-w*np.cos(theta)) + pt.y

                local_path_vis.points.append(p1)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p2)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p2)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p3)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p3)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p4)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p4)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p1)
                local_path_vis.colors.append(color)
                local_path_vis.pose.orientation.x = 0
                local_path_vis.pose.orientation.y = 0
                local_path_vis.pose.orientation.z = 0
                local_path_vis.pose.orientation.w = 1

        self.__pub_local_path_vis.publish(local_path_vis)
        self.__pub_local_path.publish(local_path)
        self.__pub_local_plan.publish(local_plan)

    def choose_goal_state(self):
        self.curr_pose_lock.acquire()
        self.global_path_lock.acquire()
        if self.global_path is None or self.curr_state is None:
            self.curr_pose_lock.release()
            self.global_path_lock.release()
            return False

        waypoint_num = self.global_path.shape[0]
        num = np.argmin(np.array([distance_global(self.curr_state, self.global_path[i]) for i in range(waypoint_num)]))

        # Scale of the path clipped
        scale = 2
        num_list = []
        for i in range(self.N):
            num_path = min(waypoint_num - 1, num + i * scale)
            num_list.append(num_path)

        for k in range(self.N):
            self.goal_state[k] = self.global_path[num_list[k]]

        self.curr_pose_lock.release()
        self.global_path_lock.release()
        return True
    
    def __user_cmd_cb(self, command):
        self.user_input = command
        self.current_vel_time = rospy.Time.now()
        
    def get_custom_traj(self):
        lin_vel = self.user_input.linear.x
        ang_vel = self.user_input.angular.z
        custom_traj = Path()

        way_point = PoseStamped()
        temp_point = PoseStamped()
        T = 0.1
        custom_traj.header.frame_id = "map"
        for i in range(self.N):
            way_point.header.frame_id = "base_link"
            way_point.header.stamp = rospy.Time(0)
            theta = ang_vel * T * i
            if(theta != 0):
                way_point.pose.position.x = lin_vel / ang_vel * np.sin(theta)
                way_point.pose.position.y = lin_vel / ang_vel * (1 - np.cos(theta))
                rot_qua = quaternion_from_euler(0, 0, theta)
                way_point.pose.orientation.x = rot_qua[0]
                way_point.pose.orientation.y = rot_qua[1]
                way_point.pose.orientation.z = rot_qua[2]
                way_point.pose.orientation.w = rot_qua[3]
            else:
                way_point.pose.position.x = lin_vel * i * T
                way_point.pose.position.y = 0
                rot_qua = quaternion_from_euler(0, 0, theta)
                way_point.pose.orientation.x = rot_qua[0]
                way_point.pose.orientation.y = rot_qua[1]
                way_point.pose.orientation.z = rot_qua[2]
                way_point.pose.orientation.w = rot_qua[3]
            temp_point = self.tf_listener.transformPose(custom_traj.header.frame_id, way_point)
            custom_traj.poses.append(temp_point)
        self.__pub_user_path.publish(custom_traj)

        self.user_input.linear.x = 0
        self.user_input.angular.z = 0

        return custom_traj

        
    def MPC_ellip(self):
        self.curr_pose_lock.acquire()
        self.global_path_lock.acquire()
        self.obstacle_lock.acquire()

        opti = ca.Opti()
        # parameters for optimization
        T = 0.1
        #gamma_k = 0.15
        gamma_k = 0.3

        v_max = 1.0
        v_min = 0.1
        omega_max = 1.2

        opt_x0 = opti.parameter(3)

        # state variables
        opt_states = opti.variable(self.N + 1, 3)
        opt_controls = opti.variable(self.N, 2)
        v = opt_controls[:, 0]
        omega = opt_controls[:, 1]

        def f(x_, u_): return ca.vertcat(*[u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2]), u_[1]])

        def exceed_ob(ob_):
            #zbedit4 check:
            # rospy.loginfo("ob_[0]: %.1f" % (ob_[0])) #zbedit4 for print
            # rospy.loginfo("ob_[1]: %.1f" % (ob_[1])) #zbedit4 for print
            # rospy.loginfo("ob_[2]: %.1f" % (ob_[2])) #zbedit4 for print
            # rospy.loginfo("ob_[3]: %.1f" % (ob_[3])) #zbedit4 for print
            # rospy.loginfo("ob_[4]: %.1f" % (ob_[4])) #zbedit4 for print
            # rospy.loginfo("ob_[5]: %.1f" % (ob_[5])) #zbedit4 for print
            # rospy.loginfo("ob_[6]: %.1f" % (ob_[6])) #zbedit4 for print

            #original:
            # l_long_axis = ob_[2]
            # l_short_axis = ob_[3]
            # long_axis = np.array([np.cos(ob_[4]) * l_long_axis, np.sin(ob_[4]) * l_long_axis])

            #zbedit4:
            # l_long_axis = ob_[6] + ob_[8] #should represent sigma_h+sigma_r
            # l_short_axis = 2*ob_[7] #should represent 2*sigma_s
            # long_axis = np.array([np.cos(ob_[9]) * l_long_axis, np.sin(ob_[9]) * l_long_axis])

            #zbedit4try:
            l_long_axis = ob_[2] + ob_[4] #should represent sigma_h+sigma_r
            l_short_axis = 2*ob_[3] #should represent 2*sigma_s
            long_axis = np.array([np.cos(ob_[5]) * l_long_axis, np.sin(ob_[5]) * l_long_axis])

            ob_vec = np.array([ob_[0], ob_[1]])
            center_vec = self.goal_state[self.N-1, :2] - ob_vec
            dist_center = np.linalg.norm(center_vec)
            cos_ = np.dot(center_vec, long_axis.T) / (dist_center * l_long_axis)

            if np.abs(cos_) > 0.1:
                tan_square = 1 / (cos_ ** 2) - 1
                d = np.sqrt((l_long_axis ** 2 * l_short_axis ** 2 * (1 + tan_square) / (
                    l_short_axis ** 2 + l_long_axis ** 2 * tan_square)))
            else:
                d = l_short_axis

            cross_pt = ob_vec + d * center_vec / dist_center

            vec1 = self.goal_state[self.N-1, :2] - cross_pt
            vec2 = self.curr_state[:2] - cross_pt
            theta = np.dot(vec1.T, vec2)

            return theta > 0

        def h(curpos_, ob_):
            #safe_dist = 1.3 # for scout
            safe_dist = 0.7 # for jackal

            #original:
            # c = ca.cos(ob_[4])
            # s = ca.sin(ob_[4])
            # a = ca.MX([ob_[2]])
            # b = ca.MX([ob_[3]])

            #zbedit4:
            # c = ca.cos(ob_[9])
            # s = ca.sin(ob_[9])
            # a = ca.MX([ob_[6] + ob_[8]])
            # b = ca.MX([2*ob_[7]])

            #zbedit4try:
            c = ca.cos(ob_[5])
            s = ca.sin(ob_[5])
            a = ca.MX([ob_[2] + ob_[4]])
            b = ca.MX([2*ob_[3]])

            ob_vec = ca.MX([ob_[0], ob_[1]])
            center_vec = curpos_[:2] - ob_vec.T

            dist = b * (ca.sqrt((c ** 2 / a ** 2 + s ** 2 / b ** 2) * center_vec[0] ** 2 + (s ** 2 / a ** 2 + c ** 2 / b ** 2) *
                                center_vec[1] ** 2 + 2 * c * s * (1 / a ** 2 - 1 / b ** 2) * center_vec[0] * center_vec[1]) - 1) - safe_dist
            
            return dist

        def quadratic(x, A):
            return ca.mtimes([x, A, x.T])

        # init_condition
        opti.subject_to(opt_states[0, :] == opt_x0.T)

        # Position Boundaries
        if(distance_global(self.curr_state, self.global_path[-1, :2]) > 1):
            opti.subject_to(opti.bounded(v_min, v, v_max))
        else:
            opti.subject_to(opti.bounded(-v_min, v, v_max))

        opti.subject_to(opti.bounded(-omega_max, omega, omega_max))

        # System Model constraints
        for i in range(self.N):
            x_next = opt_states[i, :] + T * f(opt_states[i, :], opt_controls[i, :]).T
            opti.subject_to(opt_states[i + 1, :] == x_next)

        num_obs = int(len(self.ob)/self.N)
        # rospy.loginfo("len(self.ob): %.1f" % (len(self.ob))) #zbedit4 for print
        # rospy.loginfo("self.N: %.1f" % (self.N)) #zbedit4 for print
        # rospy.loginfo("num_obs: %.1f" % (num_obs)) #zbedit4 for print
        

        #original:
        # for j in range(num_obs):
        #     if not exceed_ob(self.ob[25*j]):
        #         for i in range(self.N-1):
        #             opti.subject_to(h(opt_states[i + 1, :], self.ob[j * 25 + i + 1]) >=
        #                             (1 - gamma_k) * h(opt_states[i, :], self.ob[j * 25 + i]))
                    
        #zbedit4:
        for j in range(num_obs):
            if not exceed_ob(self.ob[35*j]):
                for i in range(self.N-1):
                    opti.subject_to(h(opt_states[i + 1, :], self.ob[j * 35 + i + 1]) >=
                                    (1 - gamma_k) * h(opt_states[i, :], self.ob[j * 35 + i]))

        obj = 0
        R = np.diag([0.1, 0.02])
        A = np.diag([0.1, 0.02])

        # -------------Modify goal state here!!! --------------------

        # if(self.user_input.linear.x != 0 or self.user_input.angular.x):
        #     user_path = self.get_custom_traj()
        #     time_diff = self.current_vel_time.to_sec() - self.last_vel_time.to_sec()
        #     self.last_vel_time = self.current_vel_time

        #     time_thrd = 0.5

        #     if(time_diff < time_thrd):
        #         self.time_step += 1
        #     else:
        #         self.time_step = 0
            
        #     weight = self.get_weight_exp(self.time_step)

        #     for i in range(self.N):
        #         self.goal_state[i, 0] = (1 - weight) * self.goal_state[i, 0] + weight * user_path.poses[i].pose.position.x
        #         self.goal_state[i, 1] = (1 - weight) * self.goal_state[i, 1] + weight * user_path.poses[i].pose.position.y


        # -------------End modify goal state-------------------------
        for i in range(self.N):
            Q = np.diag([1.0+0.05*i,1.0+0.05*i, 0.02+0.005*i])
            if i < self.N-1:
                obj += 0.1 * quadratic(opt_states[i, :] - self.goal_state[[i]], Q) + quadratic(opt_controls[i, :], R)
            else:
                obj += 0.1 * quadratic(opt_states[i, :] - self.goal_state[[i]], Q)
        Q = np.diag([1.0,1.0, 0.02])*5
        obj += quadratic(opt_states[self.N-1, :] - self.goal_state[[self.N-1]], Q)

        opti.minimize(obj)
        opts_setting = {'ipopt.max_iter': 2000, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-3,
                        'ipopt.acceptable_obj_change_tol': 1e-3}
        opti.solver('ipopt', opts_setting)
        opti.set_value(opt_x0, self.curr_state)

        try:
            sol = opti.solve()
            u_res = sol.value(opt_controls)
            state_res = sol.value(opt_states)

            self.last_input = u_res
            self.last_state = state_res
            self.mpc_success = True

        except:
            rospy.logerr("Infeasible Solution")

            if self.mpc_success:
                self.mpc_success = False
            else:
                for i in range(self.N-1):
                    self.last_input[i] = self.last_input[i+1]
                    self.last_state[i] = self.last_state[i+1]
                    self.last_input[self.N-1] = np.zeros([1, 2])

            u_res = self.last_input
            state_res = self.last_state
        
        # Modify the total input signal
        if(self.user_input.linear.x != 0 or self.user_input.angular.z != 0):
            # user_path = self.get_custom_traj()
            time_diff = self.current_vel_time.to_sec() - self.last_vel_time.to_sec()
            self.last_vel_time = self.current_vel_time

            time_thrd = 0.5

            if(time_diff < time_thrd):
                self.time_step += 1
            else:
                self.time_step = 0
            
            weight = self.get_weight_exp(self.time_step)

            for i in range(self.N):
                u_res[i, 0] = (1 - weight) * u_res[i,0] + weight * self.user_input.linear.x
                u_res[i, 1] = (1 - weight) * u_res[i,1] + weight * self.user_input.angular.z

            self.user_input.linear.x = 0
            self.user_input.angular.z = 0


        self.curr_pose_lock.release()
        self.global_path_lock.release()
        self.obstacle_lock.release()
        return state_res, u_res
    
    def get_weight_exp(self, time_step):
        k = 0.3
        return 1 - np.exp(-k * time_step)

if __name__ == '__main__':
    rospy.init_node("phri_planner")
    phri_planner = Local_Planner()

    rospy.spin()
