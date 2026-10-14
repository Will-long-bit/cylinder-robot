#!/usr/bin/env python

import rospy
import rospkg
from scipy import linalg as lnr
from matplotlib import pyplot as plt
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
import os
import sys

# import the Kalman filter we finished last week
# from KalmanFilter import KalmanFilter

class KalmanFilter(object):
    # initialization the kalman filter. 
    #   x'(t) = Ax(t) + Bu(t) + w(t)
    #   y(t) = Cx(t) + v(t)
    #   x(0) ~ N(x_0, P_0)
    def __init__(self, mass, C, Sigma_w, Sigma_v, x_0, P_0):
        self.mass = mass
        self.C = C
    
        self.n = 4
        self.m = 2

        self.Sigma_w = Sigma_w
        self.Sigma_v = Sigma_v
        
        self.t = 0
        self.x = x_0
        self.P = P_0
        self.u = np.zeros([self.m, 1])

    # Given duration dt, return the discretization of A, B, Sigma_w. Just like what we do last week.
    def _discretization_Func(self, dt):
        Atilde = np.array([
            [1, 0,  0,  0],
            [dt,1,  0,  0],
            [0, 0,  1,  0],
            [0, 0,  dt, 1]
        ])
        Btilde = np.array([
            [dt/self.mass,      0],
            [dt*dt/2/self.mass, 0],
            [0,      dt/self.mass],
            [0, dt*dt/2/self.mass]
        ])
        q1 = self.Sigma_w[0,0]
        q2 = self.Sigma_w[1,1]
        q3 = self.Sigma_w[2,2]
        q4 = self.Sigma_w[3,3]
        Sigma_w_tilde = np.array([
            [dt*q1,         dt*dt/2*q1,                 0,          0],
            [dt*dt/2*q1,    (dt*q2)+(dt*dt*dt/3*q1),    0,          0],
            [0,             0,                          dt*q3,      dt*dt/2*q3],
            [0,             0,                          dt*dt/2*q3, (dt*q4)+(dt*dt*dt/3*q3)],
        ])

        return Atilde, Btilde, Sigma_w_tilde

    ################################################################################################
    ## ================================ Edit below here ================================ vvvvvvvvvvv
    # predict step
    def _predict_Step(self, ctrl_time):
        dt = ctrl_time - self.t
        self.t = ctrl_time
        At, Bt, Sigma = self._discretization_Func(dt)
        
        self.x = At.dot(self.x) + Bt.dot(self.u)
        self.P = At.dot(self.P).dot(At.T) + Sigma
        

    # correction step
    def _correction_Step(self, y):
        innovation = y - self.C.dot(self.x)
        S = self.C.dot(self.P).dot(self.C.T) + self.Sigma_v
        K = self.P.dot(self.C.T).dot(lnr.inv(S)) 
        self.x = self.x + K.dot(innovation)
        self.P = self.P - K.dot(self.C).dot(self.P)


    # when getting the control signal, execution the predict step, update the control signal
    def control_moment(self, u_new, time_now):
        self._predict_Step(time_now)
        self.u = u_new


    # when getting the observe info, execution the predict step, and then execution the correction step
    def observe_moment(self, y_new, time_now):
        self._predict_Step(time_now)
        self._correction_Step(y_new)

    ## ==========================================================================  ^^^^^^^^^^
    ## ===================================== Edit above =====================================
class Localization(object):
    def __init__(self):
        self.name = "cylinderRobot"
        # config the subscribe information
        rospy.Subscriber('/robot/control', Twist, self.callback_control)
        rospy.Subscriber('/robot/observe', LaserScan, self.callback_observe)
        rospy.Subscriber('gazebo/set_model_state', ModelState, self.callback_state)
        self.pub = rospy.Publisher("/robot/esti_model_state", ModelState, queue_size=10)
        # catch Ctrl+C. When you press Ctrl+C, call self.visualzation()
        rospy.on_shutdown(self.visualization)

        # initialize Kalman filter. 
        self.kf = KalmanFilter(
            mass = 10, 
            C = np.array([
                [0, 1, 0, 0],
                [0, 0, 0, 1]
            ]),
            Sigma_w = np.eye(4)*0.00001,
            Sigma_v = np.array([[0.02**2, 0],[0, 0.02**2]]),
            x_0 = np.zeros([4,1]),
            P_0 = np.eye(4)/1000
            )

        # list to save data for visualization
        self.x_esti_save = []
        self.x_esti_time = []
        self.x_true_save = []
        self.x_true_time = []
        self.p_obsv_save = []
        self.p_obsv_time = []

    ################################################################################################
    ## ================================ Edit below here ================================ vvvvvvvvvvv
    def callback_control(self, twist):
        # extract control signal from message
        u_new = np.array([[twist.linear.x], [twist.angular.z]])
        current_time = rospy.get_time()
        
        # call control moment function in Kalman filter
        self.kf.control_moment(u_new, current_time)
        

        # save data for visualization
        self.x_esti_save.append(self.kf.x)
        self.x_esti_time.append(current_time)

    def callback_observe(self, laserscan):
        # room boundaries (confirm exact values from your cylinder.world)
        right_wall_x = 5.0
        top_wall_y = 5.0

        # convert distance to absolute position
        px_obs = right_wall_x - laserscan.ranges[0]
        py_obs = top_wall_y - laserscan.ranges[1]

        y = np.array([[px_obs], [py_obs]])
        current_time = rospy.get_time()

        self.kf.observe_moment(y, current_time)

        self.x_esti_save.append(self.kf.x.copy())
        self.x_esti_time.append(current_time)
        self.p_obsv_save.append(y)
        self.p_obsv_time.append(current_time)

        self.sendStateMsg()
    ## ==========================================================================  ^^^^^^^^^^
    ## ===================================== Edit above =====================================

    # restore the true state of robot for visualization. You CAN NOT get them in real world.
    def callback_state(self, state):
        current_time = rospy.get_time()
        x = np.zeros([4,1])
        x[0,0] = state.twist.linear.x
        x[1,0] = state.pose.position.x
        x[2,0] = state.twist.linear.y
        x[3,0] = state.pose.position.y
        self.x_true_save.append(x)
        self.x_true_time.append(current_time)

    def sendStateMsg(self):
        msg = ModelState()
        msg.model_name = self.name
        msg.pose.position.x = self.kf.x[1]
        msg.pose.position.y = self.kf.x[3]
        self.pub.publish(msg)

    # visualzation
    def visualization(self):
        print("Visualizing......")
        t_esti = np.array(self.x_esti_time)
        x_esti = np.concatenate(self.x_esti_save, axis=1)
        
        p_obsv = np.concatenate(self.p_obsv_save, axis=1)
        t_obsv = np.array(self.p_obsv_time)

        t_true = np.array(self.x_true_time)
        x_true = np.concatenate(self.x_true_save, axis=1)

        fig_x = plt.figure(figsize=(16,9))
        
        # top-left: px over time
        ax1 = fig_x.add_subplot(2, 2, 1)
        ax1.plot(t_esti, x_esti[1,:].T, label="esti")
        ax1.plot(t_true, x_true[1,:].T, label="true")
        ax1.legend(bbox_to_anchor=(0.85,1), loc='upper left')
        ax1.set_title('px')
        
        # bottom-left: py over time
        ax2 = fig_x.add_subplot(2, 2, 3)
        ax2.plot(t_esti, x_esti[3,:].T, label="esti")
        ax2.plot(t_true, x_true[3,:].T, label="true")
        ax2.legend(bbox_to_anchor=(0.85,1), loc='upper left')
        ax2.set_title('py')
        
        # top-right: estimated vs true trajectory
        ax3 = fig_x.add_subplot(2, 2, 2)
        ax3.plot(x_esti[1,:].T, x_esti[3,:].T, label="esti")
        ax3.plot(x_true[1,:].T, x_true[3,:].T, label="true")
        ax3.legend(bbox_to_anchor=(0.1,1), loc='upper left')
        ax3.set_title('trace: esti with truth')
        
        # bottom-right: estimated vs observation
        ax4 = fig_x.add_subplot(2, 2, 4)
        ax4.plot(x_esti[1,:].T, x_esti[3,:].T, label='esti')
        ax4.plot(p_obsv[0,:].T, p_obsv[1,:].T, label='obsv')
        ax4.legend(bbox_to_anchor=(0.1,1), loc='upper left')
        ax4.set_title('trace: esti with observation')

        fig_path = rospkg.RosPack().get_path('cylinder_robot') + "/"
        fig_x.savefig(fig_path + 'fig_x.png', dpi=120)
        print("Visualization Complete. Image saved to: " + fig_path + 'fig_x.png')



if __name__ == '__main__':
    try:
        rospy.init_node('perception', anonymous=True)
        obs = Localization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass