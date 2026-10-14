#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import numpy as np
import cvxpy as cp
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState

# 全局变量
dt = 0.1  # 控制周期
x_cur = np.zeros([4])  # 当前状态 [vx, px, vy, py]
last_time = 0.0
state_received = False

def esti_state_callback(msg):
    """
    接收卡尔曼滤波后的位置，并通过差分计算当前速度，更新全局状态 x_cur
    """
    global x_cur, dt, last_time, state_received
    
    current_time = rospy.get_time()
    actual_dt = current_time - last_time if last_time > 0 else dt
    if actual_dt <= 0: actual_dt = dt
    
    # 使用差分估算速度 vx, vy
    vx = (msg.pose.position.x - x_cur[1]) / actual_dt
    vy = (msg.pose.position.y - x_cur[3]) / actual_dt
    
    x_cur[0] = vx
    x_cur[1] = msg.pose.position.x
    x_cur[2] = vy
    x_cur[3] = msg.pose.position.y
    
    last_time = current_time
    state_received = True

class MPC_controller:
    def __init__(self, x_ref, dt):
        self.pred_steps = 15                                 # 预测步长 (对于迷宫，步长不需要太大，15步即1.5秒足够)
        self.x_ref = x_ref                                   # 全局参考轨迹
        
        # 迷宫控制的惩罚矩阵 (需要极度严苛的位置追踪，防止撞墙，降低对速度的苛求)
        self.Q = np.diag([0.1, 100.0, 0.1, 100.0])           
        self.R = np.diag([0.5, 0.5])                         
        self.Q_f = np.diag([0.1, 150.0, 0.1, 150.0])         
        
        self.mass = 10.0                                     
        self.current_idx = 0                                 
        self._discretization_Func(dt)                        
    
    def _discretization_Func(self, dt):
        self.Atilde = np.array([
            [1,  0, 0,  0],
            [dt, 1, 0,  0],
            [0,  0, 1,  0],
            [0,  0, dt, 1]
        ])
        self.Btilde = np.array([
            [dt/self.mass,      0],
            [dt*dt/2/self.mass, 0],
            [0,      dt/self.mass],
            [0, dt*dt/2/self.mass]
        ])
        
    def update(self, x_cur):
        # 轨迹结束处理
        if self.current_idx + self.pred_steps >= len(self.x_ref):
            self.current_idx = len(self.x_ref) - self.pred_steps - 1
            if self.current_idx < 0:
                return np.zeros(2)

        x = cp.Variable((self.pred_steps + 1, 4))
        u = cp.Variable((self.pred_steps, 2))
        cost = 0
        constraints = [x[0] == x_cur]

        for i in range(self.pred_steps):
            ref_idx = self.current_idx + i + 1
            constraints += [x[i + 1] == self.Atilde * x[i] + self.Btilde * u[i]]
            # 严格限制输出力的大小，防止迷宫内动作过大
            constraints += [u[i] >= -20.0, u[i] <= 20.0] 
            cost += cp.quad_form(x[i + 1] - self.x_ref[ref_idx], self.Q) + cp.quad_form(u[i], self.R)

        terminal_idx = self.current_idx + self.pred_steps
        cost += cp.quad_form(x[self.pred_steps] - self.x_ref[terminal_idx], self.Q_f)

        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve()

        self.current_idx += 1

        if prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            return u[0].value
        else:
            rospy.logwarn("MPC 问题无法求解，输出 0 控制律")
            return np.zeros(2)

def generate_maze_trajectory(dt, speed=0.4):
    """
    将离散的拐角点，插值成一条密集的带有时间戳的连续轨迹
    """
    # 迷宫目标点 (加上了起点 (0,0))
    targets = [
        (0.0, 0.0),      # 起点
        (0.0, -1.6),     # 1
        (-1.6, -1.6),    # 2
        (-1.6, 1.6),     # 3
        (0.0, 1.6),      # 4
        (0.0, 2.6),      # 5
        (-2.6, 2.6),     # 6
        (-2.6, -2.6),    # 7
        (0.0, -2.6),     # 8
        (0.0, -3.6),     # 9
        (-3.6, -3.6),    # 10
        (-3.6, 3.6),     # 11
        (0.0, 3.6),      # 12
        (0.0, 4.6),      # 13
        (-4.6, 4.6)      # 14
    ]
    
    x_ref_list = []
    
    for i in range(len(targets) - 1):
        p1 = np.array(targets[i])
        p2 = np.array(targets[i+1])
        
        # 计算两点距离和所需时间
        dist = np.linalg.norm(p2 - p1)
        T = dist / speed
        num_steps = max(int(T / dt), 2)
        
        # 线性插值生成位置
        px_vals = np.linspace(p1[0], p2[0], num_steps)
        py_vals = np.linspace(p1[1], p2[1], num_steps)
        
        # 计算匀速期望速度
        vx = (p2[0] - p1[0]) / T
        vy = (p2[1] - p1[1]) / T
        
        # 组装这段直线的状态 [vx, px, vy, py]
        for j in range(num_steps):
            x_ref_list.append([vx, px_vals[j], vy, py_vals[j]])
            
        # ！！核心逻辑：在每个拐角处强行停留 1.5 秒 (15个 step) ！！
        # 这样 MPC 在预测时，会看到未来轨迹在这里停顿，从而提前减速，精准走出直角，防止切内角撞墙
        for j in range(15):
            x_ref_list.append([0.0, p2[0], 0.0, p2[1]])
            
    # 末端保持停留在最后一个点
    for j in range(50):
        x_ref_list.append([0.0, targets[-1][0], 0.0, targets[-1][1]])
        
    return np.array(x_ref_list)

def controller():
    global dt, x_cur, state_received
    rospy.init_node('mpc_maze_controller', anonymous=True)
    pub = rospy.Publisher('/robot/control', Twist, queue_size=10)
    rospy.Subscriber("/robot/esti_model_state", ModelState, esti_state_callback)

    rospy.loginfo("等待感知节点获取初始状态...")
    while not rospy.is_shutdown() and not state_received:
        rospy.sleep(0.1)

    rospy.loginfo("正在将迷宫路径转化为 MPC 密集轨迹...")
    x_ref = generate_maze_trajectory(dt, speed=0.5)
    rospy.loginfo("轨迹生成完毕，共有 {} 个控制点。".format(len(x_ref)))

    mpc = MPC_controller(x_ref, dt)
    rate = rospy.Rate(1.0 / dt)

    rospy.loginfo("MPC 迷宫脱脱计划启动！")

    try:
        while not rospy.is_shutdown():
            if mpc.current_idx >= len(x_ref) - mpc.pred_steps - 1:
                rospy.loginfo("成功逃出迷宫！")
                pub.publish(Twist())
                break

            u = mpc.update(x_cur)
            
            twist = Twist()
            twist.linear.x = u[0]   # X 轴推力
            twist.angular.z = u[1]  # Y 轴推力 (配合 driver.py 的解析)
            pub.publish(twist)
            
            rate.sleep()
            
    except Exception as e:
        rospy.logfatal("控制器异常退出: {}".format(e))
    finally:
        pub.publish(Twist())
        sys.exit(0)

if __name__ == '__main__':
    controller()
