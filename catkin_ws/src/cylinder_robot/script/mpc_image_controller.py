#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import numpy as np
import cv2
import cvxpy as cp
from scipy.interpolate import interp1d
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
        self.pred_steps = 30                                 # 预测步长 (Horizon)
        self.x_ref = x_ref                                   # 全局参考轨迹
        self.Q = np.diag([0.1, 15.0, 0.1, 15.0])             # 状态误差惩罚矩阵 (着重惩罚位置误差 px, py)
        self.R = np.diag([1.0, 1.0])                         # 控制输入惩罚矩阵 (使得运动平滑)
        self.Q_f = np.diag([0.1, 20.0, 0.1, 20.0])           # 终端状态惩罚矩阵
        self.mass = 10.0                                     # 机器人质量
        self.current_idx = 0                                 # 当前追踪的路点索引 
        self._discretization_Func(dt)                        # 离散化动力学矩阵
    
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
            # 动力学约束
            constraints += [x[i + 1] == self.Atilde * x[i] + self.Btilde * u[i]]
            # 累加阶段 Cost
            cost += cp.quad_form(x[i + 1] - self.x_ref[ref_idx], self.Q) + cp.quad_form(u[i], self.R)

        # 终端 Cost
        terminal_idx = self.current_idx + self.pred_steps
        cost += cp.quad_form(x[self.pred_steps] - self.x_ref[terminal_idx], self.Q_f)

        # 构造并求解问题
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve()

        self.current_idx += 1

        if prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            return u[0].value
        else:
            rospy.logwarn("MPC 问题无法求解，输出 0 控制律")
            return np.zeros(2)

def generate_image_trajectory(filename="ikun", total_time=80.0, dt=0.1):
    """
    从 ../data/xxx.png 提取轮廓，生成参考轨迹 x_ref
    """
    # 获取工程根目录 (从 script 向上退一级)
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    image_path = os.path.join(base_dir, 'data', filename + '.png')
    
    if not os.path.exists(image_path):
        rospy.logerr("找不到图像文件: {}".format(image_path))
        sys.exit(1)

    image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    if image is None or image.shape[2] != 4:
        rospy.logerr("图片读取失败或没有透明(Alpha)通道！请确保是带透明背景的 PNG。")
        sys.exit(1)
    
    # 提取 Alpha 通道轮廓
    alpha_channel = image[:, :, 3]
    _, binary_mask = cv2.threshold(alpha_channel, 1, 255, cv2.THRESH_BINARY)
    
    contours_info = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]
    
    if not contours:
        rospy.logerr("在图片中未发现轮廓！")
        sys.exit(1)

    # 寻找最大轮廓
    max_contour = max(contours, key=cv2.contourArea)
    epsilon = cv2.arcLength(max_contour, True) / 500.0  # 控制多边形拟合精度
    approx = cv2.approxPolyDP(max_contour, epsilon, closed=True)
    points = approx.reshape(-1, 2).astype(np.float64)
    
    if not np.array_equal(points[0], points[-1]):
        points = np.vstack([points, points[0]])
    
    # 居中与缩放映射到 [-4.0, 4.0] 房间内
    min_xy = np.min(points, axis=0)
    max_xy = np.max(points, axis=0)
    center = (min_xy + max_xy) / 2.0
    points -= center
    
    current_span = np.max(max_xy - min_xy)
    target_span = 8.0  # [-4, 4]的跨度
    points *= (target_span / current_span)

    # 按曲率重采样与三次样条插值
    dx_dt = np.gradient(points[:, 0])
    dy_dt = np.gradient(points[:, 1])
    d2x_dt2 = np.gradient(dx_dt)
    d2y_dt2 = np.gradient(dy_dt)
    curvatures = np.abs((dx_dt * d2y_dt2 - d2x_dt2 * dy_dt) / (dx_dt**2 + dy_dt**2 + 1e-6)**1.5)
    mean_curvature = np.mean(curvatures)

    t = np.linspace(0, 1, len(points))
    t_new = []
    num_points = int(total_time / dt)

    for i in range(len(points) - 1):
        density = max(2, int((1 + abs(curvatures[i] / (mean_curvature+1e-6))) * num_points / len(points)))
        t_segment = np.linspace(t[i], t[i + 1], density)
        t_new.extend(t_segment[:-1])
    t_new.append(t[-1])
    
    fx = interp1d(t, points[:, 0], kind='cubic')
    fy = interp1d(t, points[:, 1], kind='cubic')
    
    x_new = fx(t_new)
    y_new = -fy(t_new) # 图像 y 轴向下，Gazebo y 轴向上，需要反转

    x_ref = np.zeros((len(x_new), 4))
    x_ref[:, 1] = x_new
    x_ref[:, 3] = y_new
    return x_ref

def get_full_reference(x_start, dt, filename="ikun", total_time=80.0, connect_time=5.0):
    """
    生成完整的带有接入段的轨迹，防止初始位置剧烈跳变导致 MPC 崩溃
    """
    main_ref = generate_image_trajectory(filename, total_time, dt)
    
    # 检查起始点距离，如果太远，生成三次多项式接入轨迹
    dist = np.linalg.norm(main_ref[0, [1, 3]] - x_start[[1, 3]])
    if dist > 0.1:
        connect_len = int(connect_time / dt)
        t_vals = np.linspace(0, 1, connect_len)
        x_conn = np.zeros((connect_len, 4))
        for i in [1, 3]: # px, py
            x0, x1 = x_start[i], main_ref[0, i]
            # 三次多项式连接
            x_conn[:, i] = 2*(x0 - x1)*(t_vals**3) + 3*(x1 - x0)*(t_vals**2) + x0
        
        # 停顿一下缓冲
        x_conn_wait = np.tile(x_conn[-1], (int(2.0/dt), 1))
        main_ref = np.vstack((x_conn, x_conn_wait, main_ref))
        
    # 末端保持
    x_ref_end = np.tile(main_ref[-1], (int(10.0/dt), 1))
    return np.vstack((main_ref, x_ref_end))

def controller():
    global dt, x_cur, state_received
    rospy.init_node('mpc_controller', anonymous=True)
    pub = rospy.Publisher('/robot/control', Twist, queue_size=10)
    rospy.Subscriber("/robot/esti_model_state", ModelState, esti_state_callback)

    # 1. 等待获取到卡尔曼滤波后的初始状态 (保证 x_start 正确)
    rospy.loginfo("等待感知节点与 Gazebo 的初始状态...")
    while not rospy.is_shutdown() and not state_received:
        rospy.sleep(0.1)

    # 2. 生成完整轨迹 (读取 data/ikun.png)
    image_name = "ufo"  
    rospy.loginfo("开始离线计算图像 {}.png 的 MPC 轨迹，请稍候...".format(image_name))
    x_ref = get_full_reference(x_cur, dt, filename=image_name, total_time=60.0, connect_time=5.0)
    rospy.loginfo("轨迹生成完毕，共 {} 个控制点。MPC 控制启动！".format(len(x_ref)))

    # 3. 初始化 MPC
    mpc = MPC_controller(x_ref, dt)
    rate = rospy.Rate(1.0 / dt)

    try:
        while not rospy.is_shutdown():
            if mpc.current_idx >= len(x_ref) - mpc.pred_steps - 1:
                rospy.loginfo("图像轨迹已走完！")
                pub.publish(Twist()) # 停下
                break

            u = mpc.update(x_cur)
            
            twist = Twist()
            twist.linear.x = u[0]   # 控制输入 ux (映射到 linear.x)
            twist.angular.z = u[1]  # 控制输入 uy (映射到 angular.z，与 driver.py 对应)
            pub.publish(twist)
            
            rate.sleep()
            
    except Exception as e:
        rospy.logfatal("MPC 控制器异常退出: {}".format(e))
    finally:
        pub.publish(Twist())
        sys.exit(0)

if __name__ == '__main__':
    controller()