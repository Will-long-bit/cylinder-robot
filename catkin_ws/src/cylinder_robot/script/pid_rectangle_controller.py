#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState

x_esti, y_esti = 0.0, 0.0

class PID_Controller:
    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.error_sum = 0
        self.output_min = output_min
        self.output_max = output_max
        self.last_time = rospy.get_time()

    def get_output(self, error):
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.1 

        # 积分分离：只有当误差小于 0.5 米时才开始积分，防止跑长途时积分过大
        if abs(error) < 0.5:
            self.error_sum += error * dt
        else:
            self.error_sum = 0.0
            
        error_dot = (error - self.last_error) / dt 
        
        self.last_error = error
        self.last_time = current_time
        
        output = self.kp * error + self.ki * self.error_sum + self.kd * error_dot
        
        if output > self.output_max: output = self.output_max
        elif output < self.output_min: output = self.output_min
        return output

def esti_state_callback(msg):
    global x_esti, y_esti
    x_esti = msg.pose.position.x
    y_esti = msg.pose.position.y

def controller():
    global x_esti, y_esti
    rospy.init_node('talker', anonymous=True)
    
    pub = rospy.Publisher('/robot/control', Twist, queue_size=10)
    rospy.Subscriber('/robot/esti_model_state', ModelState, esti_state_callback)
    
    rate = rospy.Rate(10)
    
    pid_x = PID_Controller(kp=25.0, ki=2.0, kd=35.0, output_min=-15.0, output_max=15.0)
    pid_y = PID_Controller(kp=25.0, ki=2.0, kd=35.0, output_min=-15.0, output_max=15.0)
    
    targets = [
        (4.0, 0.0),   
        (4.0, 3.0),   
        (0.0, 3.0),   
        (0.0, 0.0)    
    ]
    state = 0
    
    error_threshold = 0.03 
    steady_count = 0

    print("PID Controller Started. Drawing Standard Rectangle...")
    rospy.sleep(1.0)

    while not rospy.is_shutdown() and state < 4:
        target_x, target_y = targets[state]
        
        error_x = target_x - x_esti
        error_y = target_y - y_esti
        distance = math.sqrt(error_x**2 + error_y**2)
        
        if distance < error_threshold:
            steady_count += 1
        else:
            steady_count = 0
            
        # 必须在 3 厘米内连续停留 1.5 秒（15次循环），确保完全刹停才转弯
        if steady_count > 15: 
            print("Reached Point {}: ({}, {})".format(state, target_x, target_y))
            state += 1
            steady_count = 0
            continue
            
        twist = Twist()
        twist.linear.x = pid_x.get_output(error_x)
        twist.angular.z = pid_y.get_output(error_y)
        
        pub.publish(twist)
        rate.sleep()

    pub.publish(Twist())
    print("Rectangle complete. Exiting...")
    sys.exit(0)

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass