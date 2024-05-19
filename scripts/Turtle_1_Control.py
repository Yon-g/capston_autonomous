#!/usr/bin/env python3
from socket import *
import numpy as np
from matplotlib import pyplot as plt
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from capstone import CubicSpline

k = 1  # control gain
Kp = 1.0  # speed proportional gain
L = 5  # 0.15cm
max_steer = 0.1 # [rad] max steering angle

class Control(Node):

    def __init__(self):
        super().__init__('Control')
        # self.bot1_odom_sub = self.create_subscription(Odom, '/bot1/ab_odom', self.bot1_odom_callback, 10)
        # self.bot1_control_pub = self.create_publisher(Twist, '/bot1/cmd_vel', 10)

        self.local_sub1 = self.create_subscription(Odometry, '/turtle_1/real_local', self.real_local_cb1, 10)
        self.path_sub1 = self.create_subscription(Int32MultiArray, '/turtle_1/path', self.path_cb1, 10)
        self.contorl_pub = self.create_publisher(Twist, '/turtle_1/cmd_vel', 10)

        self.x = None
        self.y = None
        self.ori = None
        self.is_local_cb = False
        self.is_path_cb = False

        self.cx = None
        self.cy = None
        self.cyaw = None

        self.path_x = []
        self.path_y = []

        while(self.x is None or not self.is_path_cb):
            print("local : ",self.x)
            print("path  : ",self.is_path_cb)
            print("odom not sub")
            rclpy.spin_once(self)
        self.current_target_idx, _  = self.calc_target_index(self.cx, self.cy)

        self.timer = self.create_timer(0.01, self.pub_cmd_vel)

    def real_local_cb1(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.ori = msg.pose.pose.position.z

        self.is_local_cb = True

    def path_cb1(self, msg):
        self.path_x = []
        self.path_y = []

        for i in range(len(msg.data)//2):
            self.path_x.append(msg.data[2*i])
            self.path_y.append(msg.data[2*i+1])
        if(len(self.path_x)) >2:
            self.cx, self.cy, self.cyaw, ck, s = CubicSpline.cubic_spline_planner.calc_spline_course(self.path_x, self.path_y, ds=0.5)

            self.is_path_cb = True

    def pub_cmd_vel(self):
        di, self.current_target_idx = self.stanley_control(self.cx, self.cy, self.cyaw, self.current_target_idx)
        # dist_error = math.sqrt((self.x-self.cx[self.current_target_idx])**2 +(self.y-self.cy[self.current_target_idx])**2)
        # if di>0:
        #     add_p =  - 0.19*(1 / (1 + np.exp(-180 * (di - 0.04))))
        # else:
        #     add_p =  0.19*(1 / (1 + np.exp(-180 * (-1*di - 0.04))))
        velocity = 0.1
        ai = velocity - np.clip(np.abs(di)*velocity*5, -1*velocity/2, velocity/2)
        
        cmd = Twist()
        cmd.linear.x = ai
        # if di > max_steer:
        #     di = max_steer
        # elif di < max_steer * -1:
        #     di = -1* max_steer
        cmd.angular.z = di

        self.contorl_pub.publish(cmd)

    def calc_target_index(self, cx, cy):
        """
        Compute index in the trajectory list of the target.

        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = self.x
        fy = self.y

        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(self.ori + np.pi / 2),
                        -np.sin(self.ori + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle*3/100
    
    def stanley_control(self, cx, cy, cyaw, last_target_idx):
        """
        Stanley steering control.

        :param state: (State object)
        :param cx: ([float])
        :param cy: ([float])
        :param cyaw: ([float])
        :param last_target_idx: (int)
        :return: (float, int)
        """
        current_target_idx, error_front_axle = self.calc_target_index(cx, cy)
        # print(current_target_idx, last_target_idx, len(cx)-1)



        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        if current_target_idx >= len(cx):
            current_target_idx = len(cx)-1
        elif current_target_idx < 0:
            current_target_idx = 0

        # print(current_target_idx, last_target_idx, len(cx)-1)
        # theta_e corrects the heading error
        theta_e = self.normalize_angle(cyaw[current_target_idx] - self.ori)/math.pi
        # theta_d corrects the cross track error
        theta_d = np.arctan2(k * error_front_axle, 0.1)/math.pi*0.5
        # Steering control
        delta = (theta_e + theta_d) * 1.2
        print("path yaw : ",cyaw[current_target_idx])
        print("local yaw : ",self.ori)
        print("long error: ",error_front_axle)
        print("theta_e : ",theta_e)
        print("theta_d : ",theta_d)
        print(theta_e,theta_d)
        print()

        return delta, current_target_idx
    
    def normalize_angle(self,angle):
        """
        Normalize an angle to [-pi, pi].

        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        return CubicSpline.utils.angle_mod(angle)
    
def main(args=None):
    rclpy.init(args=args)
    center_node = Control()

    rclpy.spin(center_node)
    center_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    
        