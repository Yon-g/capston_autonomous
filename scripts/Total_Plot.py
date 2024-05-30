#!/usr/bin/env python3
from socket import *
import numpy as np
from matplotlib import pyplot as plt
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from capstone import CubicSpline

import threading

MODE = 1

class Map_Plot(Node):

    def __init__(self):
        super().__init__('Control')
        self.local_sub1 = self.create_subscription(Odometry, '/turtle_1/real_local', self.real_local_cb1, 10)
        self.local_sub1 = self.create_subscription(Odometry, '/turtle_2/real_local', self.real_local_cb2, 10)
        self.local_sub1 = self.create_subscription(Odometry, '/turtle_3/real_local', self.real_local_cb3, 10)
        self.local_sub1 = self.create_subscription(Odometry, '/turtle_4/real_local', self.real_local_cb4, 10)

        self.path_sub = self.create_subscription(Int32MultiArray, '/turtle_4/path', self.path_cb, 10)

        self.ob_sub = self.create_subscription(Float32MultiArray, '/total_obstacles', self.total_ob_cb, 10)
        self.turtle_mab_sub = self.create_subscription(Int32MultiArray, '/turtle_4/map', self.path_map_cb, 10)

        self.turtle1_x = None
        self.turtle1_y = None
        self.turtle1_ori = None
        self.turtle1_is_local_cb = False

        self.turtle2_x = None
        self.turtle2_y = None
        self.turtle2_ori = None
        self.turtle2_is_local_cb = False

        self.turtle3_x = None
        self.turtle3_y = None
        self.turtle3_ori = None
        self.turtle3_is_local_cb = False

        self.turtle4_x = None
        self.turtle4_y = None
        self.turtle4_ori = None
        self.turtle4_is_local_cb = False

        self.turtle_is_turtle_map_cb = False
        self.turtle_is_path_cb = False

        self.map = []
        self.is_map_cb = False
        
        print("map ploting start")


    def total_ob_cb(self, msg):
        self.map = []
        for i in range(len(msg.data)//3):
            self.map.append([msg.data[3*i],msg.data[3*i+1],msg.data[3*i+2]])
        self.is_map_cb = True

    def real_local_cb1(self, msg):
        self.turtle1_x = msg.pose.po2e.position.x
        self.turtle1_y = msg.pose.pose.position.y
        self.turtle1_ori = msg.pose.pose.orientation.z

        self.turtle1_is_local_cb = True

    def real_local_cb2(self, msg):
        self.turtle2_x = msg.pose.pose.position.x
        self.turtle2_y = msg.pose.pose.position.y
        self.turtle2_ori = msg.pose.pose.orientation.z

        self.turtle2_is_local_cb = True

    def real_local_cb3(self, msg):
        self.turtle3_x = msg.pose.pose.position.x
        self.turtle3_y = msg.pose.pose.position.y
        self.turtle3_ori = msg.pose.pose.orientation.z

        self.turtle3_is_local_cb = True

    def real_local_cb4(self, msg):
        self.turtle4_x = msg.pose.pose.position.x
        self.turtle4_y = msg.pose.pose.position.y
        self.turtle4_ori = msg.pose.pose.orientation.z

        self.turtle4_is_local_cb = True

    def path_cb(self, msg):
        self.turtle_path_x = []
        self.turtle_path_y = []

        for i in range(len(msg.data)//2):
            self.turtle_path_x.append(msg.data[2*i])
            self.turtle_path_y.append(msg.data[2*i+1])

        self.turtle_is_path_cb = True
    
    def path_map_cb(self, msg):
        self.turtle_map_x = []
        self.turtle_map_y = []

        for i in range(len(msg.data)//2):
            self.turtle_map_x.append(msg.data[2*i])
            self.turtle_map_y.append(msg.data[2*i+1])

        self.turtle_is_turtle_map_cb = True

    def plot_map(self):
        
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        if self.turtle_is_turtle_map_cb:
            plt.plot(self.turtle_map_x,self.turtle_map_y,"ok")
        if MODE == 1:
            if self.turtle_is_path_cb:
                plt.plot(self.turtle_path_x,self.turtle_path_y,'sg')
            if self.turtle1_is_local_cb:
                plt.plot(self.turtle1_x,self.turtle1_y,'sr')
            if self.turtle2_is_local_cb:
                plt.plot(self.turtle2_x,self.turtle2_y,'sb')
            if self.turtle3_is_local_cb:
                plt.plot(self.turtle3_x,self.turtle3_y,'sb')
            if self.turtle4_is_local_cb:
                plt.plot(self.turtle4_x,self.turtle4_y,'sy')

            if self.is_map_cb:
                for obstacle in self.map:
                    # if obstacle[2] <= 0.25:
                    #     plt.plot(obstacle[0],obstacle[1],color='gainsboro',marker='s')
                    # if 0.25 < obstacle[2] <= 0.5:
                        # plt.plot(obstacle[0],obstacle[1],color='lightgray',marker='s')
                    if 0.5 < obstacle[2] <= 0.75:
                        plt.plot(obstacle[0],obstacle[1],color='darkgrey',marker='s')
                    if 0.75 < obstacle[2] < 1.0:
                        plt.plot(obstacle[0],obstacle[1],color='grey',marker='s')
                    if 1.0 <= obstacle[2] <= 2.24:
                        plt.plot(obstacle[0],obstacle[1],color='black',marker='s')
                    if 2.24 < obstacle[2] < 30:
                        plt.plot(obstacle[0],obstacle[1],color='green',marker='s')
                    if 20 < obstacle[2] < 100:
                        plt.plot(obstacle[0],obstacle[1],color='blue',marker='s')
                    if 100 < obstacle[2]:
                        plt.plot(obstacle[0],obstacle[1],color='red',marker='s')
        # elif MODE == 2:
            
            
        plt.xlim(0, 100)  # x축 범위 설정
        plt.ylim(0, 100)  # y축 범위 설정

        plt.grid(True)
        plt.pause(0.01)
    
def main(args=None):
    rclpy.init(args=args)
    map_plot = Map_Plot()

    rclpy.spin(map_plot)
    map_plot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    
        