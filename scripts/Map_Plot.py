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

class Map_Plot(Node):

    def __init__(self):
        super().__init__('Control')
        self.local_sub1 = self.create_subscription(Odometry, '/turtle_1/real_local', self.real_local_cb1, 10)
        # self.local_sub1 = self.create_subscription(Odometry, '/turtle_1/real_local', self.real_local_cb1, 10)
        # self.local_sub1 = self.create_subscription(Odometry, '/turtle_1/real_local', self.real_local_cb1, 10)
        # self.local_sub1 = self.create_subscription(Odometry, '/turtle_1/real_local', self.real_local_cb1, 10)

        self.path_sub1 = self.create_subscription(Int32MultiArray, '/turtle_1/path', self.path_cb1, 10)
        # self.path_sub1 = self.create_subscription(Int32MultiArray, '/turtle_1/path', self.path_cb1, 10)
        # self.path_sub1 = self.create_subscription(Int32MultiArray, '/turtle_1/path', self.path_cb1, 10)
        # self.path_sub1 = self.create_subscription(Int32MultiArray, '/turtle_1/path', self.path_cb1, 10)

        self.map_sub = self.create_subscription(Float32MultiArray, '/total_obstacles', self.map_cb, 10)

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

        self.map = []
        self.is_map_cb = False
        
        print("map ploting start")
        self.timer = self.create_timer(0.01, self.plot_map)

    def real_local_cb1(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.ori = msg.pose.pose.orientation.z

        self.is_local_cb = True

    def map_cb(self, msg):

        self.map = []

        for i in range(len(msg.data)//3):
            self.map.append([msg.data[3*i],msg.data[3*i+1],msg.data[3*i+2]])

        self.is_map_cb = True

    def path_cb1(self, msg):
        self.path_x = []
        self.path_y = []

        for i in range(len(msg.data)//2):
            self.path_x.append(msg.data[2*i])
            self.path_y.append(msg.data[2*i+1])

        self.is_path_cb = True

    def plot_map(self):

        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if self.is_path_cb:
            plt.plot(self.path_x,self.path_y,'sg')
        if self.is_local_cb:
            plt.plot(self.x,self.y,'sb')
        if self.is_map_cb:
            for obstacle in self.map:
                if obstacle[2] <= 0.25:
                    plt.plot(obstacle[0],obstacle[1],color='gainsboro',marker='s')
                if 0.25 < obstacle[2] <= 0.5:
                    plt.plot(obstacle[0],obstacle[1],color='lightgray',marker='s')
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

    
        