#!/usr/bin/env python3
import math
from socket import *
import threading
import time
import copy

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState

test_mode = -1
# from cap_msg.msg import Obstacle
# from cap_msg.msg import Map

PATH = '/home/yongki/turtlebot3_ws/src/capstone/map/'

class Center_node(Node):

    def __init__(self):
        super().__init__('Center_node')
        self.bot1_odom_sub = self.create_subscription(Odometry, '/turtle_1/real_local', self.bot1_odom_callback, 10)
        self.bot2_odom_sub = self.create_subscription(Odometry, '/turtle_2/real_local', self.bot2_odom_callback, 10)
        self.bot3_odom_sub = self.create_subscription(Odometry, '/turtle_3/real_local', self.bot3_odom_callback, 10)
        self.bot4_odom_sub = self.create_subscription(Odometry, '/turtle_4/real_local', self.bot4_odom_callback, 10)

        self.bot1_path_sub = self.create_subscription(Int32MultiArray, '/turtle_1/path', self.bot1_path_callback, 10)
        self.bot2_path_sub = self.create_subscription(Int32MultiArray, '/turtle_2/path', self.bot2_path_callback, 10)
        self.bot3_path_sub = self.create_subscription(Int32MultiArray, '/turtle_3/path', self.bot3_path_callback, 10)
        self.bot4_path_sub = self.create_subscription(Int32MultiArray, '/turtle_4/path', self.bot4_path_callback, 10)

        self.bot1_control_mode_sub = self.create_subscription(Int32, '/turtle_1/control_mode', self.bot1_control_mode_callback, 10)
        self.bot2_control_mode_sub = self.create_subscription(Int32, '/turtle_2/control_mode', self.bot2_control_mode_callback, 10)
        self.bot3_control_mode_sub = self.create_subscription(Int32, '/turtle_3/control_mode', self.bot3_control_mode_callback, 10)
        self.bot4_control_mode_sub = self.create_subscription(Int32, '/turtle_4/control_mode', self.bot4_control_mode_callback, 10)
      
        self.bot1_battery_sub = self.create_subscription(BatteryState, '/turtle_1/battery_state', self.bot1_battery_callback, 10)
        self.bot2_battery_sub = self.create_subscription(BatteryState, '/turtle_2/battery_state', self.bot2_battery_callback, 10)
        self.bot3_battery_sub = self.create_subscription(BatteryState, '/turtle_3/battery_state', self.bot3_battery_callback, 10)
        self.bot4_battery_sub = self.create_subscription(BatteryState, '/turtle_4/battery_state', self.bot4_battery_callback, 10)

        self.bot1_center_control_pub = self.create_publisher(Int32, '/turtle_1/center_control', 10)
        self.bot2_center_control_pub = self.create_publisher(Int32, '/turtle_2/center_control', 10)
        self.bot3_center_control_pub = self.create_publisher(Int32, '/turtle_3/center_control', 10)
        self.bot4_center_control_pub = self.create_publisher(Int32, '/turtle_4/center_control', 10)

        self.bot1_cmd_pub = self.create_publisher(Twist, '/turtle_1/cmd_vel', 10)
        self.bot2_cmd_pub = self.create_publisher(Twist, '/turtle_2/cmd_vel', 10)
        self.bot3_cmd_pub = self.create_publisher(Twist, '/turtle_3/cmd_vel', 10)
        self.bot4_cmd_pub = self.create_publisher(Twist, '/turtle_4/cmd_vel', 10)

        self.bot1_path_create_sign_pub = self.create_publisher(Int32MultiArray, '/turtle_1/target_goal', 10)
        self.bot2_path_create_sign_pub = self.create_publisher(Int32MultiArray, '/turtle_2/target_goal', 10)
        self.bot3_path_create_sign_pub = self.create_publisher(Int32MultiArray, '/turtle_3/target_goal', 10)
        self.bot4_path_create_sign_pub = self.create_publisher(Int32MultiArray, '/turtle_4/target_goal', 10)


        self.cmd_pub_list = [self.bot1_cmd_pub,self.bot2_cmd_pub,self.bot3_cmd_pub,self.bot4_cmd_pub]
        
        self.bot_odom = [[0.00 for _ in range(3)] for __ in range(4)]

        

        self.bot1_path_len = 0
        self.bot2_path_len = 0
        self.bot3_path_len = 0
        self.bot4_path_len = 0

        self.bot_control_mode_list = [0,0,0,0]
        self.bot1_control_mode = 0
        self.bot2_control_mode = 0
        self.bot3_control_mode = 0
        self.bot4_control_mode = 0

        self.bot_path_list = [[],[],[],[]]

        self.bot_battery_list = [True,True,True,True]

        self.goal_preset_list = [
            [25,49,0.0],
            [44,67,4.7],
            [57,67,4.7],
            [73,49,3.14],
            [57,32,1.6],
            [44,32,1.6]
        ]

        self.heading_set = [0.0,0.05,-0.05,0.0,0.05,-0.05]

        self.goal_side_list = [
            [15,20,0.0],
            [86,20,3.14],
            [86,80,3.14],
            [15,80,0.0]
        ]

        self.priority_contorl = [1,2,3,4]

        self.center_control_state_list = [0,0,0,0]

        self.user_command = 0
        self.center_command = 0

        self.now_heading_list = None

        self.working_done_sign_send = False
        
        self.start_odom_save = False
        self.start_odom = None

        self.check_time = None
        
        self.new_command_time_check = False
        self.new_command_time = None

        self.now_heading_check = False

        self.parking_count_list = [0,0,0,0]
        self.parking_out_complete_time = None
        if(test_mode == -1):
            #real auto
            self.new_command = False
            self.preset = False
            self.goal_list = None
            self.goal_list_save = None
            self.parking_state_list = [0,0,0,0]
        if(test_mode == 0):
            #test parking
            self.goal_list = [1,4,2,5]
            self.new_command = True
            self.preset = True
            self.parking_state_list = [0,0,0,0]
            self.new_command_time = time.time()
            self.goal_list_save = [1,4,2,5]

        if(test_mode == 1):
            self.goal_list = [2,5,1,4]
            self.new_command = True
            self.preset = True
            self.parking_state_list = [2,2,2,2]
            self.new_command_time = time.time()
            self.goal_list_save = [0,4,1,3]
        if(test_mode == 2):
            # test out parking1
            self.goal_list = [0,4,1,3]
            self.new_command = True
            self.preset = True
            self.parking_state_list = [2,2,2,2]
            self.new_command_time = time.time()
            self.goal_list_save = [2,5,1,4]

        self.bot_odom_cb = [False,False,False,False]
        self.server_connection = False
        self.save_before_center_control = False
        self.before_center_control_list = None

        self.stop_done_sign_send = False


        self.user_stop = False

        odom_cb_check = False

        while(not odom_cb_check):
            count = 0
            for i in self.bot_odom_cb:
                if i:
                    count+=1
            if count == 4:
                odom_cb_check = True
            rclpy.spin_once(self)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.user_control)

    def user_control(self):

        if self.user_stop:
            if not self.save_before_center_control:
                self.before_center_control_list  = self.center_control_state_list
                self.save_before_center_control = True
            center_msg = Int32()
            center_msg.data = 1
            self.bot1_center_control_pub.publish(center_msg)
            self.bot2_center_control_pub.publish(center_msg)
            self.bot3_center_control_pub.publish(center_msg)
            self.bot4_center_control_pub.publish(center_msg)
            self.center_control_state_list = [1,1,1,1]
            cmd_stop_msg = Twist()
            cmd_stop_msg.linear.x = 0.0
            cmd_stop_msg.angular.z = 0.0
            for i in range(4):
                self.cmd_pub_list[i].publish(cmd_stop_msg)
        else:
            if self.save_before_center_control:
                self.center_control_state_list = self.before_center_control_list
                self.save_before_center_control = False
            
        if self.server_connection:
            print("preset : ", self.preset)
            print("new_command : ", self.new_command)
            print("도착 : ",self.bot_control_mode_list[0], self.bot_control_mode_list[1], self.bot_control_mode_list[2], self.bot_control_mode_list[3] )
            print("주차 상태 : ", self.parking_state_list)
            print("제어 상태 : ", self.center_control_state_list)
            print("유저 명령 : ", self.user_command)
            print("목표 도착지 : ", self.goal_list)
        if self.new_command and (self.parking_state_list[0] != 2 or self.parking_state_list[1] != 2 or self.parking_state_list[2] != 2 or self.parking_state_list[3] != 2) : # 새로운 명령이 들어왔는데 주차상태가 아닐 경우

            if not self.start_odom_save :
                self.start_odom_save = True
                self.start_odom = copy.deepcopy(self.bot_odom)
            # print(self.start_odom)

            if self.preset: # 만약 프리셋 동작이면

                path_start_msg_list = [Int32MultiArray() for _ in range(4)]

                for i in range(4):
                    path_start_msg_list[i].data.append(int(self.start_odom[i][0]))
                    path_start_msg_list[i].data.append(int(self.start_odom[i][1]))
                    path_start_msg_list[i].data.append(int(self.goal_preset_list[self.goal_list[i]][0]))
                    path_start_msg_list[i].data.append(int(self.goal_preset_list[self.goal_list[i]][1]))

                self.bot1_path_create_sign_pub.publish(path_start_msg_list[0])
                self.bot2_path_create_sign_pub.publish(path_start_msg_list[1])
                self.bot3_path_create_sign_pub.publish(path_start_msg_list[2])
                self.bot4_path_create_sign_pub.publish(path_start_msg_list[3])

                if self.bot_control_mode_list[0] == 3 and self.bot_control_mode_list[1] == 3 and self.bot_control_mode_list[2] == 3 and self.bot_control_mode_list[3] == 3:  # 다 도착했을 경우
                    center_msg = Int32()
                    center_msg.data = 1
                    self.bot1_center_control_pub.publish(center_msg)
                    self.bot2_center_control_pub.publish(center_msg)
                    self.bot3_center_control_pub.publish(center_msg)
                    self.bot4_center_control_pub.publish(center_msg)
                    self.parking_in_control()
                    self.center_control_state_list = [1,1,1,1]

                elif self.bot_control_mode_list[0] == 3 or self.bot_control_mode_list[1] == 3 or self.bot_control_mode_list[2] == 3 or self.bot_control_mode_list[3] == 3: # 하나라도 도착한 경우
                    center_msg = Int32()
                    center_msg.data = 1

                    cmd_right_msg = Twist()
                    cmd_right_msg.linear.x = 0.0
                    cmd_right_msg.angular.z = -0.1
                    cmd_left_msg = Twist()
                    cmd_left_msg.linear.x = 0.0
                    cmd_left_msg.angular.z = 0.1
                    cmd_stop_msg = Twist()
                    cmd_stop_msg.linear.x = 0.0
                    cmd_stop_msg.angular.z = 0.0

                    bot_control_list = []
                    for i in range(4):

                        bot_now_heading = self.bot_odom[i][2]
                        
                        if -0.76 <= bot_now_heading < 0.76 :
                            normal_heading = 0.0
                        elif 0.76 <= bot_now_heading <2.28:
                            normal_heading = 1.6
                        elif 2.28 <= bot_now_heading or bot_now_heading < -2.28:
                            normal_heading = 3.14
                        elif -2.28 <= bot_now_heading < -0.76:
                            normal_heading = 4.7     

                        bot_control_list.append(self.heading_control(normal_heading,self.bot_odom[i][2]))
                    # print(bot_control_list)

                    if self.bot_control_mode_list[0] == 3:
                        self.bot1_center_control_pub.publish(center_msg)
                        self.center_control_state_list[0] = 1
                        if bot_control_list[0] == 1:
                            self.cmd_pub_list[0].publish(cmd_right_msg)
                        elif bot_control_list[0] == 2:
                            self.cmd_pub_list[0].publish(cmd_left_msg)
                        else:
                            self.cmd_pub_list[0].publish(cmd_stop_msg)
                    if self.bot_control_mode_list[1] == 3:
                        self.bot2_center_control_pub.publish(center_msg)
                        self.center_control_state_list[1] = 1
                        if bot_control_list[1] == 1:
                            self.cmd_pub_list[1].publish(cmd_right_msg)
                        elif bot_control_list[1] == 2:
                            self.cmd_pub_list[1].publish(cmd_left_msg)
                        else:
                            self.cmd_pub_list[1].publish(cmd_stop_msg)
                    if self.bot_control_mode_list[2] == 3:
                        self.bot3_center_control_pub.publish(center_msg)
                        self.center_control_state_list[2] = 1
                        if bot_control_list[2] == 1:
                            self.cmd_pub_list[2].publish(cmd_right_msg)
                        elif bot_control_list[2] == 2:
                            self.cmd_pub_list[2].publish(cmd_left_msg)
                        else:
                            self.cmd_pub_list[2].publish(cmd_stop_msg)
                    if self.bot_control_mode_list[3] == 3:
                        self.bot4_center_control_pub.publish(center_msg)
                        self.center_control_state_list[3] = 1
                        if bot_control_list[3] == 1:
                            self.cmd_pub_list[3].publish(cmd_right_msg)
                        elif bot_control_list[3] == 2:
                            self.cmd_pub_list[3].publish(cmd_left_msg)
                        else:
                            self.cmd_pub_list[3].publish(cmd_stop_msg)

                else: # 하나도 도착하지 않은 경우
                    self.center_control_state_list = [0,0,0,0]

                    center_msg = Int32()
                    center_msg.data = 0
                    self.bot1_center_control_pub.publish(center_msg)
                    self.bot2_center_control_pub.publish(center_msg)
                    self.bot3_center_control_pub.publish(center_msg)
                    self.bot4_center_control_pub.publish(center_msg)


                

            else: #프리셋 동작이 아니면

                path_start_msg_list = [Int32MultiArray() for _ in range(4)]
                
                for i in range(4):
                    path_start_msg_list[i].data.append(int(self.start_odom[i][0]))
                    path_start_msg_list[i].data.append(int(self.start_odom[i][1]))
                    path_start_msg_list[i].data.append(int(self.goal_side_list[self.goal_list[i]][0]))
                    path_start_msg_list[i].data.append(int(self.goal_side_list[self.goal_list[i]][1]))

                self.bot1_path_create_sign_pub.publish(path_start_msg_list[0])
                self.bot2_path_create_sign_pub.publish(path_start_msg_list[1])
                self.bot3_path_create_sign_pub.publish(path_start_msg_list[2])
                self.bot4_path_create_sign_pub.publish(path_start_msg_list[3])

                if self.bot_control_mode_list[0] == 3 and self.bot_control_mode_list[1] == 3 and self.bot_control_mode_list[2] == 3 and self.bot_control_mode_list[3] == 3:  # 다 도착했을 경우

                    self.new_command = False  # 새로운 명령이 들어올 때까지 중앙제어 정지
                    self.working_done_sign_send = True
                    self.start_odom_save = False
                elif self.bot_control_mode_list[0] == 3 or self.bot_control_mode_list[1] == 3 or self.bot_control_mode_list[2] == 3 or self.bot_control_mode_list[3] == 3: # 하나라도 도착한 경우
                    center_msg = Int32()
                    center_msg.data = 1

                    cmd_right_msg = Twist()
                    cmd_right_msg.linear.x = 0.0
                    cmd_right_msg.angular.z = -0.1
                    cmd_left_msg = Twist()
                    cmd_left_msg.linear.x = 0.0
                    cmd_left_msg.angular.z = 0.1
                    cmd_stop_msg = Twist()
                    cmd_stop_msg.linear.x = 0.0
                    cmd_stop_msg.angular.z = 0.0

                    bot_control_list = []
                    for i in range(4):

                        bot_now_heading = self.bot_odom[i][2]
                        
                        if -0.76 <= bot_now_heading < 0.76 :
                            normal_heading = 0
                        elif 0.76 <= bot_now_heading <2.28:
                            normal_heading = 1.52
                        elif 2.28 <= bot_now_heading or bot_now_heading < -2.28:
                            normal_heading = 3.14
                        elif -2.28 <= bot_now_heading < -0.76:
                            normal_heading = 4.56     

                        bot_control_list.append(self.heading_control(normal_heading,self.bot_odom[i][2]))
                    print("오류난곳",self.center_control_state_list)
                    if self.bot_control_mode_list[0] == 3:
                        self.bot1_center_control_pub.publish(center_msg)
                        self.center_control_state_list[0] = 1
                        if bot_control_list[0] == 1:
                            self.cmd_pub_list[0].publish(cmd_right_msg)
                        elif bot_control_list[0] == 2:
                            self.cmd_pub_list[0].publish(cmd_left_msg)
                        else:
                            self.cmd_pub_list[0].publish(cmd_stop_msg)
                    if self.bot_control_mode_list[1] == 3:
                        self.bot2_center_control_pub.publish(center_msg)
                        self.center_control_state_list[1] = 1
                        if bot_control_list[1] == 1:
                            self.cmd_pub_list[1].publish(cmd_right_msg)
                        elif bot_control_list[1] == 2:
                            self.cmd_pub_list[1].publish(cmd_left_msg)
                        else:
                            self.cmd_pub_list[1].publish(cmd_stop_msg)
                    if self.bot_control_mode_list[2] == 3:
                        self.bot3_center_control_pub.publish(center_msg)
                        self.center_control_state_list[2] = 1
                        if bot_control_list[2] == 1:
                            self.cmd_pub_list[2].publish(cmd_right_msg)
                        elif bot_control_list[2] == 2:
                            self.cmd_pub_list[2].publish(cmd_left_msg)
                        else:
                            self.cmd_pub_list[2].publish(cmd_stop_msg)
                    if self.bot_control_mode_list[3] == 3:
                        self.bot4_center_control_pub.publish(center_msg)
                        self.center_control_state_list[3] = 1
                        if bot_control_list[3] == 1:
                            self.cmd_pub_list[3].publish(cmd_right_msg)
                        elif bot_control_list[3] == 2:
                            self.cmd_pub_list[3].publish(cmd_left_msg)
                        else:
                            self.cmd_pub_list[3].publish(cmd_stop_msg)

                else: # 하나도 도착하지 않은 경우
                    self.center_control_state_list = [0,0,0,0]
                    center_msg = Int32()
                    center_msg.data = 0
                    self.bot1_center_control_pub.publish(center_msg)
                    self.bot2_center_control_pub.publish(center_msg)
                    self.bot3_center_control_pub.publish(center_msg)
                    self.bot4_center_control_pub.publish(center_msg)

        elif self.new_command and (self.parking_state_list[0] == 2 and self.parking_state_list[1] == 2 and self.parking_state_list[2] == 2 and self.parking_state_list[3] == 2) : # 새로운 명령이 들어왔는데 주차상태일 경우
            center_msg = Int32()
            center_msg.data = 1
            self.bot1_center_control_pub.publish(center_msg)
            self.bot2_center_control_pub.publish(center_msg)
            self.bot3_center_control_pub.publish(center_msg)
            self.bot4_center_control_pub.publish(center_msg)
            self.center_control_state_list = [1,1,1,1]
            self.parking_out_contorl() # 주차 상태 빠져 나오기
            # print("후진중")
        
        elif not self.new_command :
            center_msg = Int32()
            center_msg.data = 1
            self.bot1_center_control_pub.publish(center_msg)
            self.bot2_center_control_pub.publish(center_msg)
            self.bot3_center_control_pub.publish(center_msg)
            self.bot4_center_control_pub.publish(center_msg)
            self.center_control_state_list = [1,1,1,1]
            cmd_stop_msg = Twist()
            cmd_stop_msg.linear.x = 0.0
            cmd_stop_msg.angular.z = 0.0
            for i in range(4):
                self.cmd_pub_list[i].publish(cmd_stop_msg)
            self.bot_path_list = [[],[],[],[]]

            if not self.preset:
                for i in range(4):
                    self.parking_state_list[i] = 0
                    self.bot_control_mode_list[i] = 0
            
            
        
        self.compute_priority()

    def heading_control(self, target, now_heading):
        if now_heading < 0 :
            now_heading_radian = now_heading + math.pi * 2
        else:
            now_heading_radian = now_heading

        if(abs(target - now_heading_radian) > 0.02 and abs(target - now_heading_radian) < 6.26):
            if now_heading_radian - target < 0:
                if now_heading_radian - target < - math.pi:
                    return 1
                else: 
                    return 2
            else:
                if now_heading_radian - target < math.pi:
                    return 1
                else:
                    return 2
        else:
            return 0

    def parking_out_contorl(self):
        
        if(time.time() - self.new_command_time < 7):

            for i in range(4):
                cmd_stop_msg = Twist()
                cmd_stop_msg.linear.x = -0.1
                # cmd_stop_msg.angular.z = 0.0
                cmd_stop_msg.angular.z = self.heading_set[self.goal_list_save[i]]
                self.cmd_pub_list[i].publish(cmd_stop_msg)
            self.parking_out_complete_time = time.time()
            
        else:
            if(time.time() - self.parking_out_complete_time < 3):
                cmd_stop_msg = Twist()
                cmd_stop_msg.linear.x = 0.0
                cmd_stop_msg.angular.z = 0.0
                for i in range(4):
                    self.cmd_pub_list[i].publish(cmd_stop_msg)
                # print("주차 끝@@@@@@@@@@@@@@@@@@@@@@@")
            else:
                for i in range(4):
                    self.parking_state_list[i] = 0
                    self.bot_control_mode_list[i] = 0
                center_msg = Int32()
                center_msg.data = 0
                self.bot1_center_control_pub.publish(center_msg)
                self.bot2_center_control_pub.publish(center_msg)
                self.bot3_center_control_pub.publish(center_msg)
                self.bot4_center_control_pub.publish(center_msg)

                self.center_control_state_list = [0,0,0,0]
                self.start_odom_save = False



    def parking_in_control(self):

        if self.parking_state_list[0] == 0 or self.parking_state_list[1] == 0 or self.parking_state_list[2] == 0 or self.parking_state_list[3] == 0:
            cmd_right_msg = Twist()
            cmd_right_msg.linear.x = 0.0
            cmd_right_msg.angular.z = -0.1
            cmd_left_msg = Twist()
            cmd_left_msg.linear.x = 0.0
            cmd_left_msg.angular.z = 0.1
            cmd_stop_msg = Twist()
            cmd_stop_msg.linear.x = 0.0
            cmd_stop_msg.angular.z = 0.0
            
            bot_control_list = [self.heading_control(self.goal_preset_list[self.goal_list[i]][2],self.bot_odom[i][2]) for i in range(4)]

            for i in range(4):
                if self.parking_state_list[i] == 1:
                    continue
                if bot_control_list[i] == 1:
                    self.cmd_pub_list[i].publish(cmd_right_msg)
                    self.parking_count_list[i] = 0
                elif bot_control_list[i] == 2:
                    self.cmd_pub_list[i].publish(cmd_left_msg)
                    self.parking_count_list[i] = 0
                elif bot_control_list[i] == 0:
                    self.cmd_pub_list[i].publish(cmd_stop_msg)
                    self.parking_count_list[i] += 1
                    # self.parking_state_list[i] = 1

                if self.parking_count_list[i] == 10:
                    self.parking_state_list[i] = 1

            if not (self.parking_state_list[0] == 0 or self.parking_state_list[1] == 0 or self.parking_state_list[2] == 0 or self.parking_state_list[3] == 0):
                time.sleep(1)
                self.check_time = time.time()
                self.parking_count_list = [0,0,0,0]

        
        else:
            if(time.time() - self.check_time < 3):
                cmd_go_msg = Twist()
                cmd_go_msg.linear.x = 0.1
                cmd_go_msg.angular.z = 0.0
                for i in range(4):
                    self.cmd_pub_list[i].publish(cmd_go_msg)
            else:
                cmd_stop_msg = Twist()
                cmd_stop_msg.linear.x = 0.0
                cmd_stop_msg.angular.z = 0.0
                for i in range(4):
                    self.cmd_pub_list[i].publish(cmd_stop_msg)
                    self.parking_state_list[i] = 2
                    self.new_command = False
                    self.working_done_sign_send = True
    
    def compute_priority(self):

        # path_lengths = [
        #     ('bot1', self.bot1_path_len),
        #     ('bot2', self.bot2_path_len),
        #     ('bot3', self.bot3_path_len),
        #     ('bot4', self.bot4_path_len)
        # ]

        # sorted_bots = sorted(path_lengths, key=lambda x: x[1])

        # for i, (bot, _) in enumerate(sorted_bots):
        #     if bot == 'bot1':
        #         self.priority_contorl[0] = i + 1
        #     elif bot == 'bot2':
        #         self.priority_contorl[1] = i + 1
        #     elif bot == 'bot3':
        #         self.priority_contorl[2] = i + 1
        #     elif bot == 'bot4':
        #         self.priority_contorl[3] = i + 1
        # # print("dha")
        
        e_stop_list = self.check_all_collisions(self.bot_path_list,12,4)

        center_control_yes_msg = Int32()
        center_control_yes_msg.data = 2
        center_control_no_msg = Int32()
        center_control_no_msg.data = 0

        if not self.center_control_state_list[0] == 1:
            if 0 in e_stop_list : self.bot1_center_control_pub.publish(center_control_no_msg)
            else: self.bot1_center_control_pub.publish(center_control_yes_msg)
        if not self.center_control_state_list[1] == 1:
            if 1 in e_stop_list: self.bot2_center_control_pub.publish(center_control_no_msg)
            else: self.bot2_center_control_pub.publish(center_control_yes_msg)
        if not self.center_control_state_list[2] == 1:
            if 2 in e_stop_list: self.bot3_center_control_pub.publish(center_control_no_msg)
            else: self.bot3_center_control_pub.publish(center_control_yes_msg)
        if not self.center_control_state_list[3] == 1:
            if 3 in e_stop_list: self.bot4_center_control_pub.publish(center_control_no_msg)
            else: self.bot4_center_control_pub.publish(center_control_yes_msg)
        if self.server_connection:
            print('동작할 터틀봇 :',e_stop_list)

    def check_collision(self,path1, path2, robot_size):
        for i in range(len(path1)):
            for j in range(len(path2)):
                dist = self.distance(path1[i], path2[j])
                if self.distance(path1[i], path2[j]) <= robot_size:
                    # print(path1[i], path2[j],dist)
                    return True
        return False

    def distance(self,point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def check_all_collisions(self,paths, robot_size, num_robots):
        collisions = []
        # print(paths)
        
        for i in range(num_robots):
            for j in range(i + 1, num_robots):
                if self.center_control_state_list[i] == 1 or self.center_control_state_list[j] == 1:
                    continue
                if self.check_collision(paths[i], paths[j], robot_size):
                    collisions.append((i, j))
        #             print("충돌 : ",i,j)
        # print("충돌 리스트:",collisions)
        collision_2d_list = [
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0]
        ]
        min_bot = []

        collision_edge = []
        
        for i in collisions:
            collision_2d_list[i[0]][i[1]] = 1
            collision_2d_list[i[1]][i[0]] = 1
        
        for i in range(num_robots):
            if self.center_control_state_list[i] == 1 or len(paths[i]) == 0: collision_edge.append(5)
            elif collision_2d_list[i].count(1) == 0: collision_edge.append(4)
            else: collision_edge.append(collision_2d_list[i].count(1))

        if collision_edge.count(1)==4:
            go_index = 0
            min_bot.append(go_index)
            
            for i in range(num_robots):
                if i == 0: continue
                if collision_2d_list[go_index][i] == 0:
                    min_bot.append(i)
                    return min_bot
        
        for j in range(4):
            if collision_edge[j] == 5:
                continue
            if collision_edge[j] == 4:
                min_bot.append(j)
                continue
            if min(collision_edge) == collision_edge[j] :
                if collision_2d_list[collision_edge.index(min(collision_edge))][j] == 1:
                    pass
                else:
                    min_bot.append(j)          
        
        return min_bot

    def bot1_odom_callback(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ori = msg.pose.pose.position.z
        self.bot_odom[0] = [x,y,ori]

        self.bot_odom_cb[0] = True

    def bot2_odom_callback(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ori = msg.pose.pose.position.z
        self.bot_odom[1] = [x,y,ori]

        self.bot_odom_cb[1] = True


    def bot3_odom_callback(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ori = msg.pose.pose.position.z
        self.bot_odom[2] = [x,y,ori]

        self.bot_odom_cb[2] = True


    def bot4_odom_callback(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ori = msg.pose.pose.position.z
        self.bot_odom[3] = [x,y,ori]

        self.bot_odom_cb[3] = True


    def bot1_path_callback(self,msg):
        self.bot1_path_len = len(msg.data)//2
        self.bot_path_list[0] = []

        for i in range(self.bot1_path_len):
            self.bot_path_list[0].append((msg.data[2*i],msg.data[2*i+1]))


    def bot2_path_callback(self,msg):
        self.bot2_path_len = len(msg.data)//2
        self.bot_path_list[1] = []

        for i in range(self.bot2_path_len):
            self.bot_path_list[1].append((msg.data[2*i],msg.data[2*i+1]))

    def bot3_path_callback(self,msg):
        self.bot3_path_len = len(msg.data)//2
        self.bot_path_list[2] = []

        for i in range(self.bot3_path_len):
            self.bot_path_list[2].append((msg.data[2*i],msg.data[2*i+1]))

    def bot4_path_callback(self,msg):
        self.bot4_path_len = len(msg.data)//2
        self.bot_path_list[3] = []

        for i in range(self.bot4_path_len):
            self.bot_path_list[3].append((msg.data[2*i],msg.data[2*i+1]))

    def bot1_control_mode_callback(self,msg):
        self.bot_control_mode_list[0] = msg.data

    def bot2_control_mode_callback(self,msg):
        self.bot_control_mode_list[1] = msg.data

    def bot3_control_mode_callback(self,msg):
        self.bot_control_mode_list[2] = msg.data

    def bot4_control_mode_callback(self,msg):
        self.bot_control_mode_list[3] = msg.data

    def bot1_battery_callback(self,msg):
        if msg.percentage < 40:
            self.bot_battery_list[0] = False

    def bot2_battery_callback(self,msg):
        if msg.percentage < 40:
            self.bot_battery_list[1] = False

    def bot3_battery_callback(self,msg):
        if msg.percentage < 40:
            self.bot_battery_list[2] = False

    def bot4_battery_callback(self,msg):
        if msg.percentage < 40:
            self.bot_battery_list[3] = False

    def communication_with_server(self):
        try:
            serverSock = socket(AF_INET, SOCK_STREAM)
            serverSock.bind(('', 8000))
            serverSock.listen(1)
            print("서버 오픈")
            connectionSock, addr = serverSock.accept()
            print(str(addr), '에서 접속이 확인되었습니다.')
            connectionSock.send('okay?'.encode('utf-8'))
            user_command = connectionSock.recv(1024).decode('utf-8')
            print('클라이언트 메시지 :',user_command)
            if user_command == 'ok':
                print('연결 완료')
            self.server_connection = True
            # /map 보내기과정 필요
            
            # 1. 맵크기 전송
            # 2. 맵 전송

            msg = ""
            mapSize = ""
            print(PATH + 'final_map.txt')
            f = open(PATH + 'final_map.txt','r')
            lines = f.readlines()
            
            mapSize = str(len(lines))
            mapSize += " "
            mapSize += str(len(lines[0].strip()))

            map_arr = ""
            for line in lines:
                if len(line) > 0 :
                    map_arr += line.strip()
            # print(len(map_arr) , mapSize)

            connectionSock.send(mapSize.encode('utf-8'))
            afterSendSize = connectionSock.recv(1024).decode('utf-8')
            # print(afterSendSize)
            connectionSock.send(map_arr.encode('utf-8'))
            afterSendMap = connectionSock.recv(1024).decode('utf-8')      
            
            msg = ""
            for i in range(len(self.goal_preset_list)):
                for j in range(len(self.goal_preset_list[0])):
                    msg += (" " + str(self.goal_preset_list[i][j]))
            
            # print(msg)

            connectionSock.send(msg.encode('utf-8'))
            afterGoalPos = connectionSock.recv(1024).decode('utf-8')

            #코너좌표 보내기
            msg = ""
            for i in range(len(self.goal_side_list)):
                for j in range(len(self.goal_side_list[0])):
                    msg += (" " + str(self.goal_side_list[i][j]))

            # print(msg)

            connectionSock.send(msg.encode('utf-8'))
            afterSidePos = connectionSock.recv(1024).decode('utf-8')

            while(1):
                if self.new_command == False:
                    if self.working_done_sign_send or self.user_command == 6:
                        self.center_command = 8
                        self.working_done_sign_send = False
                        self.new_command_time_check = False
                    else:
                        self.center_command = 0
                else:
                    self.center_command = 7

                if self.user_stop:
                    if self.user_command == 5:
                        if not self.stop_done_sign_send :
                            self.center_command = 8
                            self.stop_done_sign_send = True
                    else:
                        self.user_stop = False
                        self.stop_done_sign_send = False

                bot1_odom = [str(i) for i in self.bot_odom[0]]
                bot2_odom = [str(i) for i in self.bot_odom[1]]
                bot3_odom = [str(i) for i in self.bot_odom[2]]
                bot4_odom = [str(i) for i in self.bot_odom[3]]
                msg = [str(self.center_command)]+bot1_odom+bot2_odom+bot3_odom+bot4_odom
                # print(msg)
                msg = ' '.join(msg)
                connectionSock.send(msg.encode('utf-8'))
                client_msg = connectionSock.recv(1024).decode('utf-8')
                client_command_list = list(map(int, client_msg.split()))
                self.user_command = client_command_list[0]

                # print("client msg :", client_command_list)

                if self.user_command == 1 or self.user_command == 2:
                    self.goal_list_save = self.goal_list
                    self.goal_list = client_command_list[1:]
                    if not self.new_command_time_check and (self.parking_state_list[0] == 2 and self.parking_state_list[1] == 2 and self.parking_state_list[2] == 2 and self.parking_state_list[3] == 2):
                        self.new_command_time = time.time()
                        self.new_command_time_check = True
                    self.new_command = True

                    if self.user_command == 1:
                        self.preset = True
                    else:
                        self.preset = False

                elif self.user_command == 5: # 전체 정지
                    self.user_stop = True
                    self.new_command = False
                    self.parking_state_list = [0,0,0,0]
                    self.bot_control_mode_list = [0,0,0,0]
                    
                    pass
                elif self.user_command == 6: # 작동중
                    pass 
                
        except Exception as e:
            print('이미지를 전송하는 중 오류가 발생했습니다:', e)
        finally:
            serverSock.close()
            connectionSock.close()

def main(args=None):
    rclpy.init(args=args)
    center_node = Center_node()

    communication_thread= threading.Thread(target=center_node.communication_with_server,args=())
    communication_thread.start()

    rclpy.spin(center_node)
    center_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    
        