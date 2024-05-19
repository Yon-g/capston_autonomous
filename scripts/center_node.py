#!/usr/bin/env python3
import math
from socket import *
import threading

import rclpy
from rclpy.node import Node

from cap_msg.msg import Odom
# from cap_msg.msg import Obstacle
# from cap_msg.msg import Map

PATH = '/home/yongki/dev_ws/src/topic_example_pkg/topic_example_pkg'

class Center_node(Node):

    def __init__(self):
        super().__init__('Center_node')
        self.bot1_odom_sub = self.create_subscription(Odom, '/bot1/ab_odom', self.bot1_odom_callback, 10)
        self.bot2_odom_sub = self.create_subscription(Odom, '/bot2/ab_odom', self.bot2_odom_callback, 10)
        self.bot3_odom_sub = self.create_subscription(Odom, '/bot3/ab_odom', self.bot3_odom_callback, 10)
        self.bot4_odom_sub = self.create_subscription(Odom, '/bot4/ab_odom', self.bot4_odom_callback, 10)

        # self.bot1_obstacle_sub = self.create_subscription(Obstacle, '/bot1/obstacle', self.bot4_odom_callback, 10)
        # self.bot2_obstacle_sub = self.create_subscription(Obstacle, '/bot2/obstacle', self.bot4_odom_callback, 10)
        # self.bot3_obstacle_sub = self.create_subscription(Obstacle, '/bot3/obstacle', self.bot4_odom_callback, 10)
        # self.bot4_obstacle_sub = self.create_subscription(Obstacle, '/bot4/obstacle', self.bot4_odom_callback, 10)


        self.bot1_destination_pub = self.create_publisher(Odom, '/bot1/destination', 10)
        self.bot2_destination_pub = self.create_publisher(Odom, '/bot2/destination', 10)
        self.bot3_destination_pub = self.create_publisher(Odom, '/bot3/destination', 10)
        self.bot4_destination_pub = self.create_publisher(Odom, '/bot4/destination', 10)

        # self.map_pub = self.create_publisher(Map, '/map', 10)

        self.bot1_odom = [0.00 for _ in range(3)]
        self.bot2_odom = [0.00 for _ in range(3)]
        self.bot3_odom = [0.00 for _ in range(3)]
        self.bot4_odom = [0.00 for _ in range(3)]

        self.user_command = 0
        self.center_command = 0
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.user_control)
        self.timer = self.create_timer(timer_period, self.pub_map)

    def user_control(self):
        if self.user_command == 0:
            pass
        elif self.user_command == 1:
            print("mode1")
            self.pub_destination()
        elif self.user_command == 2:
            print("mode2")
            self.pub_destination()
        elif self.user_command == 3:
            print("mode3")
            self.pub_destination()
        elif self.user_command == 4:
            print("터틀봇 정지")
            self.pub_destination()
    
    def pub_destination(self): # 생각할 것. 도착지 프리셋을 사용자가 바꿨을 때 그 좌표를 내가 어떻게 알 것인가.
        print("도착지 발행")

    def pub_map(self):
        print("맵 발행1")

    def bot1_odom_callback(self,msg):
        x = math.floor(msg.pose.pose.position.x * 100)/100
        y = math.floor(msg.pose.pose.position.y * 100)/100
        ori = math.floor(msg.pose.pose.orientation.z * 100)/100
        self.bot1_odom = [x,y,ori]

    def bot2_odom_callback(self,msg):
        x = math.floor(msg.pose.pose.position.x * 100)/100
        y = math.floor(msg.pose.pose.position.y * 100)/100
        ori = math.floor(msg.pose.pose.orientation.z * 100)/100
        self.bot2_odom = [x,y,ori]

    def bot3_odom_callback(self,msg):
        x = math.floor(msg.pose.pose.position.x * 100)/100
        y = math.floor(msg.pose.pose.position.y * 100)/100
        ori = math.floor(msg.pose.pose.orientation.z * 100)/100
        self.bot3_odom = [x,y,ori]

    def bot4_odom_callback(self,msg):
        x = math.floor(msg.pose.pose.position.x * 100)/100
        y = math.floor(msg.pose.pose.position.y * 100)/100
        ori = math.floor(msg.pose.pose.orientation.z * 100)/100
        self.bot4_odom = [x,y,ori]

    def communication_with_server(self):
        try:
            print("서버 연결 시도중")
            serverSock = socket(AF_INET, SOCK_STREAM)
            serverSock.bind(('', 8000))
            serverSock.listen(1)
            connectionSock, addr = serverSock.accept()
            print(str(addr), '에서 접속이 확인되었습니다.')
            while(1):
                bot1_odom = [str(i) for i in self.bot1_odom]
                bot2_odom = [str(i) for i in self.bot2_odom]
                bot3_odom = [str(i) for i in self.bot3_odom]
                bot4_odom = [str(i) for i in self.bot4_odom]
                msg = [str(self.center_command)]+bot1_odom+bot2_odom+bot3_odom+bot4_odom
                msg = ' '.join(msg)
                connectionSock.send(msg.encode('utf-8'))
                self.user_command = int(connectionSock.recv(1024).decode('utf-8'))
                print('클라이언트 메시지 :',self.user_command)
                #받고나서 동작 확인 되면 보내기
                #작업 시작 시 7
                #작업 종료 시 8
                #작업 중 4번 빼고 다 걸러내기
                #작업 시작 7번 보내고 계속 0 들어옴
                
                #커넥션 확인
                #커넥션 받고
                #이미지 크기 보내기
                #케넥션 확인
                #이미지 보내기
                #커녁센 확인
                #while문 시작
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

    
        