"""
A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

import math
import random
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import os
import csv

from itertools import permutations

show_animation = False


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        print(sx,sy,gx,gy)

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node
        

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

def isSafe(x1,y1,x2,y2,radius):
    square_distance = ((x1 - x2)**2 + (y1 - y2)**2)
    if square_distance <= (2*radius)**2 : return False
    else : return True

def SLAM_TO_Array(source = "relative path of map image file"):
    absolute_path = os.path.dirname(os.path.realpath(__file__))
    img_path = absolute_path + "/" + source
    mapImg = Image.open(img_path)
    mapImg.show()
    mapArray = np.array(mapImg)
    mapArrayStr = []

    for i in range(mapArray.shape[0]):
        line = []
        for j in range(mapArray.shape[1]):
            line.append(str(mapArray[i][j]))
        mapArrayStr.append(line)

    mapInt = [[0 for _ in range(len(mapArrayStr[0]))]for _ in range(len(mapArrayStr))]

    for i in range(len(mapArrayStr)):
        for j in range(len(mapArrayStr[0])):
            if mapArrayStr[i][j] != '254' : mapInt[i][j] = 0
            else : mapInt[i][j] = 1

    #mapInt 배열에서 1은 갈 수 있는 곳, 0은 갈 수 없는 곳

    # csv파일로 저장
    with open(absolute_path+"/"+'MAP2arr.csv',"w") as file:
        writer = csv.writer(file)
        writer.writerows(mapInt)

    return mapInt

def run():
# def run(STARTS:list,GOALS:list)->list:
    print(__file__ + " start!!")
    mapInt = SLAM_TO_Array("static/1.pgm")

    #책상그리기?
    # mx,my = len(mapInt[0])//2,len(mapInt)//2
    # for i in range(mx-5,mx+6):
    #     for j in range(my-5,my+6):
    #         mapInt[j][i] = 0
 
    # start and goal position
    # sx = 30.0  # [m]
    # sy = 30.0  # [m]
    # gx = 150.0  # [m]
    # gy = 150.0  # [m]

    sx = []
    sy = []
    gx = []
    gy = []
    # gx = [100.0, 100.0, 50.0, 150.0]
    # gy = [50.0, 150.0, 100.0, 100.0]

    start = []
    while len(start) < 4:
        ty,tx = random.randint(0,len(mapInt[0])-1), random.randint(0,len(mapInt)-1)
        if mapInt[tx][ty] == 0 : continue
        if (tx,ty) in start : continue
        start.append((tx,ty))

    for x,y in start :
        sx.append(x)
        sy.append(y)

    goal = []
    while len(goal) < 4:
        ty,tx = random.randint(0,len(mapInt[0])-1), random.randint(0,len(mapInt)-1)
        if mapInt[tx][ty] == 0 : continue
        if (tx,ty) in start or (tx,ty) in goal : continue
        goal.append((tx,ty))
    
    for x,y in goal :
        gx.append(x)
        gy.append(y)


    grid_size = 3.0  # [m]
    robot_radius = 1.0  # [m]

    CrashWeight = 5.0

    num_robot = 4

    # set obstacle positions
    ox, oy = [], []
    # for i in range(0,201):
    #     ox.append(i)
    #     oy.append(0)

    # for i in range(0,201):
    #     ox.append(0)
    #     oy.append(i)

    # for i in range(0,201):
    #     ox.append(200)
    #     oy.append(i)

    # for i in range(0,201):
    #     ox.append(i)
    #     oy.append(200)

    # for i in range(70,131):
    #     ox.append(i)
    #     oy.append(70)

    # for i in range(70,131):
    #     ox.append(70)
    #     oy.append(i)

    # for i in range(70,131):
    #     ox.append(i)
    #     oy.append(130)

    # for i in range(70,131):
    #     ox.append(130)
    #     oy.append(i)

    for i in range(len(mapInt)):
        for j in range(len(mapInt[0])):
            if mapInt[i][j] == 0:
                ox.append(j)
                oy.append(len(mapInt) - i - 1)

    markers_s = ["og","or","oy","ob"]
    markers_g = ["xg","xr","xy","xb"]
    
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)

    paths = []
    pmts = list(permutations([0,1,2,3],4))

    for p in pmts:
        path = []
        for i in range(num_robot):
            path.append(list(a_star.planning(sx[i], sy[i], gx[p[i]], gy[p[i]])))

        paths.append(path)

    print(len(paths))
    for branch in range(len(paths)):
        # path[branch][1][0] # n번째 순열 -> 0번째 도착-시작 -> 0 / x좌표
        dataOfbranch = []
        lengthOfEach = []
        crushSpot = []

        for i in range(num_robot):
            lengthOfEach.append(len(paths[branch][i][0]))

        for i in range(num_robot):
            for j in range(i+1,num_robot):
                if len(paths[branch][i][0]) >= len(paths[branch][j][0]):
                    shorter = j
                    longer = i
                else :
                    shorter = i
                    longer = j

                for d in range(len(paths[branch][shorter][0])):
                    if not isSafe(paths[branch][i][0][d],paths[branch][i][1][d],paths[branch][j][0][d],paths[branch][j][1][d],robot_radius):
                        crushSpot.append([(shorter,longer), (paths[branch][shorter][0][d],paths[branch][shorter][1][d])])
                        break
        
        dataOfbranch = [sum(lengthOfEach),lengthOfEach,crushSpot]
        paths[branch].append(dataOfbranch)
    
    best = 0
    best_score = float('INF')
    for i in range(len(paths)):
        tmpData = paths[i][num_robot]
        score = tmpData[0] + max(tmpData[1])
        eachPathCrash = [0,0,0,0]

        for crush in tmpData[2]:
            eachPathCrash[crush[0][0]] += 1
            eachPathCrash[crush[0][1]] += 1

        for el in eachPathCrash:
            score += (1.0 + (0.1) * el) * CrashWeight

        print(i,"번째",tmpData[0],max(tmpData[1]),eachPathCrash,"점수 :",score)
        if score < best_score :    
            best = i
            best_score = score
            
    dataOfBest = paths[best][num_robot]
    print(dataOfBest)

    colors = ["-r","-g","-b","-y"]
    if True:  # pragma: no cover
        for i in range(num_robot):
            plt.plot(ox, oy, ".k")
            plt.plot(sx[i], sy[i], markers_s[i])
            plt.plot(gx[i], gy[i], markers_g[i])
            plt.grid(True)
            plt.axis("equal")

    if True:  # pragma: no cover
        for i in range(num_robot):
            plt.plot(paths[best][i][0], paths[best][i][1], colors[i])
            plt.pause(0.0005)
        plt.show()
    
    selected_path = []

    for i in range(num_robot):
        pathOfEachBot = []
        for n in range(len(paths[best][i][0])):
            xPos, yPos = paths[best][i][0][n],paths[best][i][1][n]
            pathOfEachBot.append([xPos,yPos])
        selected_path.append(reversed(pathOfEachBot))

    return selected_path

# if __name__ == '__main__':
run()