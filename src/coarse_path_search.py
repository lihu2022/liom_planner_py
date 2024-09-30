import matplotlib.pyplot as plot
import time
from math import inf as INF

from config import Config
from environmental import Environment
import heapq
import math

config = Config()

class Node():
    def __init__(self, p_node):
        self.parent_node = p_node
        self.f = INF
        self.g = INF
        self.h = INF
        self.is_in_closedlist = False
        self.is_in_openlist = False


# class Env():
#     def __init__(self, l_bd_pt, r_bd_pt, res):
#         self.l_bd_pt_ = l_bd_pt
#         self.r_bd_pt_ = r_bd_pt
#         self.res = res


#     def pos2grid(pos):
#         delta_x = pos.x - config.min_x
#         delta_y = pos.y - config.min_y
#         grid_index_x = delta_x / config.c_res + 1
#         grid_index_y = delta_y / config.c_res + 1

#         return (grid_index_x, grid_index_y)



# env = Env()
     


class Astar():

    def __init__(self, start, end, env):
        self.start_ = start
        self.end_ = end
        self.env_: Environment = env



    def heuristic(self, a, b):
        """
        计算两个坐标点之间的曼哈顿距离。

        参数：
            a: 第一个坐标点 (row, col)。
            b: 第二个坐标点 (row, col)。

        返回值：
            两个坐标点之间的欧式距离。
        """
        # return abs(a[0] - b[0]) + (a[1] - b[1])
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def visualize_map(self, grid, path=None):
        """可视化地图和路径"""
        plot.clf()  # 清除之前的图像
        plot.imshow(grid, cmap='gray')  # 显示地图，0 为白色，1 为黑色
        plot.title('A* Path Finding')

        if path:
            x = [point[1] for point in path]  # 注意：这里 x 对应列索引
            y = [point[0] for point in path]  # 注意：这里 y 对应行索引
            plot.plot(x, y, marker='.', linestyle='-', color='red')

        plot.pause(0.001) 

    

    def Plan(self):
        """
        在 0/1 代价地图上搜索从起点到终点的路径。

        参数：
            grid: 一个二维列表，表示地图，其中 0 表示可通行，1 表示障碍物。
            start: 一个元组，表示起点的坐标 (row, col)。
            goal: 一个元组，表示终点的坐标 (row, col)。

        返回值：
            如果找到路径，则返回一个列表，表示路径上的坐标点；否则返回 None。
        """
        rows = len(self.env_.dilated_cost_map)
        cols = len(self.env_.dilated_cost_map[0])

        # 定义移动方向（上下左右）
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

        # 初始化优先队列，存储待探索的节点 (cost, distance, node)
        queue = [(0, 0, self.start_)]
        # 使用字典记录每个节点的父节点，用于回溯路径
        came_from = {}
        # 记录每个节点的代价
        cost_so_far = {self.start_: 0}

        while queue:
            # 从优先队列中取出代价最小的节点
            _, distance, current = heapq.heappop(queue)
            
            plot.title(f"Expanding Node: {current}")
            print(current)
            # now_path = list(came_from.keys()) + [current]
            # self.visualize_map(self.env_.dilated_cost_map, path=now_path)  # 更新路径
            # plot.draw()
            # plot.pause(0.001)  # 暂停一段时间，以便观察

            # 如果到达终点，则回溯路径
            if current == self.end_:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start_)
                return path[::-1]  # 反转路径

            # 探索当前节点的邻居节点
            for dr, dc in directions:
                neighbor = (current[0] + dr, current[1] + dc)
                if (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and
                    self.env_.dilated_cost_map[neighbor[0]][neighbor[1]] == 0):
                    # 计算新的代价
                    new_cost = cost_so_far[current] + 1  # 在 0/1 代价地图中，每个移动的代价为 1
                # 如果邻居节点未被访问过，或者新的代价更小
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        # 更新代价和父节点
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + distance +  5 * self.heuristic(self.end_, neighbor)  # 使用曼哈顿距离作为启发函数
                        heapq.heappush(queue, (priority, distance + 1, neighbor))
                        came_from[neighbor] = current
                # else:
                    # print(neighbor[0], neighbor[1])
                    # if (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                    #     print(self.env_.dilated_cost_map[neighbor[0]][neighbor[1]])

        # 如果搜索完所有节点仍未找到路径，则返回 None
        return None
    