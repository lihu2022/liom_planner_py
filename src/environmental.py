from shapely import Polygon
from config import Config
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation




class Environment :
    

    #initial env params
    def __init__(self, config):
        polygon1 = Polygon([(0, 0), (0, 4), (-2, 0)])
        polygon2 = Polygon([(52, -50), (50, -50), (53, -53)])
        self.config_:Config = config
        self.polygons_ = [polygon1, polygon2]
        


    #create car planning environment
    def create_env(self):
        range_x = self.config_.max_x - self.config_.min_x
        range_y = self.config_.max_y - self.config_.min_y
        index_num_x = int(range_x / self.config_.c_res) + 1
        index_num_y = int(range_y / self.config_.c_res) + 1
        self.cost_map = np.zeros((index_num_x, index_num_y))

        for poly in self.polygons_:
            coords = poly.exterior.coords
            x,y = coords.xy
            p_x, p_y = zip(*coords)
            min_index_x = math.floor((min(x) - self.config_.min_x) / self.config_.c_res + 1)
            max_index_x = math.ceil((max(x) - self.config_.min_x) / self.config_.c_res + 1)
            min_index_y = math.floor((min(y) - self.config_.min_y) / self.config_.c_res + 1)
            max_index_y = math.ceil((max(y) - self.config_.min_y) / self.config_.c_res + 1)
            # print(min(x), max(x), min(y), max(y))
            plt.plot(p_x, p_y, '-')
            plt.xlim(self.config_.min_x, self.config_.max_x)
            plt.ylim(self.config_.min_y, self.config_.max_y)
            plt.title("polygon position")
            plt.show()
            for i in range(min_index_x, max_index_x + 1):
                for j in range(min_index_y, max_index_y + 1):
                    self.cost_map[i, j] = 1
                    # print(i, j)

    def index2xy(self, index):
        x = (index[0] - 1) * self.config_.c_res + self.config_.min_x;
        y = (index[1] - 1) * self.config_.c_res + self.config_.min_y;
        return (x, y)
    
    def xy2index(self, xy) :
        index_x = int((xy[0] - self.config_.min_x) / self.config_.c_res + 1)
        index_y = int((xy[1] - self.config_.min_y) / self.config_.c_res + 1)
        return (index_x, index_y)
    
    def fromindextoxys(self, path):
        result = []
        length = len(path)
        for i in range(length):
            pos_now_xy = self.index2xy(path[i])
            if (i == 0):
                now_theta = math.pi / 2
            elif (i == length - 1):
                now_theta = 0
            else:
                pos_next_xy = self.index2xy(path[i + 1])
                now_theta = math.atan2(pos_next_xy[1] - pos_now_xy[1], pos_next_xy[0] - pos_now_xy[0])
            result.append((pos_now_xy, now_theta))

        return result
            





        

    
    def visualize_costmap(self):
        plt.figure()
        matrix = self.cost_map
        plt.imshow(matrix, cmap='binary')
        plt.title("cost map show")
        plt.colorbar()
        plt.show()

    def dilated_polygons(self):
        kernel = np.ones((4, 4), dtype=int)
        dilated_cost_map = binary_dilation(self.cost_map, structure=kernel).astype(int)
        self.dilated_cost_map = dilated_cost_map

    def visualize_dilated_costmap(self):
        plt.figure()
        matrix = self.dilated_cost_map
        plt.imshow(matrix, cmap='binary')
        plt.title("dilated cost map show")
        plt.colorbar()
        plt.show()

                    
                    
        




    # def CheckPoseCollision(self, time, pose):
    #     discs = self.config_.panamera.GetdiscPosision(pose.x(), pose.y(),pose.theta())
    #     wh = self.config_



    # def CheckBoxCillision(time, box):





    # def GenerateCorridoeBox(time, x, y, radius):



    


