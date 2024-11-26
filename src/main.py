from coarse_path_search import Astar
from liom_planner import LiomPlanner
from config import Config
from environmental import Environment
import matplotlib.pyplot as plt
from path_finding_lib import PathFinding
import math 



def main():
    start = (75, -75)
    end = (-75, 75)
    start_index = env.xy2index(start)
    end_index = env.xy2index(end)
    use_path_finding_lib = False
    if (use_path_finding_lib):
        astar = PathFinding(start_index,end_index, env)
    else:
        astar = Astar(start_index, end_index, env)
    path = astar.Plan()
    if (path != None):
        print("success to search a path")
        env.visualize_costmap()
        if (use_path_finding_lib):
            x = [node.x for node in path]
            y = [node.y for node in path]
        else:
            x = [point[0] for point in path]
            y = [point[1] for point in path]
        ax = plt.gca()
        ax.plot(y, x, marker='.', linestyle='-', color= 'blue', linewidth=1)
        plt.draw()
        plt.pause(0.1)
    else:
        print("search failed!!!!!")

    path_xytheta = env.fromindextoxys(path)
    plot_path(path_xytheta)


    liom_planner = LiomPlanner(path_xytheta, env)
    if (liom_planner.Plan()):
        print("opmization success!!")
    else:
        print("optimization failed!!!!!")



def plot_path(path_xytheta):
    """
    使用 matplotlib 绘制包含 (x, y, theta) 信息的路径图。

    参数:
        path_xytheta: 路径点列表，每个元素为 (x, y, theta)，theta 是弧度制
    """
    x = [p[0][0] for p in path_xytheta]  # 提取 x 坐标
    y = [p[0][1] for p in path_xytheta]  # 提取 y 坐标
    u = [math.cos(p[1]) for p in path_xytheta]  # 计算箭头 x 方向分量
    v = [math.sin(p[1]) for p in path_xytheta]  # 计算箭头 y 方向分量

    # 创建图形
    plt.figure()
    plt.quiver(x, y, u, v, angles='xy', scale_units='xy', scale=1) # 绘制箭头
    plt.plot(x, y, 'b-', linewidth=2)  # 绘制路径线
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Path with Orientation")
    plt.grid(True)
    plt.axis('equal')  # 保持 x 和 y 轴比例一致
    plt.show()




if __name__ == "__main__":
    plt.ion()
    config = Config()
    env = Environment(config)
    env.create_env()
    env.visualize_costmap()
    env.dilated_polygons()
    env.visualize_dilated_costmap()
    main()
