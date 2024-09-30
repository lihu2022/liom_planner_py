import numpy as np
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.finder.dijkstra import DijkstraFinder
from pathfinding.core.diagonal_movement import DiagonalMovement

from environmental import Environment



class PathFinding:
    def __init__(self,start, goal, env):
        self.env : Environment = env
        self.start = start
        self.goal = goal




    def Plan(self):
        grid = Grid(matrix=self.env.dilated_cost_map)
        # grid = Grid(matrix=np.zeros((401, 401)))
        for x in range(self.env.dilated_cost_map.shape[0]):
            for y in range(self.env.dilated_cost_map.shape[1]):
                if (self.env.dilated_cost_map[x][y]):
                    grid.node(x, y).walkable = False
                else:
                    grid.node(x, y).walkable = True
        s = grid.node(*self.start)
        g = grid.node(*self.goal)
        
        # finder = DijkstraFinder()
        # print("Grid width:", grid.width)
        # print("Grid height:", grid.height)
        # # print("Grid nodes:", grid.nodes)
        # print("Grid nodes type:", type(grid.nodes))
        # print("Grid nodes[0] length:", len(grid.nodes[0]))

        # test_map = np.zeros((20, 20)) 

        # grid = Grid(matrix=test_map)

        # for y in range(grid.height):
        #     for x in range(grid.width):
        #         grid.node(x, y).walkable = True
                
    
        #  确保起点和终点设置正确，例如：
        # self.start = (10, 10)  
        # self.goal = (1, 1)  

        s = grid.node(*self.start)
        g = grid.node(*self.goal)

        s.walkable = True
        g.walkable = True
    
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        
        print("Grid width:", grid.width)
        print("Grid height:", grid.height)

        
        path, runs = finder.find_path(s, g, grid)
        print("Start node:", s.x, s.y)
        print("Goal node:", g.x, g.y)
        print("Start node walkable:", s.walkable)
        print("Goal node walkable:", g.walkable)
        print(path)

        return path

