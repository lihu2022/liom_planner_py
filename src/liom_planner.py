import math
import environmental as Env
import numpy as np





class LiomPlanner():

    def __init__(self, oripath, env : Env) -> None:
        self.oripath_ = oripath
        self.env_ = env
    
    def ResamplePath(self, path):
        length = len(path)
        gears = [None] * length
        stations = [None] * length
        for i in range(length):
            track_angle = math.atan2(path[i][2] - path[i - 1][1], path[i][0] - path[i][0])
            gear = abs(self.NormalizeAngle(track_angle - path[i][2])) < (math.pi / 2)
            gears[i] = 1 if gear else -1
            stations[i] = stations[i - 1] + self.Disdance(path[i], path[i - 1])
        
        if (len(gears) > 1):
            gears[0] = gears[1]


        time_profile = [None] * length
        last_idx = 0
        start_time = 0
        for i in range(len(gears)):
            if i == len(gears) - 1 or gears[i + 1] != gears[i]:
                station_segment = stations[last_idx:i + 1]  # 获取一个 station_segment 子列表
                # 生成最优时间分布
                profile = self.generate_optimal_time_profile_segment(station_segment, start_time)
                # 更新 time_profile 列表
                time_profile[last_idx:i + 1] = profile
                # 更新 start_time 和 last_idx
                start_time = profile[-1]
                last_idx = i

        nfe = max(self.env_.Config.min_nfe, time_profile[-1] / self.env_.time_step)
        interpolated_ticks = np.linspace(time_profile[0], time_profile[-1], nfe)
        
        

            


    def Disdance(p1, p2):
        return math.sqrt((p1[1] - p2[1])**2, (p1[0] - p2[0]))
    
    def generate_optimal_time_profile_segment(self, stations, start_time):
        # 初始化变量
        max_decel = -self.env_.Config.max_acceleration
        min_velocity = -self.env_.Config.max_velocity

        # 加速相
        accel_idx = 0
        vi = 0.0
        profile = [0.0 for _ in stations]
        for i in range(len(stations) - 1):
            ds = stations[i+1] - stations[i]

            profile[i] = vi
            vi = math.sqrt(vi**2 + 2 * self.env_.Config.max_acceleration * ds)
            vi = min(self.env_.Config.max_velocity, max(min_velocity, vi))

            if vi >= self.env_.Config.max_velocity:
                accel_idx = i + 1
                break

        # 减速相
        vi = 0.0
        decel_idx = len(stations) - 1
        for i in range(len(stations) - 1, accel_idx, -1):
            ds = stations[i] - stations[i - 1]

            profile[i] = vi
            vi = math.sqrt(vi**2 - 2 * max_decel * ds)
            vi = min(self.env_.Config.max_velocity, max(min_velocity, vi))

            if vi >= self.env_.Config.max_velocity:
                decel_idx = i
                break

        # 填充色剖面
        for i in range(accel_idx, decel_idx):
            profile[i] = self.env_.Config.max_velocity

        # 生成时间剖面
        time_profile = [start_time for _ in stations]
        for i in range(1, len(stations)):
            if profile[i] < 1e-6:
                time_profile[i] = time_profile[i - 1]
                continue
            time_profile[i] = time_profile[i - 1] + (stations[i] - stations[i - 1]) / profile[i]

        return time_profile






    def NormalizeAngle(angle):
    # 对角度进行正规化调整到[0, 2*pi)
        a = math.fmod(angle + math.pi, 2.0 * math.pi)
    # 如果计算结果为负值，调整到[0, 2*pi)的范围
        if a < 0.0:
            a += (2.0 * math.pi)
    # 再次调整，使其落在[-pi, pi)的范围
        return a - math.pi

        
        

    def GetGuessFromOripath(self):
        
        return self.ResamplePath(self.oripath_)
    

    def Plan(self):

        pass

    def ConvertSolution2Trajectory(self): 
        pass

    def CaculateKinematicInfeasibility(self):
        pass

    use_iterator = True

