# define all config here
# use c_ means coarse_search params
# other means no coarse_search params 
import math


class Carmodel:
    #params of  Porsche panamera 4
    front_hang = 1.149
    wheel_base = 2.950
    rear_hang = 0.950
    width = 1.937

    #vehicle medel physical limit
    max_velocity = 3
    min_velocity = -3
    
    max_acceleration = 1.0
    min_acceleration = -3

    max_phi = 0.7
    max_omega = 0.2
    
    #use how many circle to cover car footprint
    num_disc = 2
    disc_radius = 0
    disc_coefficients = []

    def InitializeDisc(self):
        length = self.wheel_base + self.front_hang + self.rear_hang
        self.disc_radius = 0.5 * math.hypot(length / self.num_disc, self.width)

        for i in self.num_disc:
            self.disc_coefficients.append((2 * (i + 1 - 1) / (2 * self.num_disc) * length - self.rear_hang))


    def GetDiscPosision(self, x, y, theta):
        result = []
        for i in self.num_disc:
            result.append(x + self.disc_coefficients[i] * math.cos(theta))
            result.append(y + self.disc_coefficients[i] * math.sin(theta))
        return result
    
    def calculate_corner_points(rear_x, rear_y, length, width, theta):
        corner_points_relative = [
            (-length / 2, width / 2),  # 左前
            (-length / 2, -width / 2),  # 右前
            (length / 2, width / 2),   # 左后
            (length / 2, -width / 2)   # 右后
        ]

        corner_points = []
        for x, y in corner_points_relative:
            # 旋转变换
            x_rotated = x * math.cos(theta) - y * math.sin(theta)
            y_rotated = x * math.sin(theta) + y * math.cos(theta)
            # 平移变换
            x_final = x_rotated + rear_x
            y_final = y_rotated + rear_y
            corner_points.append((x_final, y_final))

        return corner_points
        






class Config :

    #search params
    #astar resolution
    c_res = 0.5
    #search max iterator
    c_max_iter = 1e9
    #diagonal border min_x mix_y max_x max_y
    min_x = -100
    max_x = 100
    min_y = -100
    max_y = 100



    #liom nlp params
    max_kine_infeasibble = 1e-5
    max_solve_iter = 10
    max_solve_time = 3
    forward_panelty = 0.5
    backward_panelty = 1.0
    gear_change_penalty = 5.0
    steering_penalty = 0.5
    steer_change_penalty = 1.0

    #minest finiti element 
    min_nfe = 20

    #time step
    time_step = 0.4

    #corridor relative params
    corridor_max_iter = 1000

    corridor_incremental_time = 20.0

    #optimal params
    opti_w_a = 1
    opti_w_omega = 1.0
    opti_w_inner_iter_max = 100
    opti_w_penalty0 = 1e4

    #vehicle limit
    max_acceleration = 3
    max_velocity = 3
    max_reverse_acceleration = -3


    paramera = Carmodel()
    
    
    
    




