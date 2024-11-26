import casadi as ca
from config import Config
import math 



class TrajectoryNLP:
    def __init__(self, config, with_hsl=False):
        self.config:Config = config
        self.nlp_config = {
            "ipopt": {
                "linear_solver": "ma27" if with_hsl else "mumps",
                "print_level": 5
            }
        }

        self.build_iterative_nlp()

    def build_iterative_nlp(self):
        tf = 25
        nfe = 150
        x = ca.SX.sym("x", nfe)
        y = ca.SX.sym("y", nfe)
        theta = ca.SX.sym("theta", nfe)
        v = ca.SX.sym("v", nfe)
        a = ca.SX.sym("a", nfe)
        phi = ca.SX.sym("phi", nfe)
        omega = ca.SX.sym("omega", nfe)
        jerk = ca.SX.sym("jerk", nfe)

        hi = tf / nfe
        prev = ca.Slice(0, nfe - 1)
        next = slice(1, nfe)

        x_kin = x(next) - (x(prev) + v(prev) * hi * math.cos(theta(prev)))
        y_kin = y(next) - (y(prev) + v(prev) * hi * math.sin(theta(prev)))
        theta_kin = theta(next) - (theta(prev) + v(prev) * hi * math.tan(theta(prev)) / self.config.paramera.wheel_base)
        v_kin = v(next) - (v(prev) + hi * a(prev))
        phi_kin = phi(next) - (phi(prev) + hi * omega(prev))
        a_kin = a(next) - (a(prev) + hi * jerk(prev))

        infeasiblity = ca.sumsqr(ca.vertcat({
                                            x_kin, y_kin, theta_kin, v_kin, phi_kin, a_kin}))
        
        f_obs = self.config.opti_w_jerk * ca.sumsqr(jerk) + self.config.opti_w_omega * ca.sumsqr(omega) + self.config.opti_w_infeasibility * infeasiblity

        p = ca.vertcat()

        opti_x = ca.vertcat()

        nlp = {'x': x,  'p': p, 'f':f_obs}
        
            

        










        # Placeholder for the actual implementation of the iterative NLP build
        # You might initialize variables or prepare data structures here
        pass
