import numpy as np
from .controller import Controller


class PDDecentralizedController(Controller):
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd

    # def calculate_control(self, q, q_dot, q_d, q_d_dot, q_d_ddot):
    #     ### TODO: Please implement me
    #     u = None
    #     return u

    def calculate_control(self, q, q_dot, q_d, q_d_dot, q_d_ddot):

        e = q - q_d
        e_dot = q_dot - q_d_dot
        u = self.kp*e + self.kd*e_dot

        return u
        # q0_dot = x[2:].reshape(-1, 1)
        # q0 = x[:2].reshape(-1, 1)
        # q_d_dot = q_d_dot.reshape(-1, 1)
        # q_d = q_d.reshape(-1, 1)
        #
        # v = q_dd_dot + self.kd.dot(q0_dot - q_d_dot) + self.kp.dot(q0 - q_d)
        #
        # q_dot = x[2:, np.newaxis]
        # M = self.models[self.i].M(x)
        # return M @ (v + np.linalg.inv(M) @ self.models[self.i].C(x) @ q_dot)
