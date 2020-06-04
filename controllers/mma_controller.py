import numpy as np
from .controller import Controller
from models.manipulator_model import ManiuplatorModel

class MMAController(Controller):
    def __init__(self, Tp):
        # Use parameters from manipulators/mm_planar_2dof.py
        self.models = [ManiuplatorModel(Tp, 0.1, 0.05), ManiuplatorModel(Tp, 0.01, 0.01), ManiuplatorModel(Tp, 1., 0.3)]
        self.i = 0
        self.kd = np.diag((1, 1)) * [1.5, -1]
        self.kp = np.diag((1, 1)) * [-1, 2]

    def calc_x_m(self, model, x, u):
        zeros = np.zeros((2, 2), dtype=np.float32)
        invM = np.linalg.inv(model.M(x))
        A = np.concatenate([np.concatenate([zeros, np.eye(2)], 1),
                            np.concatenate([zeros, -invM @ model.C(x)], 1)], 0)
        b = np.concatenate([zeros, invM], 0)
        return A @ x[:, np.newaxis] + b @ u

    def choose_model(self, x, u, x_dot):
        min = 1e9
        best = 0

        for i, model in enumerate(self.models):
            x_m = self.calc_x_m(model, x, u)
            dif = abs(x_m - x_dot)
            sum = np.sum(dif)
            if sum < min:
                min = sum
                best = i
        self.i = best

    def calculate_control(self, x, q_d, q_d_dot, q_dd_dot):
        q0_dot = x[2:].reshape(-1, 1)
        q0 = x[:2].reshape(-1, 1)
        q_d_dot = q_d_dot.reshape(-1, 1)
        q_d = q_d.reshape(-1, 1)
        v = q_dd_dot + self.kd.dot(q0_dot - q_d_dot) + self.kp.dot(q0 - q_d)
        q_dot = x[2:, np.newaxis]
        M = self.models[self.i].M(x)
        return M @ v + self.models[self.i].C(x) @ q_dot
