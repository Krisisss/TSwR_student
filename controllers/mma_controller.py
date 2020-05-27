import numpy as np
from .controller import Controller
from models.manipulator_model import ManiuplatorModel

class MMAController(Controller):
    def __init__(self, Tp):
        # Use parameters from manipulators/mm_planar_2dof.py
        self.models = [ManiuplatorModel(Tp, 0.1, 0.1), ManiuplatorModel(Tp, 0.01, 0.01), ManiuplatorModel(Tp, 1., 0.3)]
        self.i = 0
        self.kd = np.diag((1, 1)) * [2, -2]
        self.kp = np.diag((1, 1)) * [12, 6]

    def choose_model(self, x, u, x_dot):
        error = []
        for model in self.models:
            pass

    def calculate_control(self, x, q_d, q_d_dot, q_dd_dot):

        q0_dot = x[2:].reshape(-1, 1)
        q0 = x[:2].reshape(-1, 1)
        q_d_dot = q_d_dot.reshape(-1, 1)
        q_d = q_d.reshape(-1, 1)

        v = q_dd_dot + self.kd.dot(q0_dot - q_d_dot) + self.kp.dot(q0 - q_d)

        q_dot = x[2:, np.newaxis]
        M = self.models[self.i].M(x)
        return M @ (v + np.linalg.inv(M) @ self.models[self.i].C(x) @ q_dot)
