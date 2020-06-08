import numpy as np


class ESO:
    def __init__(self, A, B, L):
        self.A = A
        self.B = B
        self.L = L

    def compute_dot(self, eso_estimates, q, u):
        e = q - eso_estimates[0]
        z_hat = eso_estimates[1]
        z_dot = self.A * z_hat + self.B * u + self.L * e
        z_dot = np.array([z_dot[:, 0]]).T

        return z_dot
