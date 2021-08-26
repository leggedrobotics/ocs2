import torch
import numpy as np


class Hamiltonian:

    # Uses the quadratic approximation of the Hamiltonian as loss
    # H(x,u) = 1/2 dx' dHdxx dx + du' dHdux dx + 1/2 du' dHduu du + dHdx' dx + dHdu' du + H
 
    def compute_torch(self, x, u, hamiltonian):
        # TODO (areske): implement once approximation of Hamiltonian is available
        return

    def compute_numpy(self, x, u, hamiltonian):
        # TODO (areske): implement once approximation of Hamiltonian is available
        return


class BehavioralCloning:

    # Uses a simple quadratic function as loss
    # BC(u) = du' R du

    def __init__(self, R_torch, R_numpy):
        self.R_torch = R_torch
        self.R_numpy = R_numpy

    def compute_torch(self, u_predicted, u_target):
        du = torch.sub(u_predicted, u_target)
        return torch.dot(du, torch.matmul(self.R_torch, du))

    def compute_numpy(self, u_predicted, u_target):
        du = np.subtract(u_predicted, u_target)
        return np.dot(du, np.matmul(self.R_numpy, du))
