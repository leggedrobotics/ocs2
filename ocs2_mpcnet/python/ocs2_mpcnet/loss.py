import torch
import numpy as np

from ocs2_mpcnet import config


class Hamiltonian:

    # Uses the linear quadratic approximation of the Hamiltonian as loss
    # H(x,u) = 1/2 dx' dHdxx dx + du' dHdux dx + 1/2 du' dHduu du + dHdx' dx + dHdu' du + H

    @staticmethod
    def compute_torch(x_inquiry, x_nominal, u_inquiry, u_nominal, hamiltonian):
        dx = torch.sub(x_inquiry, x_nominal)
        du = torch.sub(u_inquiry, u_nominal)
        dHdxx = 0.5 * torch.dot(dx, torch.matmul(torch.tensor(hamiltonian.dfdxx, dtype=config.dtype, device=config.device), dx))
        dHdux = torch.dot(du, torch.matmul(torch.tensor(hamiltonian.dfdux, dtype=config.dtype, device=config.device), dx))
        dHduu = 0.5 * torch.dot(du, torch.matmul(torch.tensor(hamiltonian.dfduu, dtype=config.dtype, device=config.device), du))
        dHdx = torch.dot(torch.tensor(hamiltonian.dfdx, dtype=config.dtype, device=config.device), dx)
        dHdu = torch.dot(torch.tensor(hamiltonian.dfdu, dtype=config.dtype, device=config.device), du)
        H = torch.tensor(hamiltonian.f, dtype=config.dtype, device=config.device)
        return dHdxx + dHdux + dHduu + dHdx + dHdu + H

    @staticmethod
    def compute_numpy(x_inquiry, x_nominal, u_inquiry, u_nominal, hamiltonian):
        dx = np.subtract(x_inquiry, x_nominal)
        du = np.subtract(u_inquiry, u_nominal)
        dHdxx = 0.5 * np.dot(dx, np.matmul(hamiltonian.dfdxx, dx))
        dHdux = np.dot(du, np.matmul(hamiltonian.dfdux, dx))
        dHduu = 0.5 * np.dot(du, np.matmul(hamiltonian.dfduu, du))
        dHdx = np.dot(hamiltonian.dfdx, dx)
        dHdu = np.dot(hamiltonian.dfdu, du)
        H = hamiltonian.f
        return dHdxx + dHdux + dHduu + dHdx + dHdu + H


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


class CrossEntropy:

    # Uses the cross entropy between two probability distributions as loss
    # CE(p_target, p_predicted) = - sum(p_target * log(p_predicted))

    def __init__(self, epsilon_torch, epsilon_numpy):
        self.epsilon_torch = epsilon_torch
        self.epsilon_numpy = epsilon_numpy

    def compute_torch(self, p_target, p_predicted):
        return - torch.dot(p_target, torch.log(torch.add(p_predicted, self.epsilon_torch)))

    def compute_numpy(self, p_target, p_predicted):
        return - np.dot(p_target, np.log(np.add(p_predicted, self.epsilon_numpy)))
