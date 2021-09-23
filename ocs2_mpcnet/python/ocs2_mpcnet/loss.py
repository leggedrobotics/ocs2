import torch

from ocs2_mpcnet.helper import bdot, bmv


class Hamiltonian:

    # Uses the linear quadratic approximation of the Hamiltonian as loss
    # H(x,u) = 1/2 dx' dHdxx dx + du' dHdux dx + 1/2 du' dHduu du + dHdx' dx + dHdu' du + H

    @staticmethod
    def compute_sample(x_inquiry, x_nominal, u_inquiry, u_nominal, dHdxx, dHdux, dHduu, dHdx, dHdu, H):
        if torch.equal(x_inquiry, x_nominal):
            du = torch.sub(u_inquiry, u_nominal)
            return 0.5 * torch.dot(du, torch.mv(dHduu, du)) + torch.dot(dHdu, du) + H
        elif torch.equal(u_inquiry, u_nominal):
            dx = torch.sub(x_inquiry, x_nominal)
            return 0.5 * torch.dot(dx, torch.mv(dHdxx, dx)) + torch.dot(dHdx, dx) + H
        else:
            dx = torch.sub(x_inquiry, x_nominal)
            du = torch.sub(u_inquiry, u_nominal)
            return 0.5 * torch.dot(dx, torch.mv(dHdxx, dx)) + torch.dot(du, torch.mv(dHdux, dx)) + 0.5 * torch.dot(du, torch.mv(dHduu, du)) + torch.dot(dHdx, dx) + torch.dot(dHdu, du) + H

    @staticmethod
    def compute_batch(x_inquiry, x_nominal, u_inquiry, u_nominal, dHdxx, dHdux, dHduu, dHdx, dHdu, H):
        if torch.equal(x_inquiry, x_nominal):
            du = torch.sub(u_inquiry, u_nominal)
            return 0.5 * bdot(du, bmv(dHduu, du)) + bdot(dHdu, du) + H
        elif torch.equal(u_inquiry, u_nominal):
            dx = torch.sub(x_inquiry, x_nominal)
            return 0.5 * bdot(dx, bmv(dHdxx, dx)) + bdot(dHdx, dx) + H
        else:
            dx = torch.sub(x_inquiry, x_nominal)
            du = torch.sub(u_inquiry, u_nominal)
            return 0.5 * bdot(dx, bmv(dHdxx, dx)) + bdot(du, bmv(dHdux, dx)) + 0.5 * bdot(du, bmv(dHduu, du)) + bdot(dHdx, dx) + bdot(dHdu, du) + H


class BehavioralCloning:

    # Uses a simple quadratic function as loss
    # BC(u) = du' R du

    def __init__(self, R, batch_size):
        self.R = R
        self.R_batch = torch.stack([R for i in range(batch_size)])

    def compute_sample(self, u_predicted, u_target):
        du = torch.sub(u_predicted, u_target)
        return torch.dot(du, torch.mv(self.R, du))

    def compute_batch(self, u_predicted, u_target):
        du = torch.sub(u_predicted, u_target)
        return bdot(du, bmv(self.R_batch, du))


class CrossEntropy:

    # Uses the cross entropy between two probability distributions as loss
    # CE(p_target, p_predicted) = - sum(p_target * log(p_predicted))

    def __init__(self, epsilon):
        self.epsilon = epsilon

    def compute_sample(self, p_target, p_predicted):
        return - torch.dot(p_target, torch.log(p_predicted + self.epsilon))

    def compute_batch(self, p_target, p_predicted):
        return - bdot(p_target, torch.log(p_predicted + self.epsilon))
