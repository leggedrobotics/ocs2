import torch
from ocs2_mpcnet import config


class CircularMemory:

    def __init__(self, capacity, time_dimension, state_dimension, input_dimension, expert_number=1):
        # init variables
        self.capacity = capacity
        self.size = 0
        self.position = 0
        # pre-allocate memory
        self.t = torch.zeros(capacity, device=config.device, dtype=config.dtype)
        self.x = torch.zeros(capacity, state_dimension, device=config.device, dtype=config.dtype)
        self.u = torch.zeros(capacity, input_dimension, device=config.device, dtype=config.dtype)
        self.p = torch.zeros(capacity, expert_number, device=config.device, dtype=config.dtype)
        self.generalized_time = torch.zeros(capacity, time_dimension, device=config.device, dtype=config.dtype)
        self.relative_state = torch.zeros(capacity, state_dimension, device=config.device, dtype=config.dtype)
        self.input_transformation = torch.zeros(capacity, input_dimension, input_dimension, device=config.device, dtype=config.dtype)
        self.dHdxx = torch.zeros(capacity, state_dimension, state_dimension, device=config.device, dtype=config.dtype)
        self.dHdux = torch.zeros(capacity, input_dimension, state_dimension, device=config.device, dtype=config.dtype)
        self.dHduu = torch.zeros(capacity, input_dimension, input_dimension, device=config.device, dtype=config.dtype)
        self.dHdx = torch.zeros(capacity, state_dimension, device=config.device, dtype=config.dtype)
        self.dHdu = torch.zeros(capacity, input_dimension, device=config.device, dtype=config.dtype)
        self.H = torch.zeros(capacity, device=config.device, dtype=config.dtype)

    def push(self, t, x, u, p, generalized_time, relative_state, input_transformation, hamiltonian):
        # push data into memory
        # note: - torch.as_tensor: no copy as data is an ndarray of the corresponding dtype and the device is the cpu
        #       - torch.Tensor.copy_: copy performed together with potential dtype and device change
        self.t[self.position].copy_(torch.as_tensor(t, dtype=None, device=torch.device("cpu")))
        self.x[self.position].copy_(torch.as_tensor(x, dtype=None, device=torch.device("cpu")))
        self.u[self.position].copy_(torch.as_tensor(u, dtype=None, device=torch.device("cpu")))
        self.p[self.position].copy_(torch.as_tensor(p, dtype=None, device=torch.device("cpu")))
        self.generalized_time[self.position].copy_(torch.as_tensor(generalized_time, dtype=None, device=torch.device("cpu")))
        self.relative_state[self.position].copy_(torch.as_tensor(relative_state, dtype=None, device=torch.device("cpu")))
        self.input_transformation[self.position].copy_(torch.as_tensor(input_transformation, dtype=None, device=torch.device("cpu")))
        self.dHdxx[self.position].copy_(torch.as_tensor(hamiltonian.dfdxx, dtype=None, device=torch.device("cpu")))
        self.dHdux[self.position].copy_(torch.as_tensor(hamiltonian.dfdux, dtype=None, device=torch.device("cpu")))
        self.dHduu[self.position].copy_(torch.as_tensor(hamiltonian.dfduu, dtype=None, device=torch.device("cpu")))
        self.dHdx[self.position].copy_(torch.as_tensor(hamiltonian.dfdx, dtype=None, device=torch.device("cpu")))
        self.dHdu[self.position].copy_(torch.as_tensor(hamiltonian.dfdu, dtype=None, device=torch.device("cpu")))
        self.H[self.position].copy_(torch.as_tensor(hamiltonian.f, dtype=None, device=torch.device("cpu")))
        # update size and position
        self.size = min(self.size + 1, self.capacity)
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        indices = torch.randint(0, self.size, (batch_size,), device=config.device)
        t_batch = self.t[indices]
        x_batch = self.x[indices]
        u_batch = self.u[indices]
        p_batch = self.p[indices]
        generalized_time_batch = self.generalized_time[indices]
        relative_state_batch = self.relative_state[indices]
        input_transformation_batch = self.input_transformation[indices]
        dHdxx_batch = self.dHdxx[indices]
        dHdux_batch = self.dHdux[indices]
        dHduu_batch = self.dHduu[indices]
        dHdx_batch = self.dHdx[indices]
        dHdu_batch = self.dHdu[indices]
        H_batch = self.H[indices]
        return t_batch, x_batch, u_batch, p_batch, generalized_time_batch, relative_state_batch,\
               input_transformation_batch, dHdxx_batch, dHdux_batch, dHduu_batch, dHdx_batch, dHdu_batch, H_batch

    def __len__(self):
        return self.size
