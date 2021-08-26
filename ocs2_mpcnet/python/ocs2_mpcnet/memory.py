import random
import numpy as np
from collections import namedtuple

Sample = namedtuple('sample', ('t', 'x', 'u', 'mode', 'generalized_time', 'relative_state', 'hamiltonian'))


class ReplayMemory:

    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = [None] * capacity  # pre-allocate memory
        self.position = 0
        self.size = 0

    def push(self, *args):
        sample = Sample(*args)
        for element in sample:
            if isinstance(element, (float, np.ndarray)):
                if np.any(np.isnan(element)):
                    print("Avoided pushing nan into memory", element)
                    return
                if np.any(np.isinf(element)):
                    print("Avoided pushing inf into memory", element)
                    return
        self.size = min(self.size + 1, self.capacity)
        self.memory[self.position] = sample
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return random.sample(self.memory[0:self.size], batch_size)

    def __len__(self):
        return self.size
