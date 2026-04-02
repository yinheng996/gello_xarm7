from typing import Any, Dict, Protocol

import numpy as np


class Agent(Protocol):
    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        raise NotImplementedError


class DummyAgent(Agent):
    def __init__(self, num_dofs: int):
        self.num_dofs = num_dofs

    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        return np.zeros(self.num_dofs)
