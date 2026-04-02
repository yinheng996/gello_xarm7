from abc import abstractmethod
from typing import Dict, Protocol

import numpy as np


class Robot(Protocol):
    @abstractmethod
    def num_dofs(self) -> int:
        raise NotImplementedError

    @abstractmethod
    def get_joint_state(self) -> np.ndarray:
        raise NotImplementedError

    @abstractmethod
    def command_joint_state(self, joint_state: np.ndarray) -> None:
        raise NotImplementedError

    @abstractmethod
    def get_observations(self) -> Dict[str, np.ndarray]:
        raise NotImplementedError


class PrintRobot(Robot):
    """A robot that prints commanded joint state — useful for testing."""

    def __init__(self, num_dofs: int, dont_print: bool = False):
        self._num_dofs = num_dofs
        self._joint_state = np.zeros(num_dofs)
        self._dont_print = dont_print

    def num_dofs(self) -> int:
        return self._num_dofs

    def get_joint_state(self) -> np.ndarray:
        return self._joint_state

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        self._joint_state = joint_state
        if not self._dont_print:
            print(self._joint_state)

    def get_observations(self) -> Dict[str, np.ndarray]:
        js = self.get_joint_state()
        return {
            "joint_positions":  js,
            "joint_velocities": js,
            "ee_pos_quat":      np.zeros(7),
            "gripper_position": np.array(0),
        }
