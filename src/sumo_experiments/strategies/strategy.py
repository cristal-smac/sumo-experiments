import math
from abc import ABC, abstractmethod
import numpy as np

class Strategy(ABC):
    """
    Abstract class to create control strategies for all the traffic lights in a network.
    """

    @abstractmethod
    def run_all_agents(self, traci):
        """
        Perform the choose action for all agents of the strategy.
        :param traci: The simulation instance of Traci
        :return: Nothing
        """
        pass