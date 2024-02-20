from abc import ABC, abstractmethod

class Agent(ABC):
    """
    The Agent class is an abstract class to implements decentralized agents that will control one and only one
    intersection in a SUMO simulation.
    """

    VEHICLE_DEFAULT_LENGTH = 5

    @abstractmethod
    def choose_action(self):
        """
        Choose an action to perform for the agent.
        :return: Nothing
        """
        pass

    @abstractmethod
    def _start_agent(self):
        """
        Start the agent at the beginning of the simulation.
        """
        pass