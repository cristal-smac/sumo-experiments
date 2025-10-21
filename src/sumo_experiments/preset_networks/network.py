from abc import ABC, abstractmethod


class Network(ABC):
    """
    Abstract class for networks.
    """

    @abstractmethod
    def run(self, traci_function, gui=False, seed=None, no_warnings=True, nb_threads=1, time_to_teleport=150):
        """
        Launch an SUMO simulation with the network configuration.
        First build the configuration files and then launch SUMO.
        This function uses TraCi, which means that the infrastructures can be controlled
        simulation steps by simpulation steps. So a function using TraCi must be defined
        to run SUMO with TraCi.
        :param traci_function: The function using TraCi package and that can control infrastructures.
        :type: function
        :param gui: True to run SUMO in graphical mode. False otherwise.
        :type gui: bool
        :param seed: The seed of the simulation. Same seeds = same simulations.
        :type seed: int
        :param no_warnings: If set to True, no warnings when executing SUMO.
        :type no_warnings: bool
        :param nb_threads: Number of thread to run SUMO
        :type nb_threads: int
        :param time_to_teleport: The time for a vehicle to teleport when the network is blocked
        :type time_to_teleport: int
        """
        pass

    @abstractmethod
    def clean_files(self):
        """
        Clean all simulation files.
        """
