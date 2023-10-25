import pandas as pd
import traci


class TraciWrapper:
    """
    Wrap TraCi functions to make only one
    The run_traci method from the Experiment class can only take one traci function as argument.
    The aim of the traci wrapper is to group multiple functions in only one object (and one final_function) to be passed to run_traci.
    Two types of traci functions are defined here :
    - Stats functions are functions that look at the simulation and returns statistics about it. This functions must return a dict object
    with the name of the data as key, and the data as value. The data can't be an iterable object.
    - Behavioural functions modify the behaviour of infrastructures within the simulation. This functions must return a new config that will be used
    in the next iterations.
    All of this function must accept a config as argument.
    The order of the functions is important : think about it when you use this wrapper.
    """

    def __init__(self):
        """
        Init of class
        """
        self.stats_functions = []
        self.behavioural_functions = []
        self.data = {'simulation_step': []}

    def add_stats_function(self, function):
        """
        Add a statistic function to the wrapper
        A statistic function is a TraCi function that will not modify the behaviour of the network,
        but only return statistics about it.
        The function must have one and only one config parameter and return a dictionary.
        :param function: The function to add
        :return: func
        """
        self.stats_functions.append(function)

    def add_behavioural_function(self, function):
        """
        Add a behavioural function to the wrapper
        A behavioural function is a TraCi function that will modify the behaviour of the network,
        and returns a modified config for next iterations.
        The function must have one and only one config parameter and return a dictionary.
        :param function: The function to add
        :return: func
        """
        self.behavioural_functions.append(function)

    def final_function(self, config):
        """
        The final function combine all functions added to the wrapper to make only one.
        :param config: The config with all parameters for all functions wrapped
        :return: dict
        """
        self.data = {'simulation_step': []}
        current_config = config
        simulation_duration = current_config['simulation_duration']
        step = 0

        while step < simulation_duration:

            traci.simulationStep()

            # Statistical functions
            for stats_function in self.stats_functions:
                res = stats_function(current_config)
                for key in res:
                    if key in self.data:
                        self.data[key].append(res[key])
                    else:
                        self.data[key] = [res[key]]

            # Behavioural functions
            for bahavioural_function in self.behavioural_functions:
                res = bahavioural_function(current_config)
                current_config = res

            self.data['simulation_step'].append(step + 1)
            step += 1

        return pd.DataFrame.from_dict(self.data)



