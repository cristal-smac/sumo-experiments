import pandas as pd
import traci
import numpy as np


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

    def __init__(self, simulation_duration, data_frequency=1):
        """
        Init of class
        """
        self.stats_functions = []
        self.behavioural_functions = []
        self.data = {'simulation_step': []}
        self.simulation_duration = simulation_duration
        self.data_frequency = data_frequency

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

    def final_function(self):
        """
        The final function combine all functions added to the wrapper to make only one.
        :return: dict
        """
        self.data = {'simulation_step': [], 'mean_travel_time': [], 'exiting_vehicles': []}
        step = 0
        running_vehicles = {}
        current_travel_times = []
        current_exiting_vehicles = []

        while step < self.simulation_duration:

            traci.simulationStep()

            simulation_time = traci.simulation.getTime()

            # We catch each inserted vehicle ID
            for id in traci.simulation.getDepartedIDList():
                running_vehicles[id] = simulation_time

            # We compute travel time for each leaving vehicle
            travel_times = []
            for id in traci.simulation.getArrivedIDList():
                travel_times.append(simulation_time - running_vehicles[id])
                del running_vehicles[id]

            current_travel_times.append(np.nanmean(travel_times) if len(travel_times) > 0 else np.nan)
            current_exiting_vehicles.append(len(travel_times))

            if step % self.data_frequency == 0:

                # Statistical functions
                for stats_function in self.stats_functions:
                    res = stats_function()
                    for key in res:
                        if key in self.data:
                            self.data[key].append(res[key])
                        else:
                            self.data[key] = [res[key]]

                # Behavioural functions
                for behavioural_function in self.behavioural_functions:
                    behavioural_function()

                self.data['simulation_step'].append(step + 1)
                filter = [False if i == 0 else True for i in current_exiting_vehicles]
                current_travel_times = np.array(current_travel_times)
                current_exiting_vehicles = np.array(current_exiting_vehicles)
                self.data['mean_travel_time'].append(np.average(current_travel_times[filter], weights=current_exiting_vehicles[filter]) if np.any(filter) else np.nan)
                self.data['exiting_vehicles'].append(np.nansum(current_exiting_vehicles))
                current_travel_times = []
                current_exiting_vehicles = []
            step += 1

        return pd.DataFrame.from_dict(self.data)



