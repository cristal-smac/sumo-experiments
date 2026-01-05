import pandas as pd
import numpy as np
import networkx as nx
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import pyRAPL

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
    The graph representation is only available for the preset artificial networks (line, grid). In the other cases, it works, but don't give great results
    in terms of simulation time and visualization.
    """

    def __init__(self, max_simulation_duration=None, data_frequency=1, graph_representation=False, print_timestep=500, vehicle_deletion_timesteps=[], scale_factors=None, consumption_csv_output='consumption_data.csv'):
        """
        Init of class
        Two conditions can trigger the end of the simulation : the maximum simulation duration is reached or there are no vehicles to run.
        :param max_simulation_duration: The maximum duration of the simulation in timestep.
        :type max_simulation_duration: int
        :param data_frequency: The frequence at which data are collected, in timestep.
        :type data_frequency: int
        :param graph_representation: If set to true, the TraciWrapper will create a visual representation of the network where intersections are nodes and lanes are edges. Lanes are colored by the color of the traffic light at its end, if any.
        :type graph_representation: bool
        :param print_timestep: The frequency at which the TraciWrapper will print the current step of the simulation.
        :type print_timestep: int
        :param vehicle_deletion_timesteps: A list of timesteps at which the TraciWrapper will delete all the vehicles in simulation.
        :type vehicle_deletion_timesteps: list
        :param consumption_csv_output: Filename to save consumption data of the strategy
        :type consumption_csv_output: str
        """
        self.stats_functions = []
        self.behavioural_functions = []
        self.data = {'simulation_step': []}
        self.simulation_duration = max_simulation_duration
        self.data_frequency = data_frequency
        self.graph_representation = graph_representation
        self.print_timestep = print_timestep
        self.vehicles_deletion_timesteps = vehicle_deletion_timesteps
        if scale_factors is not None:
            assert len(scale_factors) == len(vehicle_deletion_timesteps), "Length of scale_factors must be equal to length of vehicle_deletion_timesteps"
        self.scale_factors = scale_factors
        self.consumption_csv_output = consumption_csv_output

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

    def net_to_graph(self, traci):
        """
        Convert the network to a graph.
        :return: The graph representation of the network
        :rtype: networkx.Graph
        """
        G = nx.DiGraph()
        pos = {}
        intersections = traci.junction.getIDList()
        for intersection in intersections:
            G.add_node(intersection)
            pos[intersection] = traci.junction.getPosition(intersection)
        edges = traci.edge.getIDList()
        for edge in edges:
            if traci.edge.getFromJunction(edge) != traci.edge.getToJunction(edge):
                G.add_edge(traci.edge.getFromJunction(edge), traci.edge.getToJunction(edge))
        return G, pos

    def update_colors(self, G, traci):
        """
        Update graph representation with current phases.
        :param G: The initial graph representation of the network
        :type G: networkx.Graph
        :return: The updated graph representation of the network
        :rtype: networkx.Graph
        """
        junctions = traci.junction.getIDList()
        colors = {}
        colors_list = []
        for junction in junctions:
            # If junction is traffic light
            if junction in traci.trafficlight.getIDList():
                lanes = traci.trafficlight.getControlledLanes(junction)
                state = traci.trafficlight.getRedYellowGreenState(junction)
                for i in range(len(lanes)):
                    edge = traci.lane.getEdgeID(lanes[i])
                    k = (traci.edge.getFromJunction(edge), traci.edge.getToJunction(edge))
                    if state[i] == 'r':
                        colors[k] = 'red'
                    elif state[i] == 'y':
                        colors[k] = 'yellow'
                    else:
                        colors[k] = 'green'
            # If junction is not managed
            else:
                edges = traci.junction.getIncomingEdges(junction)
                for edge in edges:
                    k = (traci.edge.getFromJunction(edge), traci.edge.getToJunction(edge))
                    colors[k] = 'black'
        for edge in G.edges():
            colors_list.append(colors[edge])
        return colors_list


    def final_function(self, traci):
        """
        The final function combine all functions added to the wrapper to make only one.
        :return: dict
        """
        self.data = {'simulation_step': [], 'mean_travel_time': [], 'exiting_vehicles': [], 'mean_CO2_per_travel': [], 'mean_phase_time': []}
        step = 0
        running_vehicles = {}
        current_travel_times = []
        current_exiting_vehicles = []
        current_co2_travel = []
        current_phase = {tls: traci.trafficlight.getPhase(tls) for tls in traci.trafficlight.getIDList()}
        current_phase_durations = {tls: 0 for tls in traci.trafficlight.getIDList()}
        phase_durations = []

        pyRAPL.setup()

        csv_output = pyRAPL.outputs.CSVOutput(self.consumption_csv_output)

        if self.graph_representation:
            G, pos = self.net_to_graph(traci)

        if self.simulation_duration is None:
            resume = traci.simulation.getMinExpectedNumber() > 0
        else:
            resume = (step < self.simulation_duration) and (traci.simulation.getMinExpectedNumber()>0)

        while resume:

            if step in self.vehicles_deletion_timesteps:
                for vehicle_id in traci.vehicle.getIDList():
                    traci.vehicle.remove(vehicle_id)
                if self.scale_factors is not None:
                    index = self.vehicles_deletion_timesteps.index(step)
                    factor = self.scale_factors[index]
                    traci.simulation.setScale(factor)

            # Store the current state network as a graph
            if self.graph_representation:
                plt.clf()
                arc_rad = 0.25
                colors = self.update_colors(G, traci)
                sizes = []
                for color in colors:
                    if color == "green":
                        sizes.append(3)
                    else:
                        sizes.append(1)
                nx.draw(G, pos, connectionstyle=f'arc3, rad = {arc_rad}', node_size=100, edge_color=colors, width=sizes)
                plt.savefig(f'./Graphs/{step}.png')


            
            traci.simulationStep()

            simulation_time = traci.simulation.getTime()
            if simulation_time % self.print_timestep == 0:
                print(f"Simulation time : {simulation_time} s")
            # We catch each inserted vehicle ID
            for id in traci.simulation.getDepartedIDList():
                running_vehicles[id] = {'simulation_time': simulation_time, 'sum_co2': 0}

            # Updating CO2 emissions
            for id in traci.vehicle.getIDList():
                try:
                    running_vehicles[id]['sum_co2'] += traci.vehicle.getCO2Emission(id)
                except:
                    pass

            # We add travel time and co2 emissions for each leaving vehicle
            travel_times = []
            co2_emissions = []
            for id in traci.simulation.getArrivedIDList():
                try:
                    travel_times.append(simulation_time - running_vehicles[id]['simulation_time'])
                    co2_emissions.append(running_vehicles[id]['sum_co2'])
                except:
                    pass

            # Updating running list
            currently_running = traci.vehicle.getIDList()
            to_be_deleted = []
            for id in running_vehicles:
                if id not in currently_running:
                    to_be_deleted.append(id)
            for id in to_be_deleted:
                del running_vehicles[id]

            current_travel_times.append(np.nanmean(travel_times) if len(travel_times) > 0 else np.nan)
            current_co2_travel.append(np.nanmean(co2_emissions) if len(co2_emissions) > 0 else np.nan)
            current_exiting_vehicles.append(len(travel_times))

            # We store the phase time if the phase switches
            for tls in traci.trafficlight.getIDList():
                if 'y' in traci.trafficlight.getRedYellowGreenState(tls) and current_phase_durations[tls] != 0:
                    current_phase[tls] = traci.trafficlight.getPhase(tls)
                    phase_durations.append(current_phase_durations[tls])
                    current_phase_durations[tls] = 0
                elif 'y' not in traci.trafficlight.getRedYellowGreenState(tls):
                    current_phase_durations[tls] += 1

            if step % self.data_frequency == 0:

                # Statistical functions
                for stats_function in self.stats_functions:
                    res = stats_function(traci)
                    for key in res:
                        if key in self.data:
                            self.data[key].append(res[key])
                        else:
                            self.data[key] = [res[key]]

                # Behavioural functions
                meter = pyRAPL.Measurement('bar')
                meter.begin()
                for behavioural_function in self.behavioural_functions:
                    behavioural_function(traci)
                meter.end()
                meter.export(csv_output)

                self.data['simulation_step'].append(step + 1)
                filter = [False if i == 0 else True for i in current_exiting_vehicles]
                current_travel_times = np.array(current_travel_times)
                current_co2_travel = np.array(current_co2_travel)
                current_exiting_vehicles = np.array(current_exiting_vehicles)
                self.data['mean_travel_time'].append(np.average(current_travel_times[filter], weights=current_exiting_vehicles[filter]) if np.any(filter) else np.nan)
                self.data['mean_CO2_per_travel'].append(np.average(current_co2_travel[filter], weights=current_exiting_vehicles[filter]) if np.any(filter) else np.nan)
                self.data['exiting_vehicles'].append(np.nansum(current_exiting_vehicles))
                self.data['mean_phase_time'].append(np.average(phase_durations))
                current_travel_times = []
                current_co2_travel = []
                current_exiting_vehicles = []
                phase_durations = []
            step += 1

            if self.simulation_duration is None:
                resume = traci.simulation.getMinExpectedNumber() > 0
            else:
                resume = (step < self.simulation_duration) and (traci.simulation.getMinExpectedNumber()>0)

        csv_output.save()
        return pd.DataFrame.from_dict(self.data)


