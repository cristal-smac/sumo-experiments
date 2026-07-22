import pandas as pd
import numpy as np
import networkx as nx
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
from tqdm import tqdm

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

    def __init__(self, max_simulation_duration=None, data_frequency=1, graph_representation=False, print_timestep=500, vehicle_deletion_timesteps=[], scale_factors=None, save_phases=False, phases_file='phases.csv', track_edge_flows=True):
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
        :param save_phases: If True, collect the current phase of each traffic light intersection at each simulation step
        :type save_phases: bool
        :param phases_file: Name of the file to store the current phase of each traffic light. Used only if save_phases is set to True.
        :type phases_file: str
        :param track_edge_flows: If True, collect the set of distinct vehicle IDs seen on each edge during the whole simulation. Adds one TraCI call per edge per step, so disable it on large networks if not needed.
        :type track_edge_flows: bool
        :param flows_file: Name of the file to store the per-edge flow counts. Used only if track_edge_flows is set to True.
        :type flows_file: str
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
        self.tl_phases = {}
        self.save_phases = save_phases
        self.phases_file = phases_file
        self.track_edge_flows = bool(track_edge_flows)

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
                # if traci.edge.getToJunction(edge) == '202818248#6-AddedOffRampNode'
                #     print(traci.edge.getToJunction(edge))
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
        tl_ids = traci.trafficlight.getIDList()
        current_phase = {tls: traci.trafficlight.getPhase(tls) for tls in tl_ids}
        current_phase_durations = {tls: 0 for tls in tl_ids}
        phase_durations = []
        dt = 1
        deletion_step_to_index = {s: i for i, s in enumerate(self.vehicles_deletion_timesteps)}
        pbar_total = self.simulation_duration if self.simulation_duration is not None else None
        _t = {"sumo": 0.0, "behav": 0.0}

        if self.graph_representation:
            G, pos = self.net_to_graph(traci)
            nx.write_adjlist(G, path="lille_graph_adjacency.txt")

        if self.save_phases:
            for tl_id in tl_ids:
                self.tl_phases[tl_id] = []

        if self.simulation_duration is None:
            resume = traci.simulation.getMinExpectedNumber() > 0
        else:
            resume = (step < self.simulation_duration) and (traci.simulation.getMinExpectedNumber()>0)

        # if self.get_flows:
        flows = {}
        if self.track_edge_flows:
            # Built once here: edges don't change during a run, so there's no
            # need to call traci.edge.getIDList() again on every step.
            for edge in traci.edge.getIDList():
                #if '_' not in edge:
                from_junction = traci.edge.getFromJunction(edge)
                if '#' in from_junction:
                    from_junction = from_junction.split('#')[0]
                to_junction = traci.edge.getToJunction(edge)
                if '#' in to_junction:
                    to_junction = to_junction.split('#')[0]
                #flows[(from_junction, to_junction, edge)] = []
                flows[(from_junction, to_junction, edge)] = set()

        with tqdm(total=pbar_total, desc='SUMO simulation', unit='step', disable=False, miniters=100, mininterval=1.0) as pbar:
            while resume:
                deletion_index = deletion_step_to_index.get(step)
                reset_this_step = deletion_index is not None
                setattr(traci, '_sumo_experiments_episode_reset', reset_this_step)

            # Store the current state network as a graph
            # if self.graph_representation:
            #     plt.clf()
            #     arc_rad = 0.25
            #     colors = self.update_colors(G, traci)
            #     sizes = []
            #     for color in colors:
            #         if color == "green":
            #             sizes.append(3)
            #         else:
            #             sizes.append(1)
            #     nx.draw(G, pos, connectionstyle=f'arc3, rad = {arc_rad}', node_size=100, edge_color=colors, width=sizes)
            #     plt.savefig(f'./Graphs/{step}.png')


                traci.simulationStep()

                simulation_time = traci.simulation.getTime()
                pbar.update(1)
                if self.print_timestep and simulation_time % self.print_timestep == 0:
                    pbar.set_postfix(active_vehicles=len(running_vehicles))
            # We catch each inserted vehicle ID
                for id in traci.simulation.getDepartedIDList():
                    running_vehicles[id] = {'simulation_time': simulation_time, 'sum_co2': 0}

                arrived_ids = traci.simulation.getArrivedIDList()
                arrived_set = set(arrived_ids)

            # Updating CO2 emissions
                for vid, record in list(running_vehicles.items()):
                    try:
                        record['sum_co2'] += traci.vehicle.getCO2Emission(vid) * dt
                    except Exception:
                        if vid not in arrived_set:
                            running_vehicles.pop(vid, None)

            # We add travel time and co2 emissions for each leaving vehicle
                travel_times = []
                co2_emissions = []
                for vid in arrived_ids:
                    record = running_vehicles.pop(vid, None)
                    if record is None:
                        continue
                    travel_times.append(simulation_time - record['simulation_time'])
                    co2_emissions.append(record['sum_co2'])

                current_travel_times.append(np.nanmean(travel_times) if travel_times else np.nan)
                current_co2_travel.append(np.nanmean(co2_emissions) if co2_emissions else np.nan)
                current_exiting_vehicles.append(len(travel_times))

            # We store the phase time if the phase switches
            # NOT USED ?????
                for tls in tl_ids:
                    state = traci.trafficlight.getRedYellowGreenState(tls)
                    if 'y' in state and current_phase_durations[tls] != 0:
                        current_phase[tls] = traci.trafficlight.getPhase(tls)
                        phase_durations.append(current_phase_durations[tls])
                        current_phase_durations[tls] = 0
                    elif 'y' not in state:
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

                    self.data['simulation_step'].append(step + 1)
                    cev = np.array(current_exiting_vehicles)
                    ctt = np.array(current_travel_times)
                    cco2 = np.array(current_co2_travel)
                    valid = cev > 0
                    self.data['mean_travel_time'].append(np.average(ctt[valid], weights=cev[valid]) if np.any(valid) else np.nan)
                    self.data['mean_CO2_per_travel'].append(np.average(cco2[valid], weights=cev[valid]) if np.any(valid) else np.nan)
                    self.data['exiting_vehicles'].append(np.nansum(cev))
                    self.data['mean_phase_time'].append(np.average(phase_durations) if phase_durations else np.nan)
                    current_travel_times = []
                    current_co2_travel = []
                    current_exiting_vehicles = []
                    phase_durations = []

            # Behavioural functions
                for behavioural_function in self.behavioural_functions:
                    behavioural_function(traci)
                if self.save_phases:
                    for tl_id in tl_ids:
                        self.tl_phases[tl_id].append(traci.trafficlight.getPhase(tl_id))

            #if self.get_flows:
                if self.track_edge_flows:
                    for flow in flows:
                        edge = flow[2]
                        #flows[flow].append(traci.edge.getLastStepOccupancy(edge))
                        flows[flow].update(traci.edge.getLastStepVehicleIDs(edge))

            # Defer hard reset until after this step's control/stats so terminal
            # transition uses pre-reset environment dynamics.
                if reset_this_step:
                    for vehicle_id in traci.vehicle.getIDList():
                        traci.vehicle.remove(vehicle_id)
                    # Drain the pending insertion backlog too: getIDList() returns
                    # only running vehicles, so undeparted vehicles would otherwise
                    # accumulate forever and make every simulationStep O(backlog).
                    traci.simulation.clearPending()
                    running_vehicles.clear()
                    if self.scale_factors is not None:
                        factor = self.scale_factors[deletion_index]
                        traci.simulation.setScale(factor)

                step += 1

                if self.simulation_duration is None:
                    resume = traci.simulation.getMinExpectedNumber() > 0
                else:
                    resume = (step < self.simulation_duration) and (traci.simulation.getMinExpectedNumber() > 0)

        setattr(traci, '_sumo_experiments_episode_reset', False)

        if self.save_phases:
            pd.DataFrame(self.tl_phases).to_csv(self.phases_file)

        if self.track_edge_flows:
            flow_values = {}
            for flow in flows:
                flows[flow] = len(flows[flow])
                flow_values[(flow[0], flow[1])] = flows[flow]
            #print(flow_values)

            with open("occupancies.txt", "w") as f:
                f.write(str(flows))

        tl_nodes = {}
        for tl in traci.trafficlight.getIDList():
            lanes = traci.trafficlight.getControlledLanes(tl)
            edges = set([traci.lane.getEdgeID(l) for l in lanes])
            nodes = set([traci.edge.getToJunction(e) for e in edges])
            tl_nodes[tl] = list(nodes)
        #print(tl_nodes)



        return pd.DataFrame.from_dict(self.data)
        #return flow_values


