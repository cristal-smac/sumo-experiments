import warnings

import numpy as np
from sumo_experiments.components import InfrastructureBuilder, FlowBuilder, DetectorBuilder
import traci

class SquareNetwork:
    """
        The SquareNetwork class contains a set of functions for creating a SUMO network containing
        joined intersections that join to form a square.

        There are 4 different types of function:
            - Functions generating infrastructures
            - Functions generating flows
            - Functions generationg detectors
            - Functions representing junction management strategies, usable only when an experiment is launched with TraCi

        It is not possible to combine certain functions to generate a SUMO network.
        Check the documentation of functions for more information.
    """

    WE_GREEN_LIGHT = 0
    NS_GREEN_LIGHT = 2

    CONFIG_PARAMETER_LIST = [
        'exp_name', 'lane_length', 'max_speed', 'green_time', 'yellow_time', 'minimum_edge_length', 'maximum_edge_length',
        'stop_generation_time', 'flow_frequency', 'period_time', 'load_vector', 'coeff_matrix', 'min_duration_tl',
        'max_duration_tl', 'vehicle_threshold', 'simulation_duration', 'boolean_detector_length', 'square_side_length',
        'distribution', 'data_frequency'
    ]

    def __init__(self):
        """
        Init of class
        """
        self.edges_length = {}
        self.random = False


    ### Network ###

    def generate_random_infrastructures(self, config):
        """
        Generate the sumo infrastructures for a square network.
        The distance between two nodes of the network is set randomly, between 'minimum_edge_length' and 'maximum_edge_length' in config.
        The crossroad is managed by traffic lights on each road.
        The infrastructures can be customized with the config dict passed as parameter.
        A default configuration is set, and each modification in the config is modified in the default configuration.

        Valid parameters for config :
        - "minimum_edge_length" (int) : The minimum length of an edge (in meters)
        - "maximum_edge_length" (int) : The maximum length of an edge (in meters)
        - "green_time" (int) : The default green time for each phase (in seconds)
        - "yellow_time" (int) : The default yellow time for each phase (in seconds)lt
        - "max_speed" (float) : The max speed on each lane
        - "square_side_length" (int) : The number of intersections that compose a side of the square

        :param config: Customized network configuration. Check documentation to see all parameters.
        :type config: dict
        :return: All infrastructures in a NetworkBuilder object.
        :rtype: sumo_experiments.src.components.NetworkBuilder
        """

        self.random = True

        for key in config:
            if key not in self.CONFIG_PARAMETER_LIST:
                warnings.warn(f"The config parameter {key} is not a valid parameter.", stacklevel=2)

        # Select parameters
        minimum_length = config['minimum_edge_length']
        maximum_length = config['maximum_edge_length']
        if 'max_duration_tl' in config:
            green_time = config['max_duration_tl']
        else:
            green_time = config['green_time']
        yellow_time = config['yellow_time']
        max_speed = config['max_speed']
        square_side_length = config["square_side_length"]

        net = InfrastructureBuilder()

        # We check the 'square_side_length' value
        if square_side_length <= 1:
            raise ValueError("The 'square_side_length' parameter must be greater than 1.")

        # Generate and save edges length
        offset_values_x = [np.random.uniform(minimum_length, maximum_length) for _ in range(square_side_length + 1)]
        offset_values_y = [np.random.uniform(minimum_length, maximum_length) for _ in range(square_side_length + 1)]
        x_positions = [0] + offset_values_x
        y_positions = [0] + offset_values_y

        # We add the nodes
        for x in range(square_side_length + 2):
            for y in range(square_side_length + 2):

                # No nodes on the corners
                if not self.is_corner(x, y, square_side_length):
                    # If node is on boarder, we configure it as a network entry
                    if (x in [0, square_side_length + 1]) or (y in [0, square_side_length + 1]):
                        net.add_node(id=f'x{x}-y{y}', x=sum(offset_values_x[:x]),
                                     y=sum(offset_values_y[:y]))
                    # Else, it's a traffic light
                    else:
                        net.add_node(id=f'x{x}-y{y}', x=sum(offset_values_x[:x]),
                                     y=sum(offset_values_y[:y]), type='traffic_light', tl_program=f'x{x}-y{y}')

        # We add an edge type
        net.add_edge_type(id='default', params={'numLanes': 1, 'speed': max_speed})

        # We add the edges
        for x in range(square_side_length + 2):
            for y in range(square_side_length + 2):

                # No nodes on corner
                if not self.is_corner(x, y, square_side_length):
                    # We join current node with node left and right (if they exist, and if current is not on boarder)
                    if y not in [0, square_side_length + 1]:
                        if x < square_side_length + 1:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x + 1}-y{y}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x + 1}-y{y}', edge_type='default')
                            self.edges_length[f'edge_x{x}-y{y}_x{x + 1}-y{y}'] = x_positions[x+1]
                        if x > 0:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x - 1}-y{y}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x - 1}-y{y}', edge_type='default')
                            self.edges_length[f'edge_x{x}-y{y}_x{x - 1}-y{y}'] = x_positions[x]
                    # We join current node with node above and below (if they exist, and if current is not on boarder)
                    if x not in [0, square_side_length + 1]:
                        if y < square_side_length + 1:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x}-y{y + 1}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x}-y{y + 1}', edge_type='default')
                            self.edges_length[f'edge_x{x}-y{y}_x{x}-y{y + 1}'] = y_positions[y + 1]
                        if y > 0:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x}-y{y - 1}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x}-y{y - 1}', edge_type='default')
                            self.edges_length[f'edge_x{x}-y{y}_x{x}-y{y - 1}'] = y_positions[y]

        # We add the connections
        for x in range(square_side_length + 2):
            for y in range(square_side_length + 2):

                # No nodes on corners
                if not self.is_corner(x, y, square_side_length):

                    # We join edges with every neighbours on right and left
                    # If node is on boarder above or below, no connections
                    if y not in [0, square_side_length + 1]:
                        # Connections on the right
                        if x < square_side_length:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x + 1}-y{y}',
                                               to_edge=f'edge_x{x + 1}-y{y}_x{x + 2}-y{y}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x + 1}-y{y}',
                                               to_edge=f'edge_x{x + 1}-y{y}_x{x + 1}-y{y + 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x + 1}-y{y}',
                                               to_edge=f'edge_x{x + 1}-y{y}_x{x + 1}-y{y - 1}')
                        # Connections on the left
                        if x > 1:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x - 1}-y{y}',
                                               to_edge=f'edge_x{x - 1}-y{y}_x{x - 2}-y{y}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x - 1}-y{y}',
                                               to_edge=f'edge_x{x - 1}-y{y}_x{x - 1}-y{y + 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x - 1}-y{y}',
                                               to_edge=f'edge_x{x - 1}-y{y}_x{x - 1}-y{y - 1}')

                    # We join edges with every neighbours above and below
                    # If node is on boarder right or left, no connections
                    if x not in [0, square_side_length + 1]:
                        # Above
                        if y < square_side_length:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y + 1}',
                                               to_edge=f'edge_x{x}-y{y + 1}_x{x}-y{y + 2}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y + 1}',
                                               to_edge=f'edge_x{x}-y{y + 1}_x{x + 1}-y{y + 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y + 1}',
                                               to_edge=f'edge_x{x}-y{y + 1}_x{x - 1}-y{y + 1}')
                        # Below
                        if y > 1:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y - 1}',
                                               to_edge=f'edge_x{x}-y{y - 1}_x{x}-y{y - 2}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y - 1}',
                                               to_edge=f'edge_x{x}-y{y - 1}_x{x + 1}-y{y - 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y - 1}',
                                               to_edge=f'edge_x{x}-y{y - 1}_x{x - 1}-y{y - 1}')

        # We add traffic light programs
        for x in range(1, square_side_length + 1):
            for y in range(1, square_side_length + 1):
                net.add_traffic_light_program(id=f'x{x}-y{y}', phases=[
                    {'duration': green_time, 'state': 'rrrGGGrrrGGG'},
                    {'duration': yellow_time, 'state': 'rrryyyrrryyy'},
                    {'duration': green_time, 'state': 'GGGrrrGGGrrr'},
                    {'duration': yellow_time, 'state': 'yyyrrryyyrrr'}])

        return net


    def generate_infrastructures(self, config):
        """
        Generate the sumo infrastructures for a square network.
        The length of each edge is set by the parameter 'lane_length'.
        The crossroad is managed by traffic lights on each road.
        The infrastructures can be customized with the config dict passed as parameter.
        A default configuration is set, and each modification in the config is modified in the default configuration.

        Valid parameters for config :
        - "lane_length" (int) : The length of all edges (in meters)
        - "green_time" (int) : The default green time for each phase (in seconds)
        - "yellow_time" (int) : The default yellow time for each phase (in seconds)lt
        - "max_speed" (float) : The max speed on each lane
        - "square_side_length" (int) : The number of intersections that compose a side of the square

        :param config: Customized network configuration. Check documentation to see all parameters.
        :type config: dict
        :return: All infrastructures in a NetworkBuilder object.
        :rtype: sumo_experiments.src.components.NetworkBuilder
        """

        self.random = False
        for key in config:
            if key not in self.CONFIG_PARAMETER_LIST:
                warnings.warn(f"The config parameter {key} is not a valid parameter.", stacklevel=2)

        # Select parameters
        lane_length = config['lane_length']
        if 'max_duration_tl' in config:
            green_time = config['max_duration_tl']
        else:
            green_time = config['green_time']
        yellow_time = config['yellow_time']
        max_speed = config['max_speed']
        square_side_length = config["square_side_length"]

        net = InfrastructureBuilder()

        # We check the 'square_side_length' value
        if square_side_length <= 1:
            raise ValueError("The 'square_side_length' parameter must be greater than 1.")

        # We add the nodes
        for x in range(square_side_length + 2):
            for y in range(square_side_length + 2):

                # No nodes on corners
                if not self.is_corner(x, y, square_side_length):
                    # If node is on boarder, we configure it as a network entry
                    if (x in [0, square_side_length + 1]) or (y in [0, square_side_length + 1]):
                        net.add_node(id=f'x{x}-y{y}', x=x * lane_length, y=y * lane_length)
                    # Else, it's a traffic light
                    else:
                        net.add_node(id=f'x{x}-y{y}', x=x * lane_length, y=y * lane_length,
                                     type='traffic_light', tl_program=f'x{x}-y{y}')

        # We add the edge type
        net.add_edge_type(id='default', params={'numLanes': 1, 'speed': max_speed})

        # We add the edges
        for x in range(square_side_length + 2):
            for y in range(square_side_length + 2):

                # No nodes on corners
                if not self.is_corner(x, y, square_side_length):
                    # We join current node with node left and right (if they exist, and if current is not on boarder)
                    if y not in [0, square_side_length + 1]:
                        if x < square_side_length + 1:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x + 1}-y{y}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x + 1}-y{y}', edge_type='default')
                        if x > 0:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x - 1}-y{y}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x - 1}-y{y}', edge_type='default')
                    # We join current node with node above and below (if they exist, and if current is not on boarder)
                    if x not in [0, square_side_length + 1]:
                        if y < square_side_length + 1:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x}-y{y + 1}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x}-y{y + 1}', edge_type='default')
                        if y > 0:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x}-y{y - 1}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x}-y{y - 1}', edge_type='default')

        # We add the connections
        for x in range(square_side_length + 2):
            for y in range(square_side_length + 2):

                # No nodes on corners
                if not self.is_corner(x, y, square_side_length):
                    # We join edges with every neighbours on right and left
                    # If node is on boarder above or below, no connections
                    if y not in [0, square_side_length + 1]:
                        # To the right
                        if x < square_side_length:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x + 1}-y{y}',
                                               to_edge=f'edge_x{x + 1}-y{y}_x{x + 2}-y{y}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x + 1}-y{y}',
                                               to_edge=f'edge_x{x + 1}-y{y}_x{x + 1}-y{y + 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x + 1}-y{y}',
                                               to_edge=f'edge_x{x + 1}-y{y}_x{x + 1}-y{y - 1}')
                        # To the left
                        if x > 1:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x - 1}-y{y}',
                                               to_edge=f'edge_x{x - 1}-y{y}_x{x - 2}-y{y}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x - 1}-y{y}',
                                               to_edge=f'edge_x{x - 1}-y{y}_x{x - 1}-y{y + 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x - 1}-y{y}',
                                               to_edge=f'edge_x{x - 1}-y{y}_x{x - 1}-y{y - 1}')

                    # We join edges with every neighbours above and below
                    # If node is on boarder right or left, no connections
                    if x not in [0, square_side_length + 1]:
                        # Above
                        if y < square_side_length:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y + 1}',
                                               to_edge=f'edge_x{x}-y{y + 1}_x{x}-y{y + 2}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y + 1}',
                                               to_edge=f'edge_x{x}-y{y + 1}_x{x + 1}-y{y + 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y + 1}',
                                               to_edge=f'edge_x{x}-y{y + 1}_x{x - 1}-y{y + 1}')
                        # Below
                        if y > 1:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y - 1}',
                                               to_edge=f'edge_x{x}-y{y - 1}_x{x}-y{y - 2}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y - 1}',
                                               to_edge=f'edge_x{x}-y{y - 1}_x{x + 1}-y{y - 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y - 1}',
                                               to_edge=f'edge_x{x}-y{y - 1}_x{x - 1}-y{y - 1}')

        # We add traffic light programs
        for x in range(1, square_side_length + 1):
            for y in range(1, square_side_length + 1):
                net.add_traffic_light_program(id=f'x{x}-y{y}', phases=[
                    {'duration': green_time, 'state': 'rrrGGGrrrGGG'},
                    {'duration': yellow_time, 'state': 'rrryyyrrryyy'},
                    {'duration': green_time, 'state': 'GGGrrrGGGrrr'},
                    {'duration': yellow_time, 'state': 'yyyrrryyyrrr'}])

        return net



    ### Routes ###

    def generate_flows_only_ahead(self, config):
        """
        Generate flows for a square network.
        At the intersection, vehicles can not turn. They can only go ahead.
        The config parameter can contain more parameter to define the frequency of vehicles from each entry, and the
        simulation step where the flow will end.

        Valid parameters for config :
        - "stop_generation_time" (int) : The default simulation step when flows will end
        - "flow_frequency" (int) : The default flows frequency (in vehicles/hour)
        - "square_side_length" (int) : The number of intersections that compose a side of the square
        - "distribution" (str) : The distribution law for all flows. "uniform" inserts vehicles each n simulation steps
        'binomial' inserts vehicle at each simulation step with a given probability. Each of the law respect the flow frequency.

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        :return: All flows in a FlowBuilder object.
        :rtype: sumo_experiments.src.components.FlowBuilder
        """

        # Select parameters
        stop_generation_time = config['stop_generation_time']
        flow_frequency = config['flow_frequency']
        square_side_length = config["square_side_length"]
        distribution = config['distribution']

        routes = FlowBuilder()

        # We add all the entry edges in a list, clockwise.
        # The correspondant exit edge is added at the same time in another list.
        liste_entrees = []
        liste_sorties = []
        for x in range(1, square_side_length + 1):
            liste_entrees.append(f'edge_x{x}-y{square_side_length + 1}_x{x}-y{square_side_length}')
            liste_sorties.append(f'edge_x{x}-y1_x{x}-y0')
        for y in range(square_side_length, 0, -1):
            liste_entrees.append(f'edge_x{square_side_length + 1}-y{y}_x{square_side_length}-y{y}')
            liste_sorties.append(f'edge_x1-y{y}_x0-y{y}')
        for x in range(square_side_length, 0, -1):
            liste_entrees.append(f'edge_x{x}-y0_x{x}-y1')
            liste_sorties.append(f'edge_x{x}-y{square_side_length}_x{x}-y{square_side_length + 1}')
        for y in range(1, square_side_length + 1):
            liste_entrees.append(f'edge_x0-y{y}_x1-y{y}')
            liste_sorties.append(f'edge_x{square_side_length}-y{y}_x{square_side_length + 1}-y{y}')

        # We add the vehicle type
        routes.add_v_type(id='car0')

        # We create a flow for each entry with the correspondant exit
        for x in range(len(liste_entrees)):
            routes.add_flow(id=f"flow_{liste_entrees[x]}_{liste_sorties[x]}",
                            from_edge=liste_entrees[x],
                            to_edge=liste_sorties[x],
                            end=stop_generation_time,
                            frequency=flow_frequency,
                            v_type='car0',
                            distribution=distribution)

        return routes


    def generate_flows_all_directions(self, config):
        """
        Generate flows for a square network.
        At the intersection, vehicles can go left, right or ahead. The proportion for each exit is uniform.
        The config parameter can contain more parameter to define the frequency of vehicles from each entry, and the
        simulation step where the flow will end.

        Valid parameters for config :
        - "stop_generation_time" (int) : The default simulation step when flows will end
        - "flow_frequency" (int) : The default flows frequency (in vehicles/hour)
        - "square_side_length" (int) : The number of intersections that compose a side of the square
        - "distribution" (str) : The distribution law for all flows. "uniform" inserts vehicles each n simulation steps
        'binomial' inserts vehicle at each simulation step with a given probability. Each of the law respect the flow frequency.

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        :return: All flows in a FlowBuilder object.
        :rtype: sumo_experiments.src.components.FlowBuilder
        """

        # Select parameters
        stop_generation_time = config['stop_generation_time']
        flow_frequency = config['flow_frequency']
        square_side_length = config["square_side_length"]
        distribution = config['distribution']

        routes = FlowBuilder()

        # We add all the entry edges in a list, clockwise.
        # The correspondant exit edge is added at the same time in another list.
        liste_entrees = []
        liste_sorties = []
        for x in range(1, square_side_length + 1):
            liste_entrees.append(f'edge_x{x}-y{square_side_length + 1}_x{x}-y{square_side_length}')
            liste_sorties.append(f'edge_x{x}-y{square_side_length}_x{x}-y{square_side_length + 1}')
        for y in range(square_side_length, 0, -1):
            liste_entrees.append(f'edge_x{square_side_length + 1}-y{y}_x{square_side_length}-y{y}')
            liste_sorties.append(f'edge_x{square_side_length}-y{y}_x{square_side_length + 1}-y{y}')
        for x in range(square_side_length, 0, -1):
            liste_entrees.append(f'edge_x{x}-y0_x{x}-y1')
            liste_sorties.append(f'edge_x{x}-y1_x{x}-y0')
        for y in range(1, square_side_length + 1):
            liste_entrees.append(f'edge_x0-y{y}_x1-y{y}')
            liste_sorties.append(f'edge_x1-y{y}_x0-y{y}')

        # We add the vehicle type
        routes.add_v_type(id='car0')

        for i in range(len(liste_entrees)):
            for j in range(len(liste_sorties)):
                # A vehicle can't leave by the edge where it has entered
                if i != j:
                    routes.add_flow(id=f"flow_{liste_entrees[i]}_{liste_entrees[j]}",
                                    from_edge=liste_entrees[i],
                                    to_edge=liste_sorties[j],
                                    end=stop_generation_time,
                                    frequency=flow_frequency // ((square_side_length * 4) - 1),
                                    v_type='car0',
                                    distribution=distribution)

        return routes


    def generate_flows_with_matrix(self, config):
        """
        Generate flows for a square network.
        At the intersection, vehicles can go left, right or ahead.
        The vehicle frequency varies over time, following a scheme describe in a load vector and a coefficient matrix.
        The load vector describes, for each period, the frequency of vehicle entering the network.
        The coefficient matrix describes the proportion of load that will follow each route.
        Each scheme of frequency last a time defined in simulation steps.
        The config parameter can contain more parameter to define the frequency of vehicles from each entry, and the
        simulation step where the flow will end.

        Valid parameters for config :
        - "coeff_matrix" (numpy.ndarray) : The proportion of vehicles on each route
        - "load_vector" (numpy.ndarray) : The vehicle frequency on the network for each period
        - "period_time" (int) : The period duration (in simulation steps)
        - "square_side_length" (int) : The number of intersections that compose a side of the square
        - "distribution" (str) : The distribution law for all flows. "uniform" inserts vehicles each n simulation steps
        'binomial' inserts vehicle at each simulation step with a given probability. Each of the law respect the flow frequency.

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        :return: All flows in a FlowBuilder object.
        :rtype: sumo_experiments.src.components.FlowBuilder
        """

        # Select parameters
        coeffs_matrix = config['coeff_matrix']
        load_vector = config['load_vector']
        period_time = config['period_time']
        square_side_length = config['square_side_length']
        distribution = config['distribution']

        routes = FlowBuilder()

        # We add all the entry edges in a list, clockwise.
        # The same edge, but in the exit direction, is added at the same time in an exit edge list.
        liste_entrees = []
        liste_sorties = []
        for x in range(1, square_side_length + 1):
            liste_entrees.append(f'edge_x{x}-y{square_side_length + 1}_x{x}-y{square_side_length}')
            liste_sorties.append(f'edge_x{x}-y{square_side_length}_x{x}-y{square_side_length + 1}')
        for y in range(square_side_length, 0, -1):
            liste_entrees.append(f'edge_x{square_side_length + 1}-y{y}_x{square_side_length}-y{y}')
            liste_sorties.append(f'edge_x{square_side_length}-y{y}_x{square_side_length + 1}-y{y}')
        for x in range(square_side_length, 0, -1):
            liste_entrees.append(f'edge_x{x}-y0_x{x}-y1')
            liste_sorties.append(f'edge_x{x}-y1_x{x}-y0')
        for y in range(1, square_side_length + 1):
            liste_entrees.append(f'edge_x0-y{y}_x1-y{y}')
            liste_sorties.append(f'edge_x1-y{y}_x0-y{y}')

        # We add the vehicle type
        routes.add_v_type(id='car0')

        for period in range(len(load_vector)):

            vecteur_coeffs = coeffs_matrix[:, period]
            flows = vecteur_coeffs * load_vector[period]
            flow_start = period_time * period
            flow_end = period_time * (period + 1)
            compteur = 0

            for i in range(len(liste_entrees)):
                for j in range(len(liste_sorties)):
                    # A vehicle can't leave by the edge where it has entered
                    if i != j:
                        routes.add_flow(id=f"flow_{liste_entrees[i]}_{liste_entrees[j]}_{period}",
                                        from_edge=liste_entrees[i],
                                        to_edge=liste_sorties[j],
                                        begin=flow_start,
                                        end=flow_end,
                                        frequency=flows[compteur],
                                        v_type='car0',
                                        distribution=distribution)
                        compteur += 1

        return routes



    ### Additionals ###

    def generate_numerical_detectors(self, config):
        """
        Generate a DetectorBuilder with a numerical detector for each lane going to an intersection.
        A numerical detector counts and returns the number of vehicles on its scope. In SUMO, a numerical
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.

        Valid parameters for config :
        - "square_side_length" (int) : The number of intersections that compose a side of the square

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        :return: An empty DetectorBuilder object.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """

        # Select parameters
        square_side_length = config['square_side_length']

        detectors = DetectorBuilder()

        # We add the detectors
        for x in range(square_side_length + 2):
            for y in range(square_side_length + 2):

                # No nodes on the corners
                if not self.is_corner(x, y, square_side_length):
                    if y not in [0, square_side_length + 1]:
                        if x < square_side_length:
                            detectors.add_lane_area_detector(id=f'detector_x{x}-y{y}_x{x + 1}-y{y}',
                                                             lane=f'edge_x{x}-y{y}_x{x + 1}-y{y}_0')
                        if x > 1:
                            detectors.add_lane_area_detector(id=f'detector_x{x}-y{y}_x{x - 1}-y{y}',
                                                             lane=f'edge_x{x}-y{y}_x{x - 1}-y{y}_0')
                    if x not in [0, square_side_length + 1]:
                        if y < square_side_length:
                            detectors.add_lane_area_detector(id=f'detector_x{x}-y{y}_x{x}-y{y + 1}',
                                                             lane=f'edge_x{x}-y{y}_x{x}-y{y + 1}_0')
                        if y > 1:
                            detectors.add_lane_area_detector(id=f'detector_x{x}-y{y}_x{x}-y{y - 1}',
                                                             lane=f'edge_x{x}-y{y}_x{x}-y{y - 1}_0')

        return detectors

    def generate_boolean_detectors(self, config):
        """
        Generate a DetectorBuilder with a boolean detector for each lane going to an intersection.
        A boolean detector returns if a vehicle is on its scope or not. In SUMO, a boolean
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.

        Valid parameters for config :
        - "lane_length" (int) : The default length for each lane (in meters)
        - "boolean_detector_length" (float) : The scope size of the detectors
        - "square_side_length" (int) : The number of intersections that compose a side of the square

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        :return: An empty DetectorBuilder object.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """

        # Select parameters
        boolean_detector_length = config['boolean_detector_length']
        square_side_length = config['square_side_length']
        lane_length = config['lane_length']

        detectors = DetectorBuilder()

        # We add the detectors
        for x in range(square_side_length + 2):
            for y in range(square_side_length + 2):

                # No nodes on corners
                if not self.is_corner(x, y, square_side_length):
                    if y not in [0, square_side_length + 1]:
                        if x < square_side_length:
                            if x == 0:
                                lane_length_x = self.edges_length[f'edge_x{x}-y{y}_x{x + 1}-y{y}'] if self.random else lane_length
                                detectors.add_lane_area_detector(id=f'detector_x{x}-y{y}_x{x + 1}-y{y}',
                                                                 lane=f'edge_x{x}-y{y}_x{x + 1}-y{y}_0',
                                                                 pos=(lane_length_x - boolean_detector_length - 7.2))
                            else:
                                lane_length_x = self.edges_length[f'edge_x{x}-y{y}_x{x + 1}-y{y}'] if self.random else lane_length
                                detectors.add_lane_area_detector(id=f'detector_x{x}-y{y}_x{x + 1}-y{y}',
                                                                 lane=f'edge_x{x}-y{y}_x{x + 1}-y{y}_0',
                                                                 pos=(lane_length_x - boolean_detector_length - 7.2 * 2))
                        if x > 1:
                            if x == square_side_length + 1:
                                lane_length_x = self.edges_length[f'edge_x{x}-y{y}_x{x - 1}-y{y}'] if self.random else lane_length
                                detectors.add_lane_area_detector(id=f'detector_x{x}-y{y}_x{x - 1}-y{y}',
                                                                 lane=f'edge_x{x}-y{y}_x{x - 1}-y{y}_0',
                                                                 pos=(lane_length_x - boolean_detector_length - 7.2))
                            else:
                                lane_length_x = self.edges_length[f'edge_x{x}-y{y}_x{x - 1}-y{y}'] if self.random else lane_length
                                detectors.add_lane_area_detector(id=f'detector_x{x}-y{y}_x{x - 1}-y{y}',
                                                                 lane=f'edge_x{x}-y{y}_x{x - 1}-y{y}_0',
                                                                 pos=(lane_length_x - boolean_detector_length - 7.2 * 2))
                    if x not in [0, square_side_length + 1]:
                        if y < square_side_length:
                            if y == 0:
                                lane_length_y = self.edges_length[f'edge_x{x}-y{y}_x{x}-y{y + 1}'] if self.random else lane_length
                                detectors.add_lane_area_detector(id=f'detector_x{x}-y{y}_x{x}-y{y + 1}',
                                                                 lane=f'edge_x{x}-y{y}_x{x}-y{y + 1}_0',
                                                                 pos=(lane_length_y - boolean_detector_length - 7.2))
                            else:
                                lane_length_y = self.edges_length[f'edge_x{x}-y{y}_x{x}-y{y + 1}'] if self.random else lane_length
                                detectors.add_lane_area_detector(id=f'detector_x{x}-y{y}_x{x}-y{y + 1}',
                                                                 lane=f'edge_x{x}-y{y}_x{x}-y{y + 1}_0',
                                                                 pos=(lane_length_y - boolean_detector_length - 7.2 * 2))
                        if y > 1:
                            if y == square_side_length + 1:
                                lane_length_y = self.edges_length[f'edge_x{x}-y{y}_x{x}-y{y - 1}'] if self.random else lane_length
                                detectors.add_lane_area_detector(id=f'detector_x{x}-y{y}_x{x}-y{y - 1}',
                                                                 lane=f'edge_x{x}-y{y}_x{x}-y{y - 1}_0',
                                                                 pos=(lane_length_y - boolean_detector_length - 7.2))
                            else:
                                lane_length_y = self.edges_length[f'edge_x{x}-y{y}_x{x}-y{y - 1}'] if self.random else lane_length
                                detectors.add_lane_area_detector(id=f'detector_x{x}-y{y}_x{x}-y{y - 1}',
                                                                 lane=f'edge_x{x}-y{y}_x{x}-y{y - 1}_0',
                                                                 pos=(lane_length_y - boolean_detector_length - 7.2 * 2))

        return detectors



    ### Strategies ###

    def boolean_detection(self, config):
        """
        To be used with a network equipped with boolean detectors.

        Before running the simulation, three variables must be set in config :
        - The minimum duration for a traffic light phase (default 30 seconds)
        - The maximum duration for a traffic light phase (default 60 seconds)
        - The simulation duration (default 1000 simulation steps)

        A traffic light has two phases (excluding yellow phases) :
        - Green for north-south, red for west-east
        - Red for north-south, green for west-east

        At the beginning of the simulation, The intersections are set to green for the west-east way
        and red for the north-south way.
        When a traffic light detects a car on a lane where traffic light is red, and if it doesn't
        detect any car on the green lanes, it switches to the other phase if the current phase
        is set since more than the minimum duration time.
        If this condition doesn't occur, the traffic light switch to the other phase when the current phase
        last for more than the maximum duration time.

        Valid parameters for config :
        - "min_duration_tl" (int) : The minimum number of simulation step for a traffic light phase
        - "max_duration_tl" (int) : The maximum number of simulation step for a traffic light phase
        - "simulation_duration" (int) : The number of simulation steps of the experiment
        - "square_side_length" (int) : The number of intersections that compose a side of the square

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        """

        square_side_length = config['square_side_length']
        if 'cooldown_step' not in config:
            config['cooldown_step'] = np.zeros((square_side_length, square_side_length))

        # Select parameters
        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        cooldown_step = config['cooldown_step']

        for x in range(1, square_side_length + 1):
            for y in range(1, square_side_length + 1):

                if cooldown_step[x - 1, y - 1] > min_duration_tl:
                    if traci.trafficlight.getPhase(f'x{x}-y{y}') == self.WE_GREEN_LIGHT:
                        quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(
                            f'detector_x{x}-y{y + 1}_x{x}-y{y}') >= 1 \
                                              or traci.lanearea.getLastStepVehicleNumber(
                                    f'detector_x{x}-y{y - 1}_x{x}-y{y}') >= 1)
                        autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(
                            f'detector_x{x - 1}-y{y}_x{x}-y{y}') == 0 \
                                           and traci.lanearea.getLastStepVehicleNumber(
                                    f'detector_x{x + 1}-y{y}_x{x}-y{y}') == 0)
                        if (quelqun_en_attente and autre_voie_vide) or cooldown_step[x - 1, y - 1] > max_duration_tl:
                            traci.trafficlight.setPhase(f'x{x}-y{y}', self.WE_GREEN_LIGHT + 1)  # Passage au orange
                            cooldown_step[x - 1, y - 1] = 0

                    elif traci.trafficlight.getPhase(f'x{x}-y{y}') == self.NS_GREEN_LIGHT:
                        quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(
                            f'detector_x{x - 1}-y{y}_x{x}-y{y}') >= 1 \
                                              or traci.lanearea.getLastStepVehicleNumber(
                                    f'detector_x{x + 1}-y{y}_x{x}-y{y}') >= 1)
                        autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(
                            f'detector_x{x}-y{y + 1}_x{x}-y{y}') == 0 \
                                           and traci.lanearea.getLastStepVehicleNumber(
                                    f'detector_x{x}-y{y - 1}_x{x}-y{y}') == 0)
                        if (quelqun_en_attente and autre_voie_vide) or cooldown_step[x - 1, y - 1] > max_duration_tl:
                            traci.trafficlight.setPhase(f'x{x}-y{y}', self.NS_GREEN_LIGHT + 1)
                            cooldown_step[x - 1, y - 1] = 0

                cooldown_step[x - 1, y - 1] += 1

        config['cooldown_step'] = cooldown_step
        return config

    def numerical_detection_all_vehicles(self, config):
        """
        To be used with a network equipped with numerical detectors.

        Before running the simulation, four variables must be set in config :
        - The minimum duration for a traffic light phase (default 30 seconds)
        - The maximum duration for a traffic light phase (default 60 seconds)
        - The vehicle threshold to trigger a phase change (default 5)
        - The simulation duration (default 1000 simulation steps)

        A traffic light has two phases (excluding yellow phases) :
        - Green for north-south, red for west-east
        - Red for north-south, green for west-east

        At the beginning of the simulation, The intersections are set to green for the west-east way
        and red for the north-south way.
        When a traffic light detect more than the threshold of cars on a lane where traffic light is red,
        it switches to the other phase if the current phase is set since more than the minimum
        duration time. Both stopped and running cars are considered in this strategy.
        If this condition doesn't occur, the traffic light switch to the other phase when the current phase
        last for more than the maximum duration time.

        Valid parameters for config :
        - "min_duration_tl" (int) : The minimum number of simulation step for a traffic light phase
        - "max_duration_tl" (int) : The maximum number of simulation step for a traffic light phase
        - "vehicle_threshold" (int) : The number of waiting vehicles that trigger a traffic light switch
        - "simulation_duration" (int) : The number of simulation steps of the experiment
        - "square_side_length" (int) : The number of intersections that compose a side of the square

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        """

        square_side_length = config['square_side_length']
        if 'cooldown_step' not in config:
            config['cooldown_step'] = np.zeros((square_side_length, square_side_length))

        # Select parameters
        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        vehicle_threshold = config["vehicle_threshold"]
        cooldown_step = config['cooldown_step']

        for x in range(1, square_side_length + 1):
            for y in range(1, square_side_length + 1):

                if cooldown_step[x - 1, y - 1] > min_duration_tl:

                    if traci.trafficlight.getPhase(f'x{x}-y{y}') == self.NS_GREEN_LIGHT:

                        if traci.lanearea.getLastStepVehicleNumber(
                                f'detector_x{x - 1}-y{y}_x{x}-y{y}') >= vehicle_threshold \
                                or traci.lanearea.getLastStepVehicleNumber(
                            f'detector_x{x + 1}-y{y}_x{x}-y{y}') >= vehicle_threshold \
                                or cooldown_step[x - 1, y - 1] > max_duration_tl:
                            traci.trafficlight.setPhase(f'x{x}-y{y}', self.NS_GREEN_LIGHT + 1)  # Passage au orange
                            cooldown_step[x - 1, y - 1] = 0

                    elif traci.trafficlight.getPhase(f'x{x}-y{y}') == self.WE_GREEN_LIGHT:

                        if traci.lanearea.getLastStepVehicleNumber(
                                f'detector_x{x}-y{y + 1}_x{x}-y{y}') >= vehicle_threshold \
                                or traci.lanearea.getLastStepVehicleNumber(
                            f'detector_x{x}-y{y - 1}_x{x}-y{y}') >= vehicle_threshold \
                                or cooldown_step[x - 1, y - 1] > max_duration_tl:
                            traci.trafficlight.setPhase(f'x{x}-y{y}', self.WE_GREEN_LIGHT + 1)  # Passage au orange
                            cooldown_step[x - 1, y - 1] = 0

                cooldown_step[x - 1, y - 1] += 1

        config['cooldown_step'] = cooldown_step
        return config

    def numerical_detection_stopped_vehicles(self, config):
        """
        To be used with a network equipped with numerical detectors.

        Before running the simulation, four variables must be set in config :
        - The minimum duration for a traffic light phase (default 30 seconds)
        - The maximum duration for a traffic light phase (default 60 seconds)
        - The vehicle threshold to trigger a phase change (default 5)
        - The simulation duration (default 1000 simulation steps)

        A traffic light has two phases (excluding yellow phases) :
        - Green for north-south, red for west-east
        - Red for north-south, green for west-east

        At the beginning of the simulation, The intersection is set to green for the west-east way
        and red for the north-south way.
        When a traffic light detect more than the threshold of cars on a lane where traffic light is red,
        it switches to the other phase if the current phase is set since more than the minimum
        duration time. Only stopped cars are considered in this strategy.
        If this condition doesn't occur, the traffic light switch to the other phase when the current phase
        last for more than the maximum duration time.

        Valid parameters for config :
        - "min_duration_tl" (int) : The minimum number of simulation step for a traffic light phase
        - "max_duration_tl" (int) : The maximum number of simulation step for a traffic light phase
        - "vehicle_threshold" (int) : The number of waiting vehicles that trigger a traffic light switch
        - "simulation_duration" (int) : The number of simulation steps of the experiment
        - "square_side_length" (int) : The number of intersections that compose a side of the square

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        """

        square_side_length = config['square_side_length']
        if 'cooldown_step' not in config:
            config['cooldown_step'] = np.zeros((square_side_length, square_side_length))

        # Select parameters
        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        vehicle_threshold = config["vehicle_threshold"]
        cooldown_step = config['cooldown_step']

        for x in range(1, square_side_length + 1):
            for y in range(1, square_side_length + 1):

                if cooldown_step[x - 1, y - 1] > min_duration_tl:

                    if traci.trafficlight.getPhase(f'x{x}-y{y}') == self.NS_GREEN_LIGHT:

                        if traci.lanearea.getJamLengthVehicle(f'detector_x{x - 1}-y{y}_x{x}-y{y}') >= vehicle_threshold \
                                or traci.lanearea.getJamLengthVehicle(
                            f'detector_x{x + 1}-y{y}_x{x}-y{y}') >= vehicle_threshold \
                                or cooldown_step[x - 1, y - 1] > max_duration_tl:
                            traci.trafficlight.setPhase(f'x{x}-y{y}', self.NS_GREEN_LIGHT + 1)  # Passage au orange
                            cooldown_step[x - 1, y - 1] = 0


                    elif traci.trafficlight.getPhase(f'x{x}-y{y}') == self.WE_GREEN_LIGHT:

                        if traci.lanearea.getJamLengthVehicle(f'detector_x{x}-y{y + 1}_x{x}-y{y}') >= vehicle_threshold \
                                or traci.lanearea.getJamLengthVehicle(
                            f'detector_x{x}-y{y - 1}_x{x}-y{y}') >= vehicle_threshold \
                                or cooldown_step[x - 1, y - 1] > max_duration_tl:
                            traci.trafficlight.setPhase(f'x{x}-y{y}', self.WE_GREEN_LIGHT + 1)  # Passage au orange
                            cooldown_step[x - 1, y - 1] = 0

                cooldown_step[x - 1, y - 1] += 1

        config['cooldown_step'] = cooldown_step
        return config

    def is_corner(self, x, y, square_side_length):
        """
        Check if a node in the network is a corner.
        :param x: The x-coordinate of the node
        :type x: int
        :param y: The y-coordinate of the node
        :type y: int
        :param square_side_length: The square side length of the network
        :type square_side_length: int
        :return: True if the node is a corner, False otherwise
        :rtype: bool
        """
        answer = ((x == y) and (x == 0 or x == square_side_length + 1)) \
                    or (x == 0 and y == square_side_length + 1) \
                    or (y == 0 and x == square_side_length + 1)
        return answer
