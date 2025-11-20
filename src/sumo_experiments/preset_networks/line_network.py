import os
import random
import numpy as np
from sumo_experiments.components import InfrastructureBuilder, FlowBuilder, DetectorBuilder
import libsumo as traci

from sumo_experiments.preset_networks import ArtificialNetwork


class LineNetwork(ArtificialNetwork):
    """
        The LineNetwork class contains a set of functions for creating a SUMO network containing
        n consecutive crossroads.

        There are 4 different types of function:
            - Functions generating infrastructures
            - Functions generating flows
            - Functions generationg detectors
            - Functions representing junction management strategies, usable only when an experiment is launched with TraCi

        It is not possible to combine certain functions to generate a SUMO network.
        Check the documentation of functions for more information.
    """

    GREEN_LIGHT_HORIZONTAL = 0
    GREEN_LIGHT_VERTICAL = 2

    def __init__(self,
                 nb_intersections,
                 lane_length=100,
                 nb_lanes_main=1,
                 nb_lanes_minor=1,
                 max_speed=50,
                 number_of_phases=2,
                 flow_type='all_directions',
                 stop_generation_time=3600,
                 flow_frequency=100,
                 distribution='binomial',
                 period_time=None,
                 load_vector=None,
                 coeff_matrix=None,
                 boolean_detectors_length=20,
                 saturation_detectors_length=20,
                 saturation_detectors_position=0
                 ):
        """
        Init of class.
        :param nb_intersections: The number of intersections for the network
        :type nb_intersections: int
        :param lane_length: The length of all edges (in meters)
        :type lane_length: int
        :param nb_lanes_main: The number of lanes on the main road. Must be greater than 0.
        :type nb_lanes_main: int
        :param nb_lanes_minor: The number of lanes on the minor roads. Must be greater than 0.
        :type nb_lanes_minor: int
        :param max_speed: The max speed on each lane (in km/h)
        :type max_speed: int
        :param nb_phases: The number of phases of each traffic light
        :type nb_phases: int
        :param flow_type: The type of traffic flows to be generated. Can be 'all_directions', 'only_ahead' or 'matrix'.
        If 'all_directions', generates flows from every entry to every exit.
        If 'only_ahead', generates flows that do not turn at intersections.
        If 'matrix', generates flows from every entry to every exit with flows varying according to 'load_vector' and 'coeff_matrix'.
        :type flow_type: str
        :param stop_generation_time: The default simulation step when flows will end
        :type stop_generation_time: int
        :param flow_frequency: The default flows frequency (in vehicles/hour/entry)
        :type flow_frequency: int
        :param distribution: The distribution law for all flows. "uniform" inserts vehicles each n simulation steps
        'binomial' inserts vehicle at each simulation step with a given probability. Each of the law respect the flow frequency.
        :type distribution: str
        :param period_time: The period duration (in simulation steps). Only used if flow_type is 'matrix'.
        :type period_time: int
        :param load_vector: The vehicle frequency on the network for each period (in vehs/h). Only used if flow_type is 'matrix'.
        :type load_vector: numpy.ndarray
        :param coeff_matrix: The proportion of load charge for each route. Only used if flow_type is 'matrix'.
        :type coeff_matrix: numpy.ndarray
        :param boolean_detectors_length: The scope size of the detectors (in meters)
        :type boolean_detectors_length: int
        :param saturation_detectors_length: The scope size of the saturation detectors (in meters)
        :type saturation_detectors_length: int
        """
        super().__init__(f'line_network_{random.randint(1, 1000000)}')
        if nb_intersections < 2:
            raise ValueError('nb_intersections must be 2 or more.')
        if nb_lanes_main < 1:
            raise ValueError('nb_lanes_main must be 1 or more.')
        self.nb_intersections = nb_intersections
        self.saturation_detectors_position = saturation_detectors_position
        # Create infrastructures
        infrastructures = self.generate_infrastructures(lane_length, nb_lanes_main, nb_lanes_minor, max_speed, number_of_phases)
        # Create flows
        if flow_type == 'only_ahead':
            self.generate_flows_only_ahead(stop_generation_time, flow_frequency, distribution)
        elif flow_type == 'all_directions':
            flows = self.generate_flows_all_directions(stop_generation_time, flow_frequency, distribution)
        elif flow_type == 'matrix':
            if load_vector is not None and period_time is not None and coeff_matrix is not None:
                flows = self.generate_flows_with_matrix(period_time, load_vector, coeff_matrix)
            else:
                raise ValueError("load_vector and coeff_matrix ad period_time must be provided when flow_type is 'matrix'.")
        else:
            raise ValueError("flow_type must be 'only_ahead' or 'all_directions' or 'matrix'.")
        # Creates detectors
        detectors = self.generate_all_detectors(nb_lanes_main, nb_lanes_minor, lane_length, boolean_detectors_length, saturation_detectors_length)
        if number_of_phases == 2:
            self.create_TLS_DETECTORS_2_phases(nb_lanes_main, nb_lanes_minor)
        elif number_of_phases == 4:
            self.create_TLS_DETECTORS_4_phases(nb_lanes_main, nb_lanes_minor)
        # Building files
        infrastructures.build(self.file_names)
        flows.build(self.file_names)
        detectors.build(self.file_names)
        # Using netconvert to create the network
        cmd = f'$SUMO_HOME/bin/netconvert -n {self.file_names["nodes"]} -e {self.file_names["edges"]} -x {self.file_names["connections"]} -i {self.file_names["trafic_light_programs"]} -t {self.file_names["types"]} -o {self.file_names["network"]} --no-warnings'
        os.system(cmd)




    ### Networks ###

    def generate_infrastructures(self,
                                 lane_length,
                                 nb_lanes_main,
                                 nb_lanes_minor,
                                 max_speed,
                                 nb_phases):
        """
        Generate the sumo infrastructures for a network with n consecutive crossroads.

        :param lane_length: The default length for each lane (in meters)
        :type lane_length: int
        :param nb_lanes_main: The number of lanes on the main road. Must be greater than 0.
        :type nb_lanes_main: int
        :param nb_lanes_minor: The number of lanes on the minor roads. Must be greater than 0.
        :type nb_lanes_minor: int
        :param max_speed: The max speed on each lane (in km/h)
        :type max_speed: int
        :param nb_phases: The number of phases for each traffic light. Must be 2 or 4.
        :type nb_phases: int
        :return: All infrastructures in a NetworkBuilder object.
        :rtype: sumo_experiments.src.components.NetworkBuilder
        :raise: ValueError if nb_intersections is under 2
        """
        net = InfrastructureBuilder()

        # Create nodes
        net.add_node(id='w', x=0, y=0)
        for i in range(self.nb_intersections):
            i = i+1
            new_x = lane_length * i
            net.add_node(id=f'c{i}', x=new_x, y=0, type='traffic_light', tl_program=f'c{i}')
            net.add_node(id=f's{i}', x=new_x, y=-lane_length)
            net.add_node(id=f'n{i}', x=new_x, y=lane_length)
        x_east = lane_length * (self.nb_intersections + 1)
        net.add_node(id='e', x=x_east, y=0)

        # Create edges type
        net.add_edge_type(id='default', params={'numLanes': '1', 'speed': max_speed})

        # Create edges
        net.add_edge(id='edge_wc1', from_node='w', to_node='c1', edge_type='default', nb_lanes=nb_lanes_main)
        net.add_edge(id='edge_c1w', from_node='c1', to_node='w', edge_type='default', nb_lanes=nb_lanes_main)
        for i in range(1, self.nb_intersections):
            net.add_edge(id=f'edge_n{i}c{i}', from_node=f'n{i}', to_node=f'c{i}', edge_type='default', nb_lanes=nb_lanes_minor)
            net.add_edge(id=f'edge_c{i}n{i}', from_node=f'c{i}', to_node=f'n{i}', edge_type='default', nb_lanes=nb_lanes_minor)
            net.add_edge(id=f'edge_s{i}c{i}', from_node=f's{i}', to_node=f'c{i}', edge_type='default', nb_lanes=nb_lanes_minor)
            net.add_edge(id=f'edge_c{i}s{i}', from_node=f'c{i}', to_node=f's{i}', edge_type='default', nb_lanes=nb_lanes_minor)
            net.add_edge(id=f'edge_c{i}c{i + 1}', from_node=f'c{i}', to_node=f'c{i + 1}', edge_type='default', nb_lanes=nb_lanes_main)
            net.add_edge(id=f'edge_c{i + 1}c{i}', from_node=f'c{i + 1}', to_node=f'c{i}', edge_type='default', nb_lanes=nb_lanes_main)
        id_last_inter = self.nb_intersections
        net.add_edge(id=f'edge_n{id_last_inter}c{id_last_inter}', from_node=f'n{id_last_inter}', to_node=f'c{id_last_inter}', edge_type='default', nb_lanes=nb_lanes_minor)
        net.add_edge(id=f'edge_c{id_last_inter}n{id_last_inter}', from_node=f'c{id_last_inter}', to_node=f'n{id_last_inter}', edge_type='default', nb_lanes=nb_lanes_minor)
        net.add_edge(id=f'edge_s{id_last_inter}c{id_last_inter}', from_node=f's{id_last_inter}', to_node=f'c{id_last_inter}', edge_type='default', nb_lanes=nb_lanes_minor)
        net.add_edge(id=f'edge_c{id_last_inter}s{id_last_inter}', from_node=f'c{id_last_inter}', to_node=f's{id_last_inter}', edge_type='default', nb_lanes=nb_lanes_minor)
        net.add_edge(id=f'edge_c{id_last_inter}e', from_node=f'c{id_last_inter}', to_node=f'e', edge_type='default', nb_lanes=nb_lanes_main)
        net.add_edge(id=f'edge_ec{id_last_inter}', from_node=f'e', to_node=f'c{id_last_inter}', edge_type='default', nb_lanes=nb_lanes_main)

        # Create connections
        for maj_lane in range(nb_lanes_main):
            net.add_connection(from_edge='edge_n1c1', to_edge='edge_c1w', from_lane=0, to_lane=maj_lane)
            net.add_connection(from_edge='edge_n1c1', to_edge='edge_c1c2', from_lane=nb_lanes_minor-1, to_lane=maj_lane)
            net.add_connection(from_edge='edge_wc1', to_edge='edge_c1c2', from_lane=maj_lane, to_lane=maj_lane)
            net.add_connection(from_edge='edge_c2c1', to_edge='edge_c1w', from_lane=maj_lane, to_lane=maj_lane)
            net.add_connection(from_edge='edge_s1c1', to_edge='edge_c1w', from_lane=nb_lanes_minor-1, to_lane=maj_lane)
            net.add_connection(from_edge='edge_s1c1', to_edge='edge_c1c2', from_lane=0, to_lane=maj_lane)
        for min_lane in range(nb_lanes_minor):
            net.add_connection(from_edge='edge_wc1', to_edge='edge_c1n1', from_lane=nb_lanes_main - 1, to_lane=min_lane)
            net.add_connection(from_edge='edge_wc1', to_edge='edge_c1s1', from_lane=0, to_lane=min_lane)
            net.add_connection(from_edge='edge_n1c1', to_edge='edge_c1s1', from_lane=min_lane, to_lane=min_lane)
            net.add_connection(from_edge='edge_s1c1', to_edge='edge_c1n1', from_lane=min_lane, to_lane=min_lane)
            net.add_connection(from_edge='edge_c2c1', to_edge='edge_c1n1', from_lane=0, to_lane=min_lane)
            net.add_connection(from_edge='edge_c2c1', to_edge='edge_c1s1', from_lane=nb_lanes_main - 1, to_lane=min_lane)
        for i in range(1, self.nb_intersections - 1):
            id = i + 1
            for maj_lane in range(nb_lanes_main):
                net.add_connection(from_edge=f'edge_n{id}c{id}', to_edge=f'edge_c{id}c{id - 1}', from_lane=0, to_lane=maj_lane)
                net.add_connection(from_edge=f'edge_n{id}c{id}', to_edge=f'edge_c{id}c{id + 1}', from_lane=nb_lanes_minor - 1, to_lane=maj_lane)
                net.add_connection(from_edge=f'edge_c{id - 1}c{id}', to_edge=f'edge_c{id}c{id + 1}', from_lane=maj_lane, to_lane=maj_lane)
                net.add_connection(from_edge=f'edge_c{id + 1}c{id}', to_edge=f'edge_c{id}c{id - 1}', from_lane=maj_lane, to_lane=maj_lane)
                net.add_connection(from_edge=f'edge_s{id}c{id}', to_edge=f'edge_c{id}c{id - 1}', from_lane=nb_lanes_minor - 1, to_lane=maj_lane)
                net.add_connection(from_edge=f'edge_s{id}c{id}', to_edge=f'edge_c{id}c{id + 1}', from_lane=0, to_lane=maj_lane)
            for min_lane in range(nb_lanes_minor):
                net.add_connection(from_edge=f'edge_c{id - 1}c{id}', to_edge=f'edge_c{id}n{id}', from_lane=nb_lanes_main - 1, to_lane=min_lane)
                net.add_connection(from_edge=f'edge_c{id - 1}c{id}', to_edge=f'edge_c{id}s{id}', from_lane=0, to_lane=min_lane)
                net.add_connection(from_edge=f'edge_n{id}c{id}', to_edge=f'edge_c{id}s{id}', from_lane=min_lane, to_lane=min_lane)
                net.add_connection(from_edge=f'edge_s{id}c{id}', to_edge=f'edge_c{id}n{id}', from_lane=min_lane, to_lane=min_lane)
                net.add_connection(from_edge=f'edge_c{id + 1}c{id}', to_edge=f'edge_c{id}n{id}', from_lane=0, to_lane=min_lane)
                net.add_connection(from_edge=f'edge_c{id + 1}c{id}', to_edge=f'edge_c{id}s{id}', from_lane=nb_lanes_main - 1, to_lane=min_lane)
        id_last_inter = self.nb_intersections
        for maj_lane in range(nb_lanes_main):
            net.add_connection(from_edge=f'edge_n{id_last_inter}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}c{id_last_inter - 1}', from_lane=0, to_lane=maj_lane)
            net.add_connection(from_edge=f'edge_n{id_last_inter}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}e', from_lane=nb_lanes_minor-1, to_lane=maj_lane)
            net.add_connection(from_edge=f'edge_c{id_last_inter - 1}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}e', from_lane=maj_lane, to_lane=maj_lane)
            net.add_connection(from_edge=f'edge_ec{id_last_inter}', to_edge=f'edge_c{id_last_inter}c{id_last_inter - 1}', from_lane=maj_lane, to_lane=maj_lane)
            net.add_connection(from_edge=f'edge_s{id_last_inter}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}c{id_last_inter - 1}', from_lane=nb_lanes_minor-1, to_lane=maj_lane)
            net.add_connection(from_edge=f'edge_s{id_last_inter}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}e', from_lane=0, to_lane=maj_lane)
        for min_lane in range(nb_lanes_minor):
            net.add_connection(from_edge=f'edge_c{id_last_inter - 1}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}n{id_last_inter}', from_lane=nb_lanes_main - 1, to_lane=min_lane)
            net.add_connection(from_edge=f'edge_c{id_last_inter - 1}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}s{id_last_inter}', from_lane=0, to_lane=min_lane)
            net.add_connection(from_edge=f'edge_n{id_last_inter}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}s{id_last_inter}', from_lane=min_lane, to_lane=min_lane)
            net.add_connection(from_edge=f'edge_s{id_last_inter}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}n{id_last_inter}', from_lane=min_lane, to_lane=min_lane)
            net.add_connection(from_edge=f'edge_ec{id_last_inter}', to_edge=f'edge_c{id_last_inter}n{id_last_inter}', from_lane=0, to_lane=min_lane)
            net.add_connection(from_edge=f'edge_ec{id_last_inter}', to_edge=f'edge_c{id_last_inter}s{id_last_inter}', from_lane=nb_lanes_main - 1, to_lane=min_lane)


        # Create traffic lights programs
        if nb_phases == 2:
            for i in range(self.nb_intersections):
                i = i + 1
                net.add_traffic_light_program(id=f'c{i}',
                                              phases=[{'duration': 1000, 'state': 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'G'*(nb_lanes_main+2*nb_lanes_minor) + 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'G'*(nb_lanes_main+2*nb_lanes_minor)},
                                                      {'duration': 1000, 'state': 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'y'*(nb_lanes_main+2*nb_lanes_minor) + 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'y'*(nb_lanes_main+2*nb_lanes_minor)},
                                                      {'duration': 1000, 'state': 'G'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor) + 'G'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor)},
                                                      {'duration': 1000, 'state': 'y'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor) + 'y'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor)}])
        elif nb_phases == 4:
            for i in range(self.nb_intersections):
                i = i + 1
                net.add_traffic_light_program(id=f'c{i}',
                                              phases=[{'duration': 1000, 'state': 'G'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor) + 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor)},
                                                      {'duration': 1000, 'state': 'y'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor) + 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor)},
                                                      {'duration': 1000, 'state': 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'G'*(nb_lanes_main+2*nb_lanes_minor) + 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor)},
                                                      {'duration': 1000, 'state': 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'y'*(nb_lanes_main+2*nb_lanes_minor) + 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor)},
                                                      {'duration': 1000, 'state': 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor) + 'G'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor)},
                                                      {'duration': 1000, 'state': 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor) + 'y'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor)},
                                                      {'duration': 1000, 'state': 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor) + 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'G'*(nb_lanes_main+2*nb_lanes_minor)},
                                                      {'duration': 1000, 'state': 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'r'*(nb_lanes_main+2*nb_lanes_minor) + 'r'*(nb_lanes_minor+2*nb_lanes_main) + 'y'*(nb_lanes_main+2*nb_lanes_minor)},
                                                      ])

        return net



    ### Routes ###

    def generate_flows_only_ahead(self,
                                  stop_generation_time,
                                  flow_frequency,
                                  distribution='binomial'):
        """
        Generate flows for a network with n consecutive crossroads.
        At the intersections, vehicles can not turn. They can only go ahead.

        :param stop_generation_time: The default simulation step when flows will end
        :type stop_generation_time: int
        :param flow_frequency: The default flows frequency (in vehicles/hour/entry)
        :type flow_frequency: int
        :param distribution: The distribution law for all flows. "uniform" inserts vehicles each n simulation steps
        'binomial' inserts vehicle at each simulation step with a given probability. Each of the law respect the flow frequency.
        :type distribution: str
        :return: All flows in a FlowBuilder object.
        :rtype: sumo_experiments.src.components.FlowBuilder
        """
        routes = FlowBuilder()

        # Create v_type
        routes.add_v_type(id='car0')

        # Create flows
        routes.add_flow(id='flow_we', from_edge='edge_wc1', to_edge=f'edge_c{self.nb_intersections}e', end=stop_generation_time, frequency=flow_frequency, v_type='car0', distribution=distribution)
        routes.add_flow(id='flow_ew', from_edge=f'edge_ec{self.nb_intersections}', to_edge='edge_c1w', end=stop_generation_time, frequency=flow_frequency, v_type='car0', distribution=distribution)
        for i in range(1, self.nb_intersections + 1):
            routes.add_flow(id=f'flow_n{i}s{i}', from_edge=f'edge_n{i}c{i}', to_edge=f'edge_c{i}s{i}', end=stop_generation_time, frequency=flow_frequency, v_type='car0', distribution=distribution)
            routes.add_flow(id=f'flow_s{i}n{i}', from_edge=f'edge_s{i}c{i}', to_edge=f'edge_c{i}n{i}', end=stop_generation_time, frequency=flow_frequency, v_type='car0', distribution=distribution)

        return routes


    def generate_flows_all_directions(self,
                                      stop_generation_time,
                                      flow_frequency,
                                      distribution='binomial'):
        """
        Generate flows for a network with n consecutive crossroads.
        At the intersections, vehicles can go to any direction.

        :param stop_generation_time: The default simulation step when flows will end
        :type stop_generation_time: int
        :param flow_frequency: The default flows frequency (in vehicles/hour/entry)
        :type flow_frequency: int
        :param distribution: The distribution law for all flows. "uniform" inserts vehicles each n simulation steps
        'binomial' inserts vehicle at each simulation step with a given probability. Each of the law respect the flow frequency.
        :type distribution: str
        :return: All flows in a FlowBuilder object.
        :rtype: sumo_experiments.src.components.FlowBuilder
        """
        routes = FlowBuilder()

        # Create v_type
        routes.add_v_type(id='car0')

        # Listing all entries
        entries = []
        for i in range(1, self.nb_intersections + 1):
            entries.append(f'edge_n{i}c{i}')
        entries.append(f'edge_ec{self.nb_intersections}')
        for i in range(self.nb_intersections, 0, -1):
            entries.append(f'edge_s{i}c{i}')
        entries.append('edge_wc1')

        # Listing all exits
        exits = []
        for i in range(1, self.nb_intersections + 1):
            exits.append(f'edge_c{i}n{i}')
        exits.append(f'edge_c{self.nb_intersections}e')
        for i in range(self.nb_intersections, 0, -1):
            exits.append(f'edge_c{i}s{i}')
        exits.append('edge_c1w')

        for entry in entries:
            for ex in exits:
                if entries.index(entry) != exits.index(ex):
                    routes.add_flow(id=f'flow_{entry}_{ex}', from_edge=entry, to_edge=ex, end=stop_generation_time, frequency=flow_frequency // (len(exits) - 1), v_type='car0', distribution=distribution)

        return routes


    def generate_flows_with_matrix(self,
                                   period_time,
                                   load_vector,
                                   coeff_matrix,
                                   distribution='binomial'):
        """
        Generate flows for a network with n consecutive crossroads.
        At the intersections, vehicles can go to any direction.
        The vehicle frequency varies over time, following a scheme describe in a load vector and a coefficient matrix.
        The load vector describes, for each period, the frequency of vehicle entering the network.
        The coefficient matrix describes the proportion of load that will follow each route.
        Each scheme of frequency last a time defined in simulation steps.

        :param period_time: The period duration (in simulation steps)
        :type period_time: int
        :param load_vector: The vehicle frequency on the network for each period (in vehs/h)
        :type load_vector: numpy.ndarray
        :param coeff_matrix: The proportion of load charge for each route
        :type coeff_matrix: numpy.ndarray
        :param distribution: The distribution law for all flows. "uniform" inserts vehicles each n simulation steps
        'binomial' inserts vehicle at each simulation step with a given probability. Each of the law respect the flow frequency.
        :type distribution: str
        :return: All flows in a FlowBuilder object.
        :rtype: sumo_experiments.src.components.FlowBuilder
        """
        routes = FlowBuilder()

        # Create v_type
        routes.add_v_type(id='car0')

        # Listing all entries
        entries = []
        for i in range(1, self.nb_intersections + 1):
            entries.append(f'edge_n{i}c{i}')
        entries.append(f'edge_ec{self.nb_intersections}')
        for i in range(self.nb_intersections, 0, -1):
            entries.append(f'edge_s{i}c{i}')
        entries.append('edge_wc1')

        # Listing all exits
        exits = []
        for i in range(1, self.nb_intersections + 1):
            exits.append(f'edge_c{i}n{i}')
        exits.append(f'edge_c{self.nb_intersections}e')
        for i in range(self.nb_intersections, 0, -1):
            exits.append(f'edge_c{i}s{i}')
        exits.append('edge_c1w')

        for i in range(coeff_matrix.shape[1]):

            coeffs_vector = coeff_matrix[:, i]
            flow_values = coeffs_vector * load_vector[i]
            flow_start = period_time * i
            flow_end = period_time * (i+1)

            index_coeff = 0
            for entry in entries:
                index_entry = entries.index(entry)
                for ex in exits:
                    index_exit = exits.index(ex)
                    if index_entry != index_exit:
                        routes.add_flow(id=f'flow_{entry}_{ex}_{i}',
                                        from_edge=entry, to_edge=ex,
                                        begin=flow_start, end=flow_end,
                                        frequency=flow_values[index_coeff],
                                        v_type='car0', distribution=distribution)
                        index_coeff += 1

        return routes



    ### Detectors ###

    def generate_numerical_detectors(self, nb_lanes_main, nb_lanes_minor):
        """
        Generate a DetectorBuilder with a numerical detector for each lane going to an intersection.
        A numerical detector counts and returns the number of vehicles on its scope. In SUMO, a numerical
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.
        :param nb_lanes_main: The number of lanes on the main road. Must be greater than 0.
        :type nb_lanes_main: int
        :param nb_lanes_minor: The number of lanes on the minor roads. Must be greater than 0.
        :type nb_lanes_minor: int
        :return: A DetectorBuilder object.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """
        detectors = DetectorBuilder()
        for n_lane in range(nb_lanes_main):
            detectors.add_lane_area_detector(id=f"n_wc1_{n_lane}", edge="edge_wc1", lane=n_lane, type='numerical')
        for i in range(1, self.nb_intersections + 1):
            for n_lane in range(nb_lanes_minor):
                detectors.add_lane_area_detector(id=f"n_s{i}c{i}_{n_lane}", edge=f"edge_s{i}c{i}", lane=n_lane, type='numerical')
                detectors.add_lane_area_detector(id=f"n_n{i}c{i}_{n_lane}", edge=f"edge_n{i}c{i}", lane=n_lane, type='numerical')
            if i != self.nb_intersections:
                for n_lane in range(nb_lanes_main):
                    detectors.add_lane_area_detector(id=f"n_c{i+1}c{i}_{n_lane}", edge=f"edge_c{i+1}c{i}", lane=n_lane, type='numerical')
                    detectors.add_lane_area_detector(id=f"n_c{i}c{i + 1}_{n_lane}", edge=f"edge_c{i}c{i + 1}", lane=n_lane, type='numerical')
        for n_lane in range(nb_lanes_main):
            detectors.add_lane_area_detector(id=f"n_ec{self.nb_intersections}_{n_lane}", edge=f"edge_ec{self.nb_intersections}", lane=n_lane, type='numerical')
        return detectors

    def generate_exit_detectors(self, nb_lanes_main, nb_lanes_minor):
        """
        Generate a DetectorBuilder with an exit detector for each lane going to an intersection.
        A exit detector counts and returns the number of vehicles on its scope. In SUMO, an exit
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.
        :param nb_lanes_main: The number of lanes on the main road. Must be greater than 0.
        :type nb_lanes_main: int
        :param nb_lanes_minor: The number of lanes on the minor roads. Must be greater than 0.
        :type nb_lanes_minor: int
        :return: A DetectorBuilder object.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """
        detectors = DetectorBuilder()
        for n_lane in range(nb_lanes_main):
            detectors.add_lane_area_detector(id=f"e_c1w_{n_lane}", edge="edge_c1w", lane=n_lane, type='exit')
        for i in range(1, self.nb_intersections + 1):
            for n_lane in range(nb_lanes_minor):
                detectors.add_lane_area_detector(id=f"e_c{i}s{i}_{n_lane}", edge=f"edge_c{i}s{i}", lane=n_lane, type='exit')
                detectors.add_lane_area_detector(id=f"e_c{i}n{i}_{n_lane}", edge=f"edge_c{i}n{i}", lane=n_lane, type='exit')
            if i != self.nb_intersections:
                for n_lane in range(nb_lanes_main):
                    detectors.add_lane_area_detector(id=f"e_c{i}c{i+1}_{n_lane}", edge=f"edge_c{i}c{i+1}", lane=n_lane, type='exit')
                    detectors.add_lane_area_detector(id=f"e_c{i + 1}c{i}_{n_lane}", edge=f"edge_c{i + 1}c{i}", lane=n_lane, type='exit')
        for n_lane in range(nb_lanes_main):
            detectors.add_lane_area_detector(id=f"e_c{self.nb_intersections}e_{n_lane}", edge=f"edge_c{self.nb_intersections}e", lane=n_lane, type='exit')
        return detectors

    def generate_boolean_detectors(self, lane_length, boolean_detector_length, nb_lanes_main, nb_lanes_minor):
        """
        Generate a DetectorBuilder with a boolean detector for each lane going to an intersection.
        A boolean detector returns if a vehicle is on its scope or not. In SUMO, a boolean
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.
        :param nb_lanes_main: The number of lanes on the main road. Must be greater than 0.
        :type nb_lanes_main: int
        :param nb_lanes_minor: The number of lanes on the minor roads. Must be greater than 0.
        :type nb_lanes_minor: int
        :param lane_length: The default length for each lane (in meters)
        :type lane_length: int
        :param boolean_detector_length: The scope size of the detectors (in meters)
        :type boolean_detector_length: int
        :return: A DetectorBuilder object.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """
        detectors = DetectorBuilder()
        for n_lane in range(nb_lanes_main):
            detectors.add_lane_area_detector(id=f"b_wc1_{n_lane}", edge="edge_wc1", lane=n_lane, type='boolean', pos=(lane_length - boolean_detector_length - 17.2))
        for i in range(1, self.nb_intersections + 1):
            for n_lane in range(nb_lanes_minor):
                detectors.add_lane_area_detector(id=f"b_s{i}c{i}_{n_lane}", edge=f"edge_s{i}c{i}", lane=n_lane, type='boolean', pos=(lane_length - boolean_detector_length - 17.2))
                detectors.add_lane_area_detector(id=f"b_n{i}c{i}_{n_lane}", edge=f"edge_n{i}c{i}", lane=n_lane, type='boolean', pos=(lane_length - boolean_detector_length - 17.2))
            if i != self.nb_intersections:
                for n_lane in range(nb_lanes_main):
                    detectors.add_lane_area_detector(id=f"b_c{i + 1}c{i}_{n_lane}", edge=f"edge_c{i + 1}c{i}", lane=n_lane, type='boolean', pos=(lane_length - boolean_detector_length - 24))
                    detectors.add_lane_area_detector(id=f"b_c{i}c{i + 1}_{n_lane}", edge=f"edge_c{i}c{i + 1}", lane=n_lane, type='boolean', pos=(lane_length - boolean_detector_length - 24))
        for n_lane in range(nb_lanes_main):
            detectors.add_lane_area_detector(id=f"b_ec{self.nb_intersections}_{n_lane}", edge=f"edge_ec{self.nb_intersections}", lane=n_lane, type='boolean', pos=(lane_length - boolean_detector_length - 17.2))
        return detectors

    def generate_saturation_detectors(self, detector_length, nb_lanes_main, nb_lanes_minor):
        """
        Generate a DetectorBuilder with a saturation detector for each lane being an exit of an intersection.
        :param nb_lanes_main: The number of lanes on the main road. Must be greater than 0.
        :type nb_lanes_main: int
        :param nb_lanes_minor: The number of lanes on the minor roads. Must be greater than 0.
        :type nb_lanes_minor: int
        :param detector_length: The scope size of the detectors (in meters)
        :type detector_length: int
        :return: A DetectorBuilder object.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """
        detectors = DetectorBuilder()
        for n_lane in range(nb_lanes_main):
            detectors.add_lane_area_detector(id=f"s_wc1_{n_lane}", edge="edge_wc1", lane=n_lane, type='saturation', pos=self.saturation_detectors_position, end_pos=self.saturation_detectors_position+detector_length)
        for i in range(1, self.nb_intersections + 1):
            for n_lane in range(nb_lanes_minor):
                detectors.add_lane_area_detector(id=f"s_s{i}c{i}_{n_lane}", edge=f"edge_s{i}c{i}", lane=n_lane, type='saturation', pos=self.saturation_detectors_position, end_pos=self.saturation_detectors_position+detector_length)
                detectors.add_lane_area_detector(id=f"s_n{i}c{i}_{n_lane}", edge=f"edge_n{i}c{i}", lane=n_lane, type='saturation', pos=self.saturation_detectors_position, end_pos=self.saturation_detectors_position+detector_length)
            if i != self.nb_intersections:
                for n_lane in range(nb_lanes_main):
                    detectors.add_lane_area_detector(id=f"s_c{i + 1}c{i}_{n_lane}", edge=f"edge_c{i + 1}c{i}", lane=n_lane, type='saturation', pos=self.saturation_detectors_position, end_pos=self.saturation_detectors_position+detector_length)
                    detectors.add_lane_area_detector(id=f"s_c{i}c{i + 1}_{n_lane}", edge=f"edge_c{i}c{i + 1}", lane=n_lane, type='saturation', pos=self.saturation_detectors_position, end_pos=self.saturation_detectors_position+detector_length)
        for n_lane in range(nb_lanes_main):
            detectors.add_lane_area_detector(id=f"s_ec{self.nb_intersections}_{n_lane}", edge=f"edge_ec{self.nb_intersections}", lane=n_lane, type='saturation', pos=self.saturation_detectors_position, end_pos=self.saturation_detectors_position+detector_length)
        return detectors

    def generate_all_detectors(self, nb_lanes_main, nb_lanes_minor, lane_length, boolean_detector_length, saturation_detector_length):
        """
        Generate a DetectorBuilder with boolean and numerical detectors for each entry lane of an intersection.
        A boolean detector returns if a vehicle is on its scope or not. In SUMO, a boolean
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.
        A numerical detector counts and returns the number of vehicles on its scope. In SUMO, a numerical
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.
        :param nb_lanes_main: The number of lanes on the main road. Must be greater than 0.
        :type nb_lanes_main: int
        :param nb_lanes_minor: The number of lanes on the minor roads. Must be greater than 0.
        :type nb_lanes_minor: int
        :param lane_length: The default length for each lane (in meters)
        :type lane_length: int
        :param boolean_detector_length: The scope size of the boolean detectors (in meters)
        :type boolean_detector_length: int
        :param saturation_detector_length: The scope size of the saturation detectors (in meters)
        :type saturation_detector_length: int
        :return: A DetectorBuilder object.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """
        detectors = DetectorBuilder()
        detectors.laneAreaDetectors.update(self.generate_boolean_detectors(lane_length, boolean_detector_length, nb_lanes_main, nb_lanes_minor).laneAreaDetectors)
        detectors.laneAreaDetectors.update(self.generate_numerical_detectors(nb_lanes_main, nb_lanes_minor).laneAreaDetectors)
        detectors.laneAreaDetectors.update(self.generate_saturation_detectors(saturation_detector_length, nb_lanes_main, nb_lanes_minor).laneAreaDetectors)
        detectors.laneAreaDetectors.update(self.generate_exit_detectors(nb_lanes_main, nb_lanes_minor).laneAreaDetectors)
        return detectors


    def create_TLS_DETECTORS_2_phases(self, nb_lanes_main, nb_lanes_minor):
        """
        Creates the self.TLS_DETECTORS variable.
        :param nb_lanes_main: The number of lanes on the main road. Must be greater than 0.
        :type nb_lanes_main: int
        :param nb_lanes_minor: The number of lanes on the minor roads. Must be greater than 0.
        :type nb_lanes_minor: int
        :return: Nothing
        :rtype: None
        """
        self.TL_IDS = []
        self.TLS_DETECTORS = {}
        detectors = {
            0: {
                'boolean': [f'b_wc1_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'b_c2c1_{n_lane}' for n_lane in range(nb_lanes_main)],
                'saturation': [f's_wc1_{n_lane}' for n_lane in range(nb_lanes_main)] + [f's_c2c1_{n_lane}' for n_lane in range(nb_lanes_main)],
                'numerical': [f'n_wc1_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'n_c2c1_{n_lane}' for n_lane in range(nb_lanes_main)],
                'exit': [f'e_c1w_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c1c2_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c1n1_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c1s1_{n_lane}' for n_lane in range(nb_lanes_minor)]
            },
            2: {
                'boolean': [f'b_n1c1_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'b_s1c1_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'saturation': [f's_n1c1_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f's_s1c1_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'numerical': [f'n_n1c1_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'n_s1c1_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'exit': [f'e_c1n1_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c1s1_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c1w_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c1c2_{n_lane}' for n_lane in range(nb_lanes_main)]
            },
        }
        self.TLS_DETECTORS['c1'] = detectors
        self.TL_IDS.append(f'c1')
        for i in range(2, self.nb_intersections):
            detectors = {
                0: {
                    'boolean': [f'b_c{i-1}c{i}_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'b_c{i+1}c{i}_{n_lane}' for n_lane in range(nb_lanes_main)],
                    'saturation': [f's_c{i-1}c{i}_{n_lane}' for n_lane in range(nb_lanes_main)] + [f's_c{i+1}c{i}_{n_lane}' for n_lane in range(nb_lanes_main)],
                    'numerical': [f'n_c{i-1}c{i}_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'n_c{i+1}c{i}_{n_lane}' for n_lane in range(nb_lanes_main)],
                    'exit': [f'e_c{i}c{i-1}_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c{i}c{i+1}_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_n{i}c{i}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_s{i}c{i}_{n_lane}' for n_lane in range(nb_lanes_minor)]
                },
                2: {
                    'boolean': [f'b_n{i}c{i}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'b_s{i}c{i}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                    'saturation': [f's_n{i}c{i}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f's_s{i}c{i}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                    'numerical': [f'n_c{i}n{i}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'n_c{i}s{i}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                    'exit': [f'e_n{i}c{i}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_s{i}c{i}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c{i}c{i-1}_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c{i}c{i+1}_{n_lane}' for n_lane in range(nb_lanes_main)]
                },
            }
            self.TLS_DETECTORS[f'c{i}'] = detectors
            self.TL_IDS.append(f'c{i}')
        detectors = {
            0: {
                'boolean': [f'b_c{self.nb_intersections - 1}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'b_ec{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_main)],
                'saturation': [f's_c{self.nb_intersections - 1}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_main)] + [f's_ec{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_main)],
                'numerical': [f'n_c{self.nb_intersections - 1}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'n_ec{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_main)],
                'exit': [f'e_c{self.nb_intersections}c{self.nb_intersections - 1}_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c{self.nb_intersections}e_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c{self.nb_intersections}n{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c{self.nb_intersections}s{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)]
            },
            2: {
                'boolean': [f'b_n{self.nb_intersections}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'b_s{self.nb_intersections}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'saturation': [f's_n{self.nb_intersections}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f's_s{self.nb_intersections}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'numerical': [f'n_n{self.nb_intersections}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'n_s{self.nb_intersections}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'exit': [f'e_c{self.nb_intersections}n{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c{self.nb_intersections}s{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c{self.nb_intersections}c{self.nb_intersections - 1}_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c{self.nb_intersections}e_{n_lane}' for n_lane in range(nb_lanes_main)]
            },
        }
        self.TLS_DETECTORS[f'c{self.nb_intersections}'] = detectors
        self.TL_IDS.append(f'c{self.nb_intersections}')


    def create_TLS_DETECTORS_4_phases(self, nb_lanes_main, nb_lanes_minor):
        """
        Creates the self.TLS_DETECTORS variable.
        :param nb_lanes_main: The number of lanes on the main road. Must be greater than 0.
        :type nb_lanes_main: int
        :param nb_lanes_minor: The number of lanes on the minor roads. Must be greater than 0.
        :type nb_lanes_minor: int
        :return: Nothing
        :rtype: None
        """
        self.TL_IDS = []
        self.TLS_DETECTORS = {}
        detectors = {
            0: {
                'boolean': [f'b_n1c1_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'saturation': [f's_n1c1_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'numerical': [f'n_n1c1_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'exit': [f'e_c1c2_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c1s1_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c1w_{n_lane}' for n_lane in range(nb_lanes_main)]
            },
            2: {
                'boolean':[f'b_c2c1_{n_lane}' for n_lane in range(nb_lanes_main)],
                'saturation':[f's_c2c1_{n_lane}' for n_lane in range(nb_lanes_main)],
                'numerical':[f'n_c2c1_{n_lane}' for n_lane in range(nb_lanes_main)],
                'exit': [f'e_c1n1_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c1s1_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c1w_{n_lane}' for n_lane in range(nb_lanes_main)]
            },
            4: {
                'boolean':[f'b_s1c1_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'saturation':[f's_s1c1_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'numerical':[f'n_s1c1_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'exit': [f'e_c1n1_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c1c2_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c1w_{n_lane}' for n_lane in range(nb_lanes_main)]
            },
            6: {
                'boolean': [f'b_wc1_{n_lane}' for n_lane in range(nb_lanes_main)],
                'saturation': [f's_wc1_{n_lane}' for n_lane in range(nb_lanes_main)],
                'numerical': [f'n_wc1_{n_lane}' for n_lane in range(nb_lanes_main)],
                'exit': [f'e_c1n1_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c1c2_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c1s1_{n_lane}' for n_lane in range(nb_lanes_minor)]
            },
        }
        self.TLS_DETECTORS['c1'] = detectors
        self.TL_IDS.append(f'c1')
        for i in range(2, self.nb_intersections):
            detectors = {
                0: {
                    'boolean': [f'b_n{i}c{i}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                    'saturation': [f's_n{i}c{i}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                    'numerical': [f'n_n{i}c{i}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                    'exit': [f'e_c{i}c{i + 1}_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c{i}s{i}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c{i}c{i - 1}_{n_lane}' for n_lane in range(nb_lanes_main)]
                },
                2: {
                    'boolean': [f'b_c{i + 1}c{i}_{n_lane}' for n_lane in range(nb_lanes_main)],
                    'saturation': [f's_c{i + 1}c{i}_{n_lane}' for n_lane in range(nb_lanes_main)],
                    'numerical': [f'n_c{i + 1}c{i}_{n_lane}' for n_lane in range(nb_lanes_main)],
                    'exit': [f'e_c{i}n{i}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c{i}s{i}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c{i}c{i - 1}_{n_lane}' for n_lane in range(nb_lanes_main)]
                },
                4: {
                    'boolean': [f'b_s{i}c{i}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                    'saturation': [f's_s{i}c{i}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                    'numerical': [f'n_s{i}c{i}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                    'exit': [f'e_c{i}n{i}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c{i}c{i + 1}_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c{i}c{i - 1}_{n_lane}' for n_lane in range(nb_lanes_main)]
                },
                6: {
                    'boolean': [f'b_c{i - 1}c{i}_{n_lane}' for n_lane in range(nb_lanes_main)],
                    'saturation': [f's_c{i - 1}c{i}_{n_lane}' for n_lane in range(nb_lanes_main)],
                    'numerical': [f'n_c{i - 1}c{i}_{n_lane}' for n_lane in range(nb_lanes_main)],
                    'exit': [f'e_c{i}n{i}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c{i}c{i + 1}_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c{i}s{i}_{n_lane}' for n_lane in range(nb_lanes_minor)]
                },
            }
            self.TLS_DETECTORS[f'c{i}'] = detectors
            self.TL_IDS.append(f'c{i}')
        detectors = {
            0: {
                'boolean': [f'b_n{self.nb_intersections}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'saturation': [f's_n{self.nb_intersections}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'numerical': [f'n_n{self.nb_intersections}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'exit':  [f'e_c{self.nb_intersections}e_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c{self.nb_intersections}s{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c{self.nb_intersections}c{self.nb_intersections - 1}_{n_lane}' for n_lane in range(nb_lanes_main)]
            },
            2: {
                'boolean': [f'b_ec{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_main)],
                'saturation': [f's_ec{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_main)],
                'numerical': [f'n_ec{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_main)],
                'exit':  [f'e_c{self.nb_intersections}n{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c{self.nb_intersections}s{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c{self.nb_intersections}c{self.nb_intersections - 1}_{n_lane}' for n_lane in range(nb_lanes_main)]
            },
            4: {
                'boolean': [f'b_s{self.nb_intersections}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'saturation': [f's_s{self.nb_intersections}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'numerical': [f'n_s{self.nb_intersections}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)],
                'exit':  [f'e_c{self.nb_intersections}n{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c{self.nb_intersections}e_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c{self.nb_intersections}c{self.nb_intersections - 1}_{n_lane}' for n_lane in range(nb_lanes_main)]
            },
            6: {
                'boolean': [f'b_c{self.nb_intersections - 1}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_main)],
                'saturation': [f's_c{self.nb_intersections - 1}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_main)],
                'numerical': [f'n_c{self.nb_intersections - 1}c{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_main)],
                'exit': [f'e_c{self.nb_intersections}n{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)] + [f'e_c{self.nb_intersections}e_{n_lane}' for n_lane in range(nb_lanes_main)] + [f'e_c{self.nb_intersections}s{self.nb_intersections}_{n_lane}' for n_lane in range(nb_lanes_minor)]
            },
        }
        self.TLS_DETECTORS[f'c{self.nb_intersections}'] = detectors
        self.TL_IDS.append(f'c{self.nb_intersections}')
