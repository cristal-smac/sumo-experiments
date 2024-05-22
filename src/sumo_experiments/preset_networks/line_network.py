import numpy as np

from sumo_experiments.components import InfrastructureBuilder, FlowBuilder, DetectorBuilder
import traci


class LineNetwork:
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

    def __init__(self, nb_intersections):
        """
        Init of class.
        :param nb_intersections: The number of intersections for the network
        :type nb_intersections: int
        """
        if nb_intersections < 2:
            raise ValueError('nb_intersections must be 2 or more.')
        self.nb_intersections = nb_intersections

    ### Networks ###

    def generate_infrastructures(self,
                                 lane_length,
                                 green_time,
                                 yellow_time,
                                 max_speed):
        """
        Generate the sumo infrastructures for a network with n consecutive crossroads.

        :param lane_length: The default length for each lane (in meters)
        :type lane_length: int
        :param green_time: The default green time for each phase (in seconds)
        :type green_time: int
        :param yellow_time: The default yellow time for each phase (in seconds)
        :type yellow_time: int
        :param max_speed: The max speed on each lane (in km/h)
        :type max_speed: int
        :return: All infrastructures in a NetworkBuilder object.
        :rtype: sumo_experiments.src.components.NetworkBuilder
        :raise: ValueError if nb_intersections is under 2
        """
        net = InfrastructureBuilder()
        self.lane_length = lane_length

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
        net.add_edge(id='edge_wc1', from_node='w', to_node='c1', edge_type='default')
        net.add_edge(id='edge_c1w', from_node='c1', to_node='w', edge_type='default')
        for i in range(1, self.nb_intersections):
            net.add_edge(id=f'edge_n{i}c{i}', from_node=f'n{i}', to_node=f'c{i}', edge_type='default')
            net.add_edge(id=f'edge_c{i}n{i}', from_node=f'c{i}', to_node=f'n{i}', edge_type='default')
            net.add_edge(id=f'edge_s{i}c{i}', from_node=f's{i}', to_node=f'c{i}', edge_type='default')
            net.add_edge(id=f'edge_c{i}s{i}', from_node=f'c{i}', to_node=f's{i}', edge_type='default')
            net.add_edge(id=f'edge_c{i}c{i + 1}', from_node=f'c{i}', to_node=f'c{i + 1}', edge_type='default')
            net.add_edge(id=f'edge_c{i + 1}c{i}', from_node=f'c{i + 1}', to_node=f'c{i}', edge_type='default')
        id_last_inter = self.nb_intersections
        net.add_edge(id=f'edge_n{id_last_inter}c{id_last_inter}', from_node=f'n{id_last_inter}', to_node=f'c{id_last_inter}', edge_type='default')
        net.add_edge(id=f'edge_c{id_last_inter}n{id_last_inter}', from_node=f'c{id_last_inter}', to_node=f'n{id_last_inter}', edge_type='default')
        net.add_edge(id=f'edge_s{id_last_inter}c{id_last_inter}', from_node=f's{id_last_inter}', to_node=f'c{id_last_inter}', edge_type='default')
        net.add_edge(id=f'edge_c{id_last_inter}s{id_last_inter}', from_node=f'c{id_last_inter}', to_node=f's{id_last_inter}', edge_type='default')
        net.add_edge(id=f'edge_c{id_last_inter}e', from_node=f'c{id_last_inter}', to_node=f'e', edge_type='default')
        net.add_edge(id=f'edge_ec{id_last_inter}', from_node=f'e', to_node=f'c{id_last_inter}', edge_type='default')

        # Create connections
        net.add_connection(from_edge='edge_wc1', to_edge='edge_c1c2')
        net.add_connection(from_edge='edge_wc1', to_edge='edge_c1n1')
        net.add_connection(from_edge='edge_wc1', to_edge='edge_c1s1')
        net.add_connection(from_edge='edge_n1c1', to_edge='edge_c1w')
        net.add_connection(from_edge='edge_n1c1', to_edge='edge_c1s1')
        net.add_connection(from_edge='edge_n1c1', to_edge='edge_c1c2')
        net.add_connection(from_edge='edge_s1c1', to_edge='edge_c1w')
        net.add_connection(from_edge='edge_s1c1', to_edge='edge_c1n1')
        net.add_connection(from_edge='edge_s1c1', to_edge='edge_c1c2')
        net.add_connection(from_edge='edge_c2c1', to_edge='edge_c1w')
        net.add_connection(from_edge='edge_c2c1', to_edge='edge_c1n1')
        net.add_connection(from_edge='edge_c2c1', to_edge='edge_c1s1')
        for i in range(1, self.nb_intersections - 1):
            id = i + 1
            net.add_connection(from_edge=f'edge_c{id-1}c{id}', to_edge=f'edge_c{id}c{id+1}')
            net.add_connection(from_edge=f'edge_c{id-1}c{id}', to_edge=f'edge_c{id}n{id}')
            net.add_connection(from_edge=f'edge_c{id-1}c{id}', to_edge=f'edge_c{id}s{id}')
            net.add_connection(from_edge=f'edge_n{id}c{id}', to_edge=f'edge_c{id}c{id-1}')
            net.add_connection(from_edge=f'edge_n{id}c{id}', to_edge=f'edge_c{id}s{id}')
            net.add_connection(from_edge=f'edge_n{id}c{id}', to_edge=f'edge_c{id}c{id+1}')
            net.add_connection(from_edge=f'edge_s{id}c{id}', to_edge=f'edge_c{id}c{id-1}')
            net.add_connection(from_edge=f'edge_s{id}c{id}', to_edge=f'edge_c{id}n{id}')
            net.add_connection(from_edge=f'edge_s{id}c{id}', to_edge=f'edge_c{id}c{id+1}')
            net.add_connection(from_edge=f'edge_c{id+1}c{id}', to_edge=f'edge_c{id}c{id-1}')
            net.add_connection(from_edge=f'edge_c{id+1}c{id}', to_edge=f'edge_c{id}n{id}')
            net.add_connection(from_edge=f'edge_c{id+1}c{id}', to_edge=f'edge_c{id}s{id}')
        id_last_inter = self.nb_intersections
        net.add_connection(from_edge=f'edge_c{id_last_inter - 1}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}e')
        net.add_connection(from_edge=f'edge_c{id_last_inter - 1}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}n{id_last_inter}')
        net.add_connection(from_edge=f'edge_c{id_last_inter - 1}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}s{id_last_inter}')
        net.add_connection(from_edge=f'edge_n{id_last_inter}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}c{id_last_inter - 1}')
        net.add_connection(from_edge=f'edge_n{id_last_inter}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}s{id_last_inter}')
        net.add_connection(from_edge=f'edge_n{id_last_inter}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}e')
        net.add_connection(from_edge=f'edge_s{id_last_inter}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}c{id_last_inter - 1}')
        net.add_connection(from_edge=f'edge_s{id_last_inter}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}n{id_last_inter}')
        net.add_connection(from_edge=f'edge_s{id_last_inter}c{id_last_inter}', to_edge=f'edge_c{id_last_inter}e')
        net.add_connection(from_edge=f'edge_ec{id_last_inter}', to_edge=f'edge_c{id_last_inter}c{id_last_inter - 1}')
        net.add_connection(from_edge=f'edge_ec{id_last_inter}', to_edge=f'edge_c{id_last_inter}n{id_last_inter}')
        net.add_connection(from_edge=f'edge_ec{id_last_inter}', to_edge=f'edge_c{id_last_inter}s{id_last_inter}')

        # Create traffic lights programs
        for i in range(self.nb_intersections):
            i = i + 1
            net.add_traffic_light_program(id=f'c{i}',
                                          phases=[{'duration': green_time, 'state': 'rrrGGGrrrGGG'},
                                                  {'duration': yellow_time, 'state': 'rrryyyrrryyy'},
                                                  {'duration': green_time, 'state': 'GGGrrrGGGrrr'},
                                                  {'duration': yellow_time, 'state': 'yyyrrryyyrrr'}])

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

    def generate_numerical_detectors(self):
        """
        Generate a DetectorBuilder with a numerical detector for each lane going to an intersection.
        A numerical detector counts and returns the number of vehicles on its scope. In SUMO, a numerical
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.

        :return: A DetectorBuilder object.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """
        detectors = DetectorBuilder()

        detectors.add_lane_area_detector(id="n_wc1", edge="edge_wc1", lane=0, type='numerical')
        for i in range(1, self.nb_intersections + 1):
            detectors.add_lane_area_detector(id=f"n_s{i}c{i}", edge=f"edge_s{i}c{i}", lane=0, type='numerical')
            detectors.add_lane_area_detector(id=f"n_n{i}c{i}", edge=f"edge_n{i}c{i}", lane=0, type='numerical')
            if i != self.nb_intersections:
                detectors.add_lane_area_detector(id=f"n_c{i+1}c{i}", edge=f"edge_c{i+1}c{i}", lane=0, type='numerical')
                detectors.add_lane_area_detector(id=f"n_c{i}c{i + 1}", edge=f"edge_c{i}c{i + 1}", lane=0, type='numerical')
        detectors.add_lane_area_detector(id=f"n_ec{self.nb_intersections}", edge=f"edge_ec{self.nb_intersections}", lane=0, type='numerical')

        return detectors

    def generate_boolean_detectors(self, boolean_detector_length):
        """
        Generate a DetectorBuilder with a boolean detector for each lane going to an intersection.
        A boolean detector returns if a vehicle is on its scope or not. In SUMO, a boolean
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.
        :param boolean_detector_length: The scope size of the detectors (in meters)
        :type boolean_detector_length: int
        :return: A DetectorBuilder object.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """
        detectors = DetectorBuilder()
        detectors.add_lane_area_detector(id="b_wc1", edge="edge_wc1", lane=0, type='boolean', pos=(self.lane_length - boolean_detector_length - 7.2))
        for i in range(1, self.nb_intersections + 1):
            detectors.add_lane_area_detector(id=f"b_s{i}c{i}", edge=f"edge_s{i}c{i}", lane=0, type='boolean', pos=(self.lane_length - boolean_detector_length - 7.2))
            detectors.add_lane_area_detector(id=f"b_n{i}c{i}", edge=f"edge_n{i}c{i}", lane=0, type='boolean', pos=(self.lane_length - boolean_detector_length - 7.2))
            if i != self.nb_intersections:
                detectors.add_lane_area_detector(id=f"b_c{i + 1}c{i}", edge=f"edge_c{i + 1}c{i}", lane=0, type='boolean', pos=(self.lane_length - boolean_detector_length - 14))
                detectors.add_lane_area_detector(id=f"b_c{i}c{i + 1}", edge=f"edge_c{i}c{i + 1}", lane=0, type='boolean', pos=(self.lane_length - boolean_detector_length - 14))
        detectors.add_lane_area_detector(id=f"b_ec{self.nb_intersections}", edge=f"edge_ec{self.nb_intersections}", lane=0, type='boolean', pos=(self.lane_length - boolean_detector_length - 7.2))
        return detectors

    def generate_saturation_detectors(self, detector_length):
        """
        Generate a DetectorBuilder with a saturation detector for each lane beeing an exit of an intersection.
        :param detector_length: The scope size of the detectors (in meters)
        :type detector_length: int
        :return: A DetectorBuilder object.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """
        detectors = DetectorBuilder()
        for i in range(1, self.nb_intersections + 1):
            if i != self.nb_intersections:
                detectors.add_lane_area_detector(id=f"s_c{i + 1}c{i}", edge=f"edge_c{i + 1}c{i}", lane=0, type='saturation', pos=0, end_pos=detector_length)
                detectors.add_lane_area_detector(id=f"s_c{i}c{i + 1}", edge=f"edge_c{i}c{i + 1}", lane=0, type='saturation', pos=0, end_pos=detector_length)
        return detectors

    def generate_all_detectors(self, boolean_detector_length, saturation_detector_length):
        """
        Generate a DetectorBuilder with boolean and numerical detectors for each entry lane of an intersection.
        A boolean detector returns if a vehicle is on its scope or not. In SUMO, a boolean
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.
        A numerical detector counts and returns the number of vehicles on its scope. In SUMO, a numerical
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.
        :param boolean_detector_length: The scope size of the boolean detectors (in meters)
        :type boolean_detector_length: int
        :param saturation_detector_length: The scope size of the saturation detectors (in meters)
        :type saturation_detector_length: int
        :return: A DetectorBuilder object.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """
        detectors = DetectorBuilder()
        detectors.laneAreaDetectors.update(self.generate_boolean_detectors(boolean_detector_length).laneAreaDetectors)
        detectors.laneAreaDetectors.update(self.generate_numerical_detectors().laneAreaDetectors)
        detectors.laneAreaDetectors.update(self.generate_saturation_detectors(saturation_detector_length).laneAreaDetectors)
        return detectors
