import warnings

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

        :return: An empty DetectorBuilder object.
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
        :return: An empty DetectorBuilder object.
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

    def generate_all_detectors(self, boolean_detector_length):
        """
        Generate a DetectorBuilder with boolean and numerical detectors for each entry lane of an intersection.
        A boolean detector returns if a vehicle is on its scope or not. In SUMO, a boolean
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.
        A numerical detector counts and returns the number of vehicles on its scope. In SUMO, a numerical
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.
        :param boolean_detector_length: The scope size of the detectors (in meters)
        :type boolean_detector_length: int
        :return: The numerical detectors.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """
        detectors = DetectorBuilder()
        detectors.laneAreaDetectors.update(self.generate_boolean_detectors(boolean_detector_length).laneAreaDetectors)
        detectors.laneAreaDetectors.update(self.generate_numerical_detectors().laneAreaDetectors)
        return detectors




    ### Strategies ###

    def boolean_detection(self, config):
        """
        To be used with a network equipped with boolean detectors.

        Before running the simulation, three variables must be set in config :
        - The minimum duration for a traffic light phase (default 30 seconds)
        - The maximum duration for a traffic light phase (default 60 seconds)
        - The simulation duration (default 1000 simulation steps)
        - "nb_intersections" (int >= 2) : The number of intersections of the network

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
        - "nb_intersections" (int >= 2) : The number of intersections of the network

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        """

        nb_intersections = config['nb_intersections']

        if 'cooldown_step' not in config:
            config['cooldown_step'] = np.zeros(nb_intersections)

        # Select parameters
        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        cooldown_step = config['cooldown_step']

        for i in range(len(cooldown_step)):
            if cooldown_step[i] > min_duration_tl:
                id_tl = i + 1
                if traci.trafficlight.getPhase(f"c{id_tl}") == self.GREEN_LIGHT_HORIZONTAL:
                    quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(f"n{id_tl}c{id_tl}") >= 1 or traci.lanearea.getLastStepVehicleNumber(f"s{id_tl}c{id_tl}") >= 1)
                    if id_tl == 1:
                        autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber("wc1") == 0 and traci.lanearea.getLastStepVehicleNumber("c2c1") == 0)
                    elif id_tl == nb_intersections:
                        autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(f"c{nb_intersections-1}c{nb_intersections}") == 0 and traci.lanearea.getLastStepVehicleNumber(f"ec{nb_intersections}") == 0)
                    else:
                        autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(f"c{id_tl-1}c{id_tl}") == 0 and traci.lanearea.getLastStepVehicleNumber(f"c{id_tl+1}c{id_tl}") == 0)
                    if (quelqun_en_attente and autre_voie_vide) or cooldown_step[i] > max_duration_tl:
                        traci.trafficlight.setPhase(f"c{id_tl}", self.GREEN_LIGHT_HORIZONTAL + 1)  # Passage au orange
                        cooldown_step[i] = 0

                elif traci.trafficlight.getPhase(f"c{id_tl}") == self.GREEN_LIGHT_VERTICAL:
                    if id_tl == 1:
                        quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber("wc1") >= 1 and traci.lanearea.getLastStepVehicleNumber("c2c1") >= 1)
                    elif id_tl == nb_intersections:
                        quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(f"c{nb_intersections - 1}c{nb_intersections}") >= 1 and traci.lanearea.getLastStepVehicleNumber(f"ec{nb_intersections}") >= 1)
                    else:
                        quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(f"c{id_tl - 1}c{id_tl}") >= 1 and traci.lanearea.getLastStepVehicleNumber(f"c{id_tl + 1}c{id_tl}") >= 1)
                    autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(f"n{id_tl}c{id_tl}") == 0 or traci.lanearea.getLastStepVehicleNumber(f"s{id_tl}c{id_tl}") == 0)
                    if (quelqun_en_attente and autre_voie_vide) or cooldown_step[i] > max_duration_tl:
                        traci.trafficlight.setPhase(f"c{id_tl}", self.GREEN_LIGHT_VERTICAL + 1)
                        cooldown_step[i] = 0

        cooldown_step += 1
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
        - "nb_intersections" (int >= 2) : The number of intersections of the network

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        """

        nb_intersections = config['nb_intersections']

        if 'cooldown_step' not in config:
            config['cooldown_step'] = np.zeros(nb_intersections)

        # Select parameters
        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        vehicle_threshold = config["vehicle_threshold"]
        cooldown_step = config['cooldown_step']

        for i in range(len(cooldown_step)):
            if cooldown_step[i] > min_duration_tl:
                id_tl = i + 1
                if traci.trafficlight.getPhase(f"c{id_tl}") == self.GREEN_LIGHT_VERTICAL:
                    if id_tl == 1:
                        threshold_passed = (traci.lanearea.getLastStepVehicleNumber("wc1") >= vehicle_threshold
                                            or traci.lanearea.getLastStepVehicleNumber("c2c1") >= vehicle_threshold)
                    elif id_tl == nb_intersections:
                        threshold_passed = (traci.lanearea.getLastStepVehicleNumber(f"ec{nb_intersections}") >= vehicle_threshold
                                            or traci.lanearea.getLastStepVehicleNumber(f"c{nb_intersections - 1}c{nb_intersections}") >= vehicle_threshold)
                    else:
                        threshold_passed = (traci.lanearea.getLastStepVehicleNumber(f"c{id_tl-1}c{id_tl}") >= vehicle_threshold
                                            or traci.lanearea.getLastStepVehicleNumber(f"c{id_tl + 1}c{id_tl}") >= vehicle_threshold)
                    if threshold_passed or cooldown_step[i] > max_duration_tl:
                        traci.trafficlight.setPhase(f"c{id_tl}", self.GREEN_LIGHT_VERTICAL + 1)  # Passage au orange
                        cooldown_step[i] = 0

                elif traci.trafficlight.getPhase(f"c{id_tl}") == self.GREEN_LIGHT_HORIZONTAL:
                    threshold_passed = (traci.lanearea.getLastStepVehicleNumber(f"s{id_tl}c{id_tl}") >= vehicle_threshold
                                        or traci.lanearea.getLastStepVehicleNumber(f"n{id_tl}c{id_tl}") >= vehicle_threshold)
                    if threshold_passed or cooldown_step[i] > max_duration_tl:
                        traci.trafficlight.setPhase(f"c{id_tl}", self.GREEN_LIGHT_HORIZONTAL + 1)
                        cooldown_step[i] = 0

        cooldown_step += 1
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
        - "nb_intersections" (int >= 2) : The number of intersections of the network

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        """

        nb_intersections = config['nb_intersections']

        if 'cooldown_step' not in config:
            config['cooldown_step'] = np.zeros(nb_intersections)

        # Select parameters
        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        vehicle_threshold = config["vehicle_threshold"]
        cooldown_step = config['cooldown_step']

        for i in range(len(cooldown_step)):
            if cooldown_step[i] > min_duration_tl:
                id_tl = i + 1
                if traci.trafficlight.getPhase(f"c{id_tl}") == self.GREEN_LIGHT_VERTICAL:
                    if id_tl == 1:
                        threshold_passed = (traci.lanearea.getJamLengthVehicle("wc1") >= vehicle_threshold
                                            or traci.lanearea.getJamLengthVehicle("c2c1") >= vehicle_threshold)
                    elif id_tl == nb_intersections:
                        threshold_passed = (traci.lanearea.getJamLengthVehicle(f"ec{nb_intersections}") >= vehicle_threshold
                                            or traci.lanearea.getJamLengthVehicle(f"c{nb_intersections - 1}c{nb_intersections}") >= vehicle_threshold)
                    else:
                        threshold_passed = (traci.lanearea.getJamLengthVehicle(f"c{id_tl - 1}c{id_tl}") >= vehicle_threshold
                                            or traci.lanearea.getJamLengthVehicle(f"c{id_tl + 1}c{id_tl}") >= vehicle_threshold)
                    if threshold_passed or cooldown_step[i] > max_duration_tl:
                        traci.trafficlight.setPhase(f"c{id_tl}", self.GREEN_LIGHT_VERTICAL + 1)  # Passage au orange
                        cooldown_step[i] = 0

                elif traci.trafficlight.getPhase(f"c{id_tl}") == self.GREEN_LIGHT_HORIZONTAL:
                    threshold_passed = (traci.lanearea.getJamLengthVehicle(f"s{id_tl}c{id_tl}") >= vehicle_threshold
                                        or traci.lanearea.getJamLengthVehicle(f"n{id_tl}c{id_tl}") >= vehicle_threshold)
                    if threshold_passed or cooldown_step[i] > max_duration_tl:
                        traci.trafficlight.setPhase(f"c{id_tl}", self.GREEN_LIGHT_HORIZONTAL + 1)
                        cooldown_step[i] = 0

        cooldown_step += 1
        config['cooldown_step'] = cooldown_step
        return config