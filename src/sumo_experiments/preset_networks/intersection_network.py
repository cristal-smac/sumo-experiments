import os
import random
from sumo_experiments.components import InfrastructureBuilder, FlowBuilder, DetectorBuilder

from sumo_experiments.preset_networks import ArtificialNetwork


class IntersectionNetwork(ArtificialNetwork):
    """
    The IntersectionNetwork class contains a set of functions for creating a SUMO network containing a single intersection.
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

    def __init__(self,
                 lane_length=100,
                 max_speed=50,
                 flow_type='all_directions',
                 stop_generation_time=3600,
                 flow_frequency=100,
                 distribution='binomial',
                 period_time=None,
                 load_vector=None,
                 coeff_matrix=None,
                 boolean_detectors_length=20,
                 saturation_detectors_length=20):
        """
        Init of class.
        :param lane_length: The length of all edges (in meters)
        :type lane_length: int
        :param max_speed: The max speed on each lane (in km/h)
        :type max_speed: int
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
        super().__init__(f'intersection_network_{random.randint(1, 1000000)}')
        # Create infrastructures
        infrastructures = self.generate_infrastructures(lane_length, max_speed)
        # Create flows
        if flow_type == 'only_ahead':
            self.generate_flows_only_ahead(stop_generation_time, flow_frequency, distribution)
        elif flow_type == 'all_directions':
            flows = self.generate_flows_all_directions(stop_generation_time, flow_frequency, distribution)
        elif flow_type == 'matrix':
            if load_vector != None and period_time != None and coeff_matrix != None:
                flows = self.generate_flows_with_matrix(period_time, load_vector, coeff_matrix)
            else:
                raise ValueError("load_vector and coeff_matrix ad period_time must be provided when flow_type is 'matrix'.")
        else:
            raise ValueError("flow_type must be 'only_ahead' or 'all_directions' or 'matrix'.")
        # Creates detectors
        detectors = self.generate_all_detectors(lane_length, boolean_detectors_length, saturation_detectors_length)
        self.create_TLS_DETECTORS()
        # Building files
        infrastructures.build(self.file_names)
        flows.build(self.file_names)
        detectors.build(self.file_names)
        # Using netconvert to create the network
        cmd = f'$SUMO_HOME/bin/netconvert -n {self.file_names["nodes"]} -e {self.file_names["edges"]} -x {self.file_names["connections"]} -i {self.file_names["trafic_light_programs"]} -t {self.file_names["types"]} -o {self.file_names["network"]} --no-warnings'
        os.system(cmd)





    ### Network ###

    def generate_infrastructures(self,
                                 lane_length,
                                 max_speed
                                 ):
        """
        Generate the sumo infrastructures for a network with a single crossroad, joining 4 roads.
        Each road has a lane going to the crossroad, and another going to an exit of the network.
        The crossroad is managed by traffic lights on each entry.

        :param lane_length: The default length for each lane (in meters)
        :type lane_length: int
        :param max_speed: The max speed on each lane (in km/h)
        :type max_speed: int
        :return: All infrastructures in a NetworkBuilder object.
        :rtype: sumo_experiments.src.components.NetworkBuilder
        """
        # Select parameters

        # Instantiation of the infrastructures builder
        infrastructures = InfrastructureBuilder()

        # Create nodes
        infrastructures.add_node(id='c', x=0, y=0, type='traffic_light', tl_program='c')    # Central node
        infrastructures.add_node(id='w', x=-lane_length, y=0)        # West node
        infrastructures.add_node(id='e', x=lane_length, y=0)     # East node
        infrastructures.add_node(id='s', x=0, y=-lane_length)        # South node
        infrastructures.add_node(id='n', x=0, y=lane_length)     # North node

        # Create edge types
        infrastructures.add_edge_type(id='default', params={'numLanes': '1', 'speed': max_speed})

        # Create edges to center
        infrastructures.add_edge(id='edge_wc', from_node='w', to_node='c', edge_type='default')
        infrastructures.add_edge(id='edge_sc', from_node='s', to_node='c', edge_type='default')
        infrastructures.add_edge(id='edge_ec', from_node='e', to_node='c', edge_type='default')
        infrastructures.add_edge(id='edge_nc', from_node='n', to_node='c', edge_type='default')

        # Create edges to exits
        infrastructures.add_edge(id='edge_ce', from_node='c', to_node='e', edge_type='default')
        infrastructures.add_edge(id='edge_cn', from_node='c', to_node='n', edge_type='default')
        infrastructures.add_edge(id='edge_cw', from_node='c', to_node='w', edge_type='default')
        infrastructures.add_edge(id='edge_cs', from_node='c', to_node='s', edge_type='default')


        # Create connections between edges
        # From west
        infrastructures.add_connection(from_edge='edge_wc', to_edge='edge_ce')
        infrastructures.add_connection(from_edge='edge_wc', to_edge='edge_cs')
        infrastructures.add_connection(from_edge='edge_wc', to_edge='edge_cn')
        # From north
        infrastructures.add_connection(from_edge='edge_nc', to_edge='edge_cs')
        infrastructures.add_connection(from_edge='edge_nc', to_edge='edge_ce')
        infrastructures.add_connection(from_edge='edge_nc', to_edge='edge_cw')
        # From east
        infrastructures.add_connection(from_edge='edge_ec', to_edge='edge_cs')
        infrastructures.add_connection(from_edge='edge_ec', to_edge='edge_cn')
        infrastructures.add_connection(from_edge='edge_ec', to_edge='edge_cw')
        # From south
        infrastructures.add_connection(from_edge='edge_sc', to_edge='edge_cn')
        infrastructures.add_connection(from_edge='edge_sc', to_edge='edge_ce')
        infrastructures.add_connection(from_edge='edge_sc', to_edge='edge_cw')

        # Create traffic light program
        infrastructures.add_traffic_light_program(id='c',
                                      phases=[{'duration': 1000, 'state': 'rrrGGGrrrGGG'},
                                              {'duration': 1000, 'state': 'rrryyyrrryyy'},
                                              {'duration': 1000, 'state': 'GGGrrrGGGrrr'},
                                              {'duration': 1000, 'state': 'yyyrrryyyrrr'}])

        return infrastructures




    ### Routes ###
    def generate_flows_only_ahead(self,
                                  stop_generation_time,
                                  flow_frequency,
                                  distribution='binomial'):
        """
        Generate flows for a simple crossroad.
        At the intersection, vehicles can not turn. They can only go ahead.

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
        # Intantiation of flows builder
        flows = FlowBuilder()

        # Create vehicle type
        flows.add_v_type(id='car0')

        # Create routes
        flows.add_route(id='route_we', type='car0', from_edge='edge_wc', to_edge='edge_ce')
        flows.add_route(id='route_ew', type='car0', from_edge='edge_ec', to_edge='edge_cw')
        flows.add_route(id='route_sn', type='car0', from_edge='edge_sc', to_edge='edge_cn')
        flows.add_route(id='route_ns', type='car0', from_edge='edge_nc', to_edge='edge_cs')

        # Create flows
        flows.add_flow(id='flow_we', route='route_we', end=stop_generation_time, frequency=flow_frequency, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ew', route='route_ew', end=stop_generation_time, frequency=flow_frequency, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_sn', route='route_sn', end=stop_generation_time, frequency=flow_frequency, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ns', route='route_ns', end=stop_generation_time, frequency=flow_frequency, v_type='car0', distribution=distribution)

        return flows


    def generate_flows_all_directions(self,
                                      stop_generation_time,
                                      flow_frequency,
                                      distribution='binomial'):
        """
        Generate flows for a simple crossroad.
        At the intersection, vehicles can go left, right or ahead. The proportion for each exit is uniform.

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
        # Instantiation of flows builder
        flows = FlowBuilder()

        # Create vehicle type
        flows.add_v_type(id='car0')

        # Create flows
        # From north
        flows.add_flow(id='flow_ns', from_edge='edge_nc', to_edge='edge_cs', end=stop_generation_time, frequency=flow_frequency // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ne', from_edge='edge_nc', to_edge='edge_ce', end=stop_generation_time, frequency=flow_frequency // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_nw', from_edge='edge_nc', to_edge='edge_cw', end=stop_generation_time, frequency=flow_frequency // 3, v_type='car0', distribution=distribution)
        # From east
        flows.add_flow(id='flow_es', from_edge='edge_ec', to_edge='edge_cs', end=stop_generation_time, frequency=flow_frequency // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_en', from_edge='edge_ec', to_edge='edge_cn', end=stop_generation_time, frequency=flow_frequency // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ew', from_edge='edge_ec', to_edge='edge_cw', end=stop_generation_time, frequency=flow_frequency // 3, v_type='car0', distribution=distribution)
        # From south
        flows.add_flow(id='flow_se', from_edge='edge_sc', to_edge='edge_ce', end=stop_generation_time, frequency=flow_frequency // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_sn', from_edge='edge_sc', to_edge='edge_cn', end=stop_generation_time, frequency=flow_frequency // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_sw', from_edge='edge_sc', to_edge='edge_cw', end=stop_generation_time, frequency=flow_frequency // 3, v_type='car0', distribution=distribution)
        # From west
        flows.add_flow(id='flow_we', from_edge='edge_wc', to_edge='edge_ce', end=stop_generation_time, frequency=flow_frequency // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_wn', from_edge='edge_wc', to_edge='edge_cn', end=stop_generation_time, frequency=flow_frequency // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ws', from_edge='edge_wc', to_edge='edge_cs', end=stop_generation_time, frequency=flow_frequency // 3, v_type='car0', distribution=distribution)

        return flows



    def generate_flows_with_matrix(self,
                                   period_time,
                                   load_vector,
                                   coeff_matrix,
                                   distribution='binomial'):
        """
        Generate flows for a simple crossroad.
        At the intersection, vehicles can go left, right or ahead.
        The vehicle frequency varies over time, following a scheme describe in a load vector and a coefficient matrix.
        The load vector describes, for each period, the frequency of vehicle entering the network.
        The coefficient matrix describes the proportion of load that will follow each route.
        Each scheme of frequency last a time defined in simulation steps.

        Valid parameters for config :
        - "coeff_matrix" (numpy.ndarray) : The proportion of vehicles on each route
        - "load_vector" (numpy.ndarray) : The vehicle frequency on the network for each period
        - "period_time" (int) : The period duration (in simulation steps)
        - "distribution" (str) : The distribution law for all flows. "uniform" inserts vehicles each n simulation steps
        'binomial' inserts vehicle at each simulation step with a given probability. Each of the law respect the flow frequency.

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
        # Instantiation of flows builder
        flows = FlowBuilder()

        # Create vehicle type
        flows.add_v_type(id='car0')

        # Create flows
        for i in range(coeff_matrix.shape[1]):

            coeffs_vector = coeff_matrix[:, i]
            flow_values = coeffs_vector * load_vector[i]
            flow_start = period_time * i
            flow_end = period_time * (i+1)

            # From north
            flows.add_flow(id=f'{i}_flow_ne', from_edge='edge_nc', to_edge='edge_ce', begin=flow_start, end=flow_end, frequency=flow_values[0], v_type='car0', distribution=distribution)
            flows.add_flow(id=f'{i}_flow_ns', from_edge='edge_nc', to_edge='edge_cs', begin=flow_start, end=flow_end, frequency=flow_values[1], v_type='car0', distribution=distribution)
            flows.add_flow(id=f'{i}_flow_nw', from_edge='edge_nc', to_edge='edge_cw', begin=flow_start, end=flow_end, frequency=flow_values[2], v_type='car0', distribution=distribution)
            # From east
            flows.add_flow(id=f'{i}_flow_en', from_edge='edge_ec', to_edge='edge_cn', begin=flow_start, end=flow_end, frequency=flow_values[3], v_type='car0', distribution=distribution)
            flows.add_flow(id=f'{i}_flow_es', from_edge='edge_ec', to_edge='edge_cs', begin=flow_start, end=flow_end, frequency=flow_values[4], v_type='car0', distribution=distribution)
            flows.add_flow(id=f'{i}_flow_ew', from_edge='edge_ec', to_edge='edge_cw', begin=flow_start, end=flow_end, frequency=flow_values[5], v_type='car0', distribution=distribution)
            # From south
            flows.add_flow(id=f'{i}_flow_sn', from_edge='edge_sc', to_edge='edge_cn', begin=flow_start, end=flow_end, frequency=flow_values[6], v_type='car0', distribution=distribution)
            flows.add_flow(id=f'{i}_flow_se', from_edge='edge_sc', to_edge='edge_ce', begin=flow_start, end=flow_end, frequency=flow_values[7], v_type='car0', distribution=distribution)
            flows.add_flow(id=f'{i}_flow_sw', from_edge='edge_sc', to_edge='edge_cw', begin=flow_start, end=flow_end, frequency=flow_values[8], v_type='car0', distribution=distribution)
            # From west
            flows.add_flow(id=f'{i}_flow_wn', from_edge='edge_wc', to_edge='edge_cn', begin=flow_start, end=flow_end, frequency=flow_values[9], v_type='car0', distribution=distribution)
            flows.add_flow(id=f'{i}_flow_we', from_edge='edge_wc', to_edge='edge_ce', begin=flow_start, end=flow_end, frequency=flow_values[10], v_type='car0', distribution=distribution)
            flows.add_flow(id=f'{i}_flow_ws', from_edge='edge_wc', to_edge='edge_cs', begin=flow_start, end=flow_end, frequency=flow_values[11], v_type='car0', distribution=distribution)

        return flows



    ### Additionals ###

    def generate_numerical_detectors(self):
        """
        Generate a DetectorBuilder with a numerical detector for each entry lane of the network.
        A numerical detector counts and returns the number of vehicles on its scope. In SUMO, a numerical
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.

        :return: The boolean detectors.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """
        detectors = DetectorBuilder()

        detectors.add_lane_area_detector(id="n_wc", edge='edge_wc', lane=0, type='numerical', target_tlid='c')
        detectors.add_lane_area_detector(id="n_sc", edge='edge_sc', lane=0, type='numerical', target_tlid='c')
        detectors.add_lane_area_detector(id="n_nc", edge='edge_nc', lane=0, type='numerical', target_tlid='c')
        detectors.add_lane_area_detector(id="n_ec", edge='edge_ec', lane=0, type='numerical', target_tlid='c')

        return detectors

    def generate_boolean_detectors(self, lane_length, detector_length):
        """
        Generate a DetectorBuilder with a boolean detector for each entry lane of the network.
        A boolean detector returns if a vehicle is on its scope or not. In SUMO, a boolean
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.

        :param lane_length: The default length for each lane (in meters)
        :type lane_length: int
        :param detector_length: The scope size of the detectors (in meters)
        :type detector_length: int
        :return: The numerical detectors.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """

        detectors = DetectorBuilder()

        detectors.add_lane_area_detector(id="b_wc", edge='edge_wc', lane=0, type='boolean', pos=(lane_length - detector_length - 7.2))
        detectors.add_lane_area_detector(id="b_sc", edge='edge_sc', lane=0, type='boolean', pos=(lane_length - detector_length - 7.2))
        detectors.add_lane_area_detector(id="b_nc", edge='edge_nc', lane=0, type='boolean', pos=(lane_length - detector_length - 7.2))
        detectors.add_lane_area_detector(id="b_ec", edge='edge_ec', lane=0, type='boolean', pos=(lane_length - detector_length - 7.2))

        return detectors

    def generate_saturation_detectors(self, detector_length):
        """
        Generate a DetectorBuilder with a boolean detector for each entry lane of the network.
        A boolean detector returns if a vehicle is on its scope or not. In SUMO, a boolean
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.

        :param detector_length: The scope size of the detectors (in meters)
        :type detector_length: int
        :return: The numerical detectors.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """

        detectors = DetectorBuilder()

        detectors.add_lane_area_detector(id="s_wc", edge='edge_wc', lane=0, type='saturation', pos=0, end_pos=detector_length)
        detectors.add_lane_area_detector(id="s_sc", edge='edge_sc', lane=0, type='saturation', pos=0, end_pos=detector_length)
        detectors.add_lane_area_detector(id="s_nc", edge='edge_nc', lane=0, type='saturation', pos=0, end_pos=detector_length)
        detectors.add_lane_area_detector(id="s_ec", edge='edge_ec', lane=0, type='saturation', pos=0, end_pos=detector_length)

        return detectors


    def generate_all_detectors(self, lane_length, boolean_detectors_length, saturation_detectors_length):
        """
        Generate a DetectorBuilder with boolean and numerical detectors for each entry lane of an intersection.
        A boolean detector returns if a vehicle is on its scope or not. In SUMO, a boolean
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.
        A numerical detector counts and returns the number of vehicles on its scope. In SUMO, a numerical
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.
        :param lane_length: The default length for each lane (in meters)
        :type lane_length: int
        :param boolean_detectors_length: The scope size of the detectors (in meters)
        :type boolean_detectors_length: int
        :param saturation_detectors_length: The length of the saturation detectors (in meters)
        :type saturation_detectors_length: int
        :return: The numerical detectors.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """
        detectors = DetectorBuilder()
        detectors.laneAreaDetectors.update(self.generate_boolean_detectors(lane_length, boolean_detectors_length).laneAreaDetectors)
        detectors.laneAreaDetectors.update(self.generate_numerical_detectors().laneAreaDetectors)
        detectors.laneAreaDetectors.update(self.generate_saturation_detectors(saturation_detectors_length).laneAreaDetectors)
        return detectors


    def create_TLS_DETECTORS(self):
        """
        Creates the self.TLS_DETECTORS variable. The function is static for there's only one intersection.
        :return: Nothing
        :rtype: None
        """

        self.DETECTORS_c = {
            0: {
                'boolean': ['b_wc', 'b_ec'],
                'saturation': ['s_wc', 's_ec'],
                'numerical': ['n_wc', 'n_ec'],
                'exit': []
            },
            2: {
                'boolean': ['b_nc', 'b_sc'],
                'saturation': ['s_nc', 's_sc'],
                'numerical': ['n_nc', 'n_sc'],
                'exit': []
            },
        }

        self.TLS_DETECTORS = {'c': self.DETECTORS_c}
        self.TL_IDS = ['c']

