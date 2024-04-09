from sumo_experiments.components import InfrastructureBuilder, FlowBuilder, DetectorBuilder
import traci

class OneCrossroadNetwork:
    """
    The OneCrossroadNetwork class contains a set of functions for creating a SUMO network containing a single crossroad.
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

    def __init__(self):
        """
        Init of class.
        """
        self.north_length = None
        self.south_length = None
        self.east_length = None
        self.west_length = None

    ### Network ###

    def generate_infrastructures(self,
                                 lane_length,
                                 green_time,
                                 yellow_time,
                                 max_speed,
                                 north_length=None,
                                 east_length=None,
                                 south_length=None,
                                 west_length=None,
                                 green_time_north_south=None,
                                 green_time_west_east=None,
                                 yellow_time_north_south=None,
                                 yellow_time_west_east=None,
                                 ):
        """
        Generate the sumo infrastructures for a network with a single crossroad, joining 4 roads.
        Each road has a lane going to the crossroad, and another going to an exit of the network.
        The crossroad is managed by traffic lights on each entry.

        :param lane_length: The default length for each lane (in meters)
        :type lane_length: int
        :param green_time: The default green time for each phase (in seconds)
        :type green_time: int
        :param yellow_time: The default yellow time for each phase (in seconds)
        :type yellow_time: int
        :param max_speed: The max speed on each lane (in km/h)
        :type max_speed: int
        :param north_length: The north lane length (in meters), override default
        :type north_length: int
        :param east_length: The east lane length (in meters), override default
        :type east_length: int
        :param south_length: The south lane length (in meters), override default
        :type south_length: int
        :param west_length: The west lane length (in meters), override default
        :type west_length: int
        :param green_time_north_south: The north-south phase green time (in seconds), override default
        :type green_time_north_south: int
        :param green_time_west_east: The west-east phase green time (in seconds), override default
        :type green_time_west_east: int
        :param yellow_time_north_south: The north-south phase yellow time (in seconds), override default
        :type yellow_time_north_south: int
        :param yellow_time_west_east: The west-east phase yellow time (in seconds), override default
        :type yellow_time_west_east: int
        :return: All infrastructures in a NetworkBuilder object.
        :rtype: sumo_experiments.src.components.NetworkBuilder
        """
        # Select parameters
        len_north = lane_length if north_length is None else north_length
        len_east = lane_length if east_length is None else east_length
        len_south = lane_length if south_length is None else south_length
        len_west = lane_length if west_length is None else west_length
        gt_north_south = green_time if green_time_north_south is None else green_time_north_south
        gt_west_east = green_time if green_time_west_east is None else green_time_west_east
        yt_north_south = yellow_time if yellow_time_north_south is None else yellow_time_north_south
        yt_west_east = yellow_time if yellow_time_west_east is None else yellow_time_west_east
        self.north_length = len_north
        self.east_length = len_east
        self.south_length = len_south
        self.west_length = len_west

        # Instantiation of the infrastructures builder
        infrastructures = InfrastructureBuilder()

        # Create nodes
        infrastructures.add_node(id='c', x=0, y=0, type='traffic_light', tl_program='c')    # Central node
        infrastructures.add_node(id='w', x=-len_west, y=0)        # West node
        infrastructures.add_node(id='e', x=len_east, y=0)     # East node
        infrastructures.add_node(id='s', x=0, y=-len_south)        # South node
        infrastructures.add_node(id='n', x=0, y=len_north)     # North node

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
                                      phases=[{'duration': gt_west_east, 'state': 'rrrGGGrrrGGG'},
                                              {'duration': yt_west_east, 'state': 'rrryyyrrryyy'},
                                              {'duration': gt_north_south, 'state': 'GGGrrrGGGrrr'},
                                              {'duration': yt_north_south, 'state': 'yyyrrryyyrrr'}])

        return infrastructures




    ### Routes ###
    def generate_flows_only_ahead(self,
                                  stop_generation_time,
                                  flow_frequency,
                                  distribution='binomial',
                                  stop_generation_time_north=None,
                                  stop_generation_time_east=None,
                                  stop_generation_time_south=None,
                                  stop_generation_time_west=None,
                                  flow_frequency_north=None,
                                  flow_frequency_east=None,
                                  flow_frequency_south=None,
                                  flow_frequency_west=None):
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
        :param stop_generation_time_north: The simulation step when north flows will end, override default
        :type stop_generation_time_north: int
        :param stop_generation_time_east: The simulation step when east flows will end, override default
        :type stop_generation_time_east: int
        :param stop_generation_time_south: The simulation step when south flows will end, override default
        :type stop_generation_time_south: int
        :param stop_generation_time_west: The simulation step when west flows will end, override default
        :type stop_generation_time_west: int
        :param flow_frequency_north: The north flows frequency (in vehicles/hour/entry), override default
        :type flow_frequency_north: int
        :param flow_frequency_east: The east flows frequency (in vehicles/hour/entry), override default
        :type flow_frequency_east: int
        :param flow_frequency_south: The south flows frequency (in vehicles/hour/entry), override default
        :type flow_frequency_south: int
        :param flow_frequency_west: The west flows frequency (in vehicles/hour/entry), override default
        :type flow_frequency_west: int
        :return: All flows in a FlowBuilder object.
        :rtype: sumo_experiments.src.components.FlowBuilder
        """

        # Select parameters
        stop_north = stop_generation_time if stop_generation_time_north is None else stop_generation_time_north
        stop_east = stop_generation_time if stop_generation_time_east is None else stop_generation_time_east
        stop_south = stop_generation_time if stop_generation_time_south is None else stop_generation_time_south
        stop_west = stop_generation_time if stop_generation_time_west is None else stop_generation_time_west
        frequency_north = flow_frequency if flow_frequency_north is None else flow_frequency_north
        frequency_east = flow_frequency if flow_frequency_east is None else flow_frequency_east
        frequency_south = flow_frequency if flow_frequency_south is None else flow_frequency_south
        frequency_west = flow_frequency if flow_frequency_west is None else flow_frequency_west

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
        flows.add_flow(id='flow_we', route='route_we', end=stop_west, frequency=frequency_west, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ew', route='route_ew', end=stop_east, frequency=frequency_east, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_sn', route='route_sn', end=stop_south, frequency=frequency_south, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ns', route='route_ns', end=stop_north, frequency=frequency_north, v_type='car0', distribution=distribution)

        return flows


    def generate_flows_all_directions(self,
                                      stop_generation_time,
                                      flow_frequency,
                                      distribution='binomial',
                                      stop_generation_time_north=None,
                                      stop_generation_time_east=None,
                                      stop_generation_time_south=None,
                                      stop_generation_time_west=None,
                                      flow_frequency_north=None,
                                      flow_frequency_east=None,
                                      flow_frequency_south=None,
                                      flow_frequency_west=None):
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
        :param stop_generation_time_north: The simulation step when north flows will end, override default
        :type stop_generation_time_north: int
        :param stop_generation_time_east: The simulation step when east flows will end, override default
        :type stop_generation_time_east: int
        :param stop_generation_time_south: The simulation step when south flows will end, override default
        :type stop_generation_time_south: int
        :param stop_generation_time_west: The simulation step when west flows will end, override default
        :type stop_generation_time_west: int
        :param flow_frequency_north: The north flows frequency (in vehicles/hour/entry), override default
        :type flow_frequency_north: int
        :param flow_frequency_east: The east flows frequency (in vehicles/hour/entry), override default
        :type flow_frequency_east: int
        :param flow_frequency_south: The south flows frequency (in vehicles/hour/entry), override default
        :type flow_frequency_south: int
        :param flow_frequency_west: The west flows frequency (in vehicles/hour/entry), override default
        :type flow_frequency_west: int
        :return: All flows in a FlowBuilder object.
        :rtype: sumo_experiments.src.components.FlowBuilder
        """

        # Select parameters
        stop_north = stop_generation_time if stop_generation_time_north is None else stop_generation_time_north
        stop_east = stop_generation_time if stop_generation_time_east is None else stop_generation_time_east
        stop_south = stop_generation_time if stop_generation_time_south is None else stop_generation_time_south
        stop_west = stop_generation_time if stop_generation_time_west is None else stop_generation_time_west
        frequency_north = flow_frequency if flow_frequency_north is None else flow_frequency_north
        frequency_east = flow_frequency if flow_frequency_east is None else flow_frequency_east
        frequency_south = flow_frequency if flow_frequency_south is None else flow_frequency_south
        frequency_west = flow_frequency if flow_frequency_west is None else flow_frequency_west

        # Instantiation of flows builder
        flows = FlowBuilder()

        # Create vehicle type
        flows.add_v_type(id='car0')

        # Create flows
        # From north
        flows.add_flow(id='flow_ns', from_edge='edge_nc', to_edge='edge_cs', end=stop_north, frequency=frequency_north // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ne', from_edge='edge_nc', to_edge='edge_ce', end=stop_north, frequency=frequency_north // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_nw', from_edge='edge_nc', to_edge='edge_cw', end=stop_north, frequency=frequency_north // 3, v_type='car0', distribution=distribution)
        # From east
        flows.add_flow(id='flow_es', from_edge='edge_ec', to_edge='edge_cs', end=stop_east, frequency=frequency_east // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_en', from_edge='edge_ec', to_edge='edge_cn', end=stop_east, frequency=frequency_east // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ew', from_edge='edge_ec', to_edge='edge_cw', end=stop_east, frequency=frequency_east // 3, v_type='car0', distribution=distribution)
        # From south
        flows.add_flow(id='flow_se', from_edge='edge_sc', to_edge='edge_ce', end=stop_south, frequency=frequency_south // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_sn', from_edge='edge_sc', to_edge='edge_cn', end=stop_south, frequency=frequency_south // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_sw', from_edge='edge_sc', to_edge='edge_cw', end=stop_south, frequency=frequency_south // 3, v_type='car0', distribution=distribution)
        # From west
        flows.add_flow(id='flow_we', from_edge='edge_wc', to_edge='edge_ce', end=stop_west, frequency=frequency_west // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_wn', from_edge='edge_wc', to_edge='edge_cn', end=stop_west, frequency=frequency_west // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ws', from_edge='edge_wc', to_edge='edge_cs', end=stop_west, frequency=frequency_west // 3, v_type='car0', distribution=distribution)

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

        detectors.add_lane_area_detector(id="n_wc", edge='edge_wc', lane=0, type='numerical')
        detectors.add_lane_area_detector(id="n_sc", edge='edge_sc', lane=0, type='numerical')
        detectors.add_lane_area_detector(id="n_nc", edge='edge_nc', lane=0, type='numerical')
        detectors.add_lane_area_detector(id="n_ec", edge='edge_ec', lane=0, type='numerical')

        return detectors

    def generate_boolean_detectors(self, boolean_detector_length):
        """
        Generate a DetectorBuilder with a boolean detector for each entry lane of the network.
        A boolean detector returns if a vehicle is on its scope or not. In SUMO, a boolean
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.

        :param boolean_detector_length: The scope size of the detectors (in meters)
        :type boolean_detector_length: int
        :return: The numerical detectors.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """

        detectors = DetectorBuilder()

        detectors.add_lane_area_detector(id="b_wc", edge='edge_wc', lane=0, type='boolean', pos=(self.west_length - boolean_detector_length - 7.2))
        detectors.add_lane_area_detector(id="b_sc", edge='edge_sc', lane=0, type='boolean', pos=(self.south_length - boolean_detector_length - 7.2))
        detectors.add_lane_area_detector(id="b_nc", edge='edge_nc', lane=0, type='boolean', pos=(self.north_length - boolean_detector_length - 7.2))
        detectors.add_lane_area_detector(id="b_ec", edge='edge_ec', lane=0, type='boolean', pos=(self.east_length - boolean_detector_length - 7.2))

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
