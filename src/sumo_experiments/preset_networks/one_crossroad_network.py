import numpy as np
from sumo_experiments.components import InfrastructureBuilder, FlowBuilder, DetectorBuilder
import traci
import warnings

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

    config = {}

    WE_GREEN_LIGHT = 0
    NS_GREEN_LIGHT = 2

    CONFIG_PARAMETER_LIST = [
        'exp_name', 'lane_length', 'max_speed', 'green_time', 'yellow_time', 'cooldown_step',
        'stop_generation_time', 'flow_frequency', 'period_time', 'load_vector', 'coeff_matrix', 'min_duration_tl',
        'max_duration_tl', 'vehicle_threshold', 'simulation_duration', 'north_length', 'east_length',
        'south_length', 'west_length', 'green_time_north_south', 'green_time_west_east', 'yellow_time_north_south',
        'yellow_time_west_east', 'stop_generation_time_north', 'stop_generation_time_east', 'stop_generation_time_south',
        'stop_generation_time_west', 'flow_frequency_north', 'flow_frequency_east', 'flow_frequency_south', 'flow_frequency_west',
        'boolean_detector_length', 'simulation_duration', 'distribution'
    ]


    ### Network ###

    def generate_infrastructures(self, config):
        """
        Generate the sumo infrastructures for a network with a single crossroad, joining 4 roads.
        Each road has a lane going to the crossroad, and another going to an exit of the network.
        The crossroad is managed by traffic lights on each roads.
        The infrastructures can be customized with the config dict passed as parameter.
        A default configuration is set, and each modification in the config is modified in the default configuration.

        Valid parameters for config :
        - "lane_length" (int) : The default length for each lane (in meters)
        - "north_length" (int) : The north lane length (in meters), override default
        - "east_length" (int) : The east lane length (in meters), override default
        - "south_length" (int) : The south lane length (in meters), override default
        - "west_length" (int) : The west lane length (in meters), override default
        - "green_time" (int) : The default green time for each phase (in seconds)
        - "green_time_north_south" (int) : The north-south phase green time (in seconds), override default
        - "green_time_west_east" (int) : The west-east phase green time (in seconds), override default
        - "yellow_time" (int) : The default yellow time for each phase (in seconds)
        - "yellow_time_north_south" (int) : The north-south phase yellow time (in seconds), override default
        - "yellow_time_west_east" (int) : The west-east phase yellow time (in seconds), override default
        - "max_speed" (float) : The max speed on each lane

        :param config: Customized network configuration. Check documentation to see all parameters.
        :type config: dict
        :return: All infrastructures in a NetworkBuilder object.
        :rtype: sumo_experiments.src.components.NetworkBuilder
        """

        for key in config:
            if key not in self.CONFIG_PARAMETER_LIST:
                warnings.warn(f"The config parameter {key} is not a valid parameter.", stacklevel=2)

        # Select parameters
        len_north = config['north_length'] if 'north_length' in config else config['lane_length']
        len_east = config['east_length'] if 'east_length' in config else config['lane_length']
        len_south = config['south_length'] if 'south_length' in config else config['lane_length']
        len_west = config['west_length'] if 'west_length' in config else config['lane_length']
        if 'max_duration_tl' in config:
            gt_north_south = config['max_duration_tl']
            gt_west_east = config['max_duration_tl']
        else:
            gt_north_south = config['green_time_north_south'] if 'green_time_north_south' in config else config['green_time']
            gt_west_east = config['green_time_west_east'] if 'green_time_west_east' in config else config['green_time']
        yt_north_south = config['yellow_time_north_south'] if 'yellow_time_north_south' in config else config['yellow_time']
        yt_west_east = config['yellow_time_west_east'] if 'yellow_time_west_east' in config else config['yellow_time']
        max_speed = config['max_speed']

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
    def generate_flows_only_ahead(self, config):
        """
        Generate flows for a simple crossroad.
        At the intersection, vehicles can not turn. They can only go ahead.
        The config parameter can contain more parameter to define the frequency of vehicles from each entry, and the
        simulation step where the flow will end.

        Valid parameters for config :
        - "stop_generation_time" (int) : The default simulation step when flows will end
        - "stop_generation_time_north" (int) : The simulation step when north flows will end, override default
        - "stop_generation_time_east" (int) : The simulation step when east flows will end, override default
        - "stop_generation_time_south" (int) : The simulation step when south flows will end, override default
        - "stop_generation_time_west" (int) : The simulation step when west flows will end, override default
        - "flow_frequency" (int) : The default flows frequency (in vehicles/hour)
        - "flow_frequency_north" (int) : The north flows frequency (in vehicles/hour), override default
        - "flow_frequency_east" (int) : The east flows frequency (in vehicles/hour), override default
        - "flow_frequency_south" (int) : The south flows frequency (in vehicles/hour), override default
        - "flow_frequency_west" (int) : The west flows frequency (in vehicles/hour), override default
        - "distribution" (str) : The distribution law for all flows. "uniform" inserts vehicles each n simulation steps
        'binomial' inserts vehicle at each simulation step with a given probability. Each of the law respect the flow frequency.

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        :return: All flows in a FlowBuilder object.
        :rtype: sumo_experiments.src.components.FlowBuilder
        """

        # Select parameters
        stop_generation_time_north = config['stop_generation_time_north'] if 'stop_generation_time_north' in config else config['stop_generation_time']
        stop_generation_time_east = config['stop_generation_time_east'] if 'stop_generation_time_east' in config else config['stop_generation_time']
        stop_generation_time_south = config['stop_generation_time_south'] if 'stop_generation_time_south' in config else config['stop_generation_time']
        stop_generation_time_west = config['stop_generation_time_west'] if 'stop_generation_time_west' in config else config['stop_generation_time']
        flow_frequency_north = config['flow_frequency_north'] if 'flow_frequency_north' in config else config['flow_frequency']
        flow_frequency_east = config['flow_frequency_east'] if 'flow_frequency_east' in config else config['flow_frequency']
        flow_frequency_south = config['flow_frequency_south'] if 'flow_frequency_south' in config else config['flow_frequency']
        flow_frequency_west = config['flow_frequency_west'] if 'flow_frequency_west' in config else config['flow_frequency']
        distribution = config['distribution']

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
        flows.add_flow(id='flow_we', route='route_we', end=stop_generation_time_west, frequency=flow_frequency_west, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ew', route='route_ew', end=stop_generation_time_east, frequency=flow_frequency_east, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_sn', route='route_sn', end=stop_generation_time_south, frequency=flow_frequency_south, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ns', route='route_ns', end=stop_generation_time_north, frequency=flow_frequency_north, v_type='car0', distribution=distribution)

        return flows


    def generate_flows_all_directions(self, config):
        """
        Generate flows for a simple crossroad.
        At the intersection, vehicles can go left, right or ahead. The proportion for each exit is uniform.
        The config parameter can contain more parameter to define the frequency of vehicles from each entry, and the
        simulation step where the flow will end.

        Valid parameters for config :
        - "stop_generation_time" (int) : The default simulation step when flows will end
        - "stop_generation_time_north" (int) : The simulation step when north flows will end, override default
        - "stop_generation_time_east" (int) : The simulation step when east flows will end, override default
        - "stop_generation_time_south" (int) : The simulation step when south flows will end, override default
        - "stop_generation_time_west" (int) : The simulation step when west flows will end, override default
        - "flow_frequency" (int) : The default flows frequency (in vehicles/hour)
        - "flow_frequency_north" (int) : The north flows frequency (in vehicles/hour), override default
        - "flow_frequency_east" (int) : The east flows frequency (in vehicles/hour), override default
        - "flow_frequency_south" (int) : The south flows frequency (in vehicles/hour), override default
        - "flow_frequency_west" (int) : The west flows frequency (in vehicles/hour), override default
        - "distribution" (str) : The distribution law for all flows. "uniform" inserts vehicles each n simulation steps
        'binomial' inserts vehicle at each simulation step with a given probability. Each of the law respect the flow frequency.

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        :return: All flows in a FlowBuilder object.
        :rtype: sumo_experiments.src.components.FlowBuilder
        """

        # Select parameters
        stop_generation_time_north = config['stop_generation_time_north'] if 'stop_generation_time_north' in config else config['stop_generation_time']
        stop_generation_time_east = config['stop_generation_time_east'] if 'stop_generation_time_east' in config else config['stop_generation_time']
        stop_generation_time_south = config['stop_generation_time_south'] if 'stop_generation_time_south' in config else config['stop_generation_time']
        stop_generation_time_west = config['stop_generation_time_west'] if 'stop_generation_time_west' in config else config['stop_generation_time']
        flow_frequency_north = config['flow_frequency_north'] if 'flow_frequency_north' in config else config['flow_frequency']
        flow_frequency_east = config['flow_frequency_east'] if 'flow_frequency_east' in config else config['flow_frequency']
        flow_frequency_south = config['flow_frequency_south'] if 'flow_frequency_south' in config else config['flow_frequency']
        flow_frequency_west = config['flow_frequency_west'] if 'flow_frequency_west' in config else config['flow_frequency']
        distribution = config['distribution']

        # Instantiation of flows builder
        flows = FlowBuilder()

        # Create vehicle type
        flows.add_v_type(id='car0')

        # Create flows
        # From north
        flows.add_flow(id='flow_ns', from_edge='edge_nc', to_edge='edge_cs', end=stop_generation_time_north, frequency=flow_frequency_north // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ne', from_edge='edge_nc', to_edge='edge_ce', end=stop_generation_time_north, frequency=flow_frequency_north // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_nw', from_edge='edge_nc', to_edge='edge_cw', end=stop_generation_time_north, frequency=flow_frequency_north // 3, v_type='car0', distribution=distribution)
        # From east
        flows.add_flow(id='flow_es', from_edge='edge_ec', to_edge='edge_cs', end=stop_generation_time_east, frequency=flow_frequency_east // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_en', from_edge='edge_ec', to_edge='edge_cn', end=stop_generation_time_east, frequency=flow_frequency_east // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ew', from_edge='edge_ec', to_edge='edge_cw', end=stop_generation_time_east, frequency=flow_frequency_east // 3, v_type='car0', distribution=distribution)
        # From south
        flows.add_flow(id='flow_se', from_edge='edge_sc', to_edge='edge_ce', end=stop_generation_time_south, frequency=flow_frequency_south // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_sn', from_edge='edge_sc', to_edge='edge_cn', end=stop_generation_time_south, frequency=flow_frequency_south // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_sw', from_edge='edge_sc', to_edge='edge_cw', end=stop_generation_time_south, frequency=flow_frequency_south // 3, v_type='car0', distribution=distribution)
        # From west
        flows.add_flow(id='flow_we', from_edge='edge_wc', to_edge='edge_ce', end=stop_generation_time_west, frequency=flow_frequency_west // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_wn', from_edge='edge_wc', to_edge='edge_cn', end=stop_generation_time_west, frequency=flow_frequency_west // 3, v_type='car0', distribution=distribution)
        flows.add_flow(id='flow_ws', from_edge='edge_wc', to_edge='edge_cs', end=stop_generation_time_west, frequency=flow_frequency_west // 3, v_type='car0', distribution=distribution)

        return flows



    def generate_flows_with_matrix(self, config):
        """
        Generate flows for a simple crossroad.
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
        distribution = config['distribution']

        # Instantiation of flows builder
        flows = FlowBuilder()

        # Create vehicle type
        flows.add_v_type(id='car0')

        # Create flows
        for i in range(coeffs_matrix.shape[1]):

            coeffs_vector = coeffs_matrix[:, i]
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

    def generate_numerical_detectors(self, config):
        """
        Generate a DetectorBuilder with a numerical detector for each entry lane of the network.
        A numerical detector counts and returns the number of vehicles on its scope. In SUMO, a numerical
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.

        Valid parameters for config : No parameters needed

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        :return: An empty DetectorBuilder object.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """
        detectors = DetectorBuilder()

        detectors.add_lane_area_detector(id="wc", lane="edge_wc_0")
        detectors.add_lane_area_detector(id="sc", lane="edge_sc_0")
        detectors.add_lane_area_detector(id="nc", lane="edge_nc_0")
        detectors.add_lane_area_detector(id="ec", lane="edge_ec_0")

        return detectors

    def generate_boolean_detectors(self, config):
        """
        Generate a DetectorBuilder with a boolean detector for each entry lane of the network.
        A boolean detector returns if a vehicle is on its scope or not. In SUMO, a boolean
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.

        Valid parameters for config :
        - "lane_length" (int) : The default length for each lane (in meters)
        - "north_length" (int) : The north lane length (in meters), override default
        - "east_length" (int) : The east lane length (in meters), override default
        - "south_length" (int) : The south lane length (in meters), override default
        - "west_length" (int) : The west lane length (in meters), override default
        - "boolean_detector_length" (float) : The scope size of the detectors

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        :return: An empty DetectorBuilder object.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """

        # Select parameters
        boolean_detector_length = config['boolean_detector_length']
        len_north = config['north_length'] if 'north_length' in config else config['lane_length']
        len_east = config['east_length'] if 'east_length' in config else config['lane_length']
        len_south = config['south_length'] if 'south_length' in config else config['lane_length']
        len_west = config['west_length'] if 'west_length' in config else config['lane_length']

        detectors = DetectorBuilder()

        detectors.add_lane_area_detector(id="wc", lane="edge_wc_0", pos=(len_west - boolean_detector_length - 7.2))
        detectors.add_lane_area_detector(id="sc", lane="edge_sc_0", pos=(len_south - boolean_detector_length - 7.2))
        detectors.add_lane_area_detector(id="nc", lane="edge_nc_0", pos=(len_north - boolean_detector_length - 7.2))
        detectors.add_lane_area_detector(id="ec", lane="edge_ec_0", pos=(len_east - boolean_detector_length - 7.2))

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

        At the beginning of the simulation, The intersection is set to green for the west-east way
        and red for the north-south way.
        When the traffic light detect a car on a lane where traffic light is red, and if it doesn't
        detect any car on the green lanes, the traffic light switch to the other phase if the current phase
        is set since more than the minimum duration time.
        If this condition doesn't occur, the traffic light switch to the other phase when the current phase
        last for more than the maximum duration time.

        Valid parameters for config :
        - "min_duration_tl" (int) : The minimum number of simulation step for a traffic light phase
        - "max_duration_tl" (int) : The maximum number of simulation step for a traffic light phase
        - "simulation_duration" (int) : The number of simulation steps of the experiment

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        """

        if 'cooldown_step' not in config:
            config['cooldown_step'] = 0

        # Select parameters
        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        cooldown_step = config['cooldown_step']

        if cooldown_step > min_duration_tl:
            if traci.trafficlight.getPhase("c") == self.WE_GREEN_LIGHT:
                somebody_waiting = (traci.lanearea.getLastStepVehicleNumber("nc") >= 1 or traci.lanearea.getLastStepVehicleNumber("sc") >= 1)
                other_lane_empty = (traci.lanearea.getLastStepVehicleNumber("wc") == 0 and traci.lanearea.getLastStepVehicleNumber("ec") == 0)
                if (somebody_waiting and other_lane_empty) or (cooldown_step > max_duration_tl):
                    traci.trafficlight.setPhase("c", self.WE_GREEN_LIGHT + 1)  # Passage au orange
                    cooldown_step = 0

            elif traci.trafficlight.getPhase("c") == self.NS_GREEN_LIGHT:
                somebody_waiting = (traci.lanearea.getLastStepVehicleNumber("wc") >= 1 or traci.lanearea.getLastStepVehicleNumber("ec") >= 1)
                other_lane_empty = (traci.lanearea.getLastStepVehicleNumber("nc") == 0 and traci.lanearea.getLastStepVehicleNumber("sc") == 0)
                if (somebody_waiting and other_lane_empty) or (cooldown_step > max_duration_tl):
                    traci.trafficlight.setPhase("c", self.NS_GREEN_LIGHT + 1)
                    cooldown_step = 0
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
        When the traffic light detect more than the threshold of cars on a lane where traffic light is red,
        the traffic light switch to the other phase if the current phase is set since more than the minimum
        duration time. Only stopped cars are considered in this strategy.
        If this condition doesn't occur, the traffic light switch to the other phase when the current phase
        last for more than the maximum duration time.

        Valid parameters for config :
        - "min_duration_tl" (int) : The minimum number of simulation step for a traffic light phase
        - "max_duration_tl" (int) : The maximum number of simulation step for a traffic light phase
        - "vehicle_threshold" (int) : The number of waiting vehicles that trigger a traffic light switch
        - "simulation_duration" (int) : The number of simulation steps of the experiment

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        """

        if 'cooldown_step' not in config:
            config['cooldown_step'] = 0

        # Select parameters
        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        vehicle_threshold = config["vehicle_threshold"]
        cooldown_step = config['cooldown_step']

        if cooldown_step > min_duration_tl:
            if traci.trafficlight.getPhase("c") == self.NS_GREEN_LIGHT:
                if traci.lanearea.getLastStepVehicleNumber(
                        "wc") >= vehicle_threshold or traci.lanearea.getLastStepVehicleNumber(
                        "ec") >= vehicle_threshold or cooldown_step > max_duration_tl:
                    traci.trafficlight.setPhase("c", self.NS_GREEN_LIGHT + 1)  # Passage au orange
                    cooldown_step = 0

            elif traci.trafficlight.getPhase("c") == self.WE_GREEN_LIGHT:
                if traci.lanearea.getLastStepVehicleNumber(
                        "sc") >= vehicle_threshold or traci.lanearea.getLastStepVehicleNumber(
                        "nc") >= vehicle_threshold or cooldown_step > max_duration_tl:
                    traci.trafficlight.setPhase("c", self.WE_GREEN_LIGHT + 1)
                    cooldown_step = 0

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

        At the beginning of the simulation, The intersection is set to green for the west-east way
        and red for the north-south way.
        When the traffic light detect more than the thresholf of cars on a lane where traffic light is red,
        the traffic light switch to the other phase if the current phase is set since more than the minimum
        duration time. Both stopped and running cars are considered in this strategy.
        If this condition doesn't occur, the traffic light switch to the other phase when the current phase
        last for more than the maximum duration time.

        Valid parameters for config :
        - "min_duration_tl" (int) : The minimum number of simulation step for a traffic light phase
        - "max_duration_tl" (int) : The maximum number of simulation step for a traffic light phase
        - "vehicle_threshold" (int) : The number of waiting vehicles that trigger a traffic light switch
        - "simulation_duration" (int) : The number of simulation steps of the experiment

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        """

        if 'cooldown_step' not in config:
            config['cooldown_step'] = 0

        # Select parameters
        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        vehicle_threshold = config["vehicle_threshold"]
        cooldown_step = config['cooldown_step']

        if cooldown_step > min_duration_tl:
            if traci.trafficlight.getPhase("c") == self.NS_GREEN_LIGHT:
                if traci.lanearea.getJamLengthVehicle("wc") >= vehicle_threshold or traci.lanearea.getJamLengthVehicle(
                        "ec") >= vehicle_threshold or cooldown_step > max_duration_tl:
                    traci.trafficlight.setPhase("c", self.NS_GREEN_LIGHT + 1)  # Passage au orange
                    cooldown_step = 0

            elif traci.trafficlight.getPhase("c") == self.WE_GREEN_LIGHT:
                if traci.lanearea.getJamLengthVehicle("sc") >= vehicle_threshold or traci.lanearea.getJamLengthVehicle(
                        "nc") >= vehicle_threshold or cooldown_step > max_duration_tl:
                    traci.trafficlight.setPhase("c", self.WE_GREEN_LIGHT + 1)
                    cooldown_step = 0

        cooldown_step += 1
        config['cooldown_step'] = cooldown_step
        return config