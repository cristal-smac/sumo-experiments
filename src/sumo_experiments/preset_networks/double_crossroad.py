import warnings

from sumo_experiments.components import InfrastructureBuilder, FlowBuilder, DetectorBuilder
import traci


class TwoCrossroadsNetwork:
    """
        The TwoCrossroadsNetwork class contains a set of functions for creating a SUMO network containing
        two consecutive crossroads.

        There are 4 different types of function:
            - Functions generating infrastructures
            - Functions generating flows
            - Functions generationg detectors
            - Functions representing junction management strategies, usable only when an experiment is launched with TraCi

        It is not possible to combine certain functions to generate a SUMO network.
        Check the documentation of functions for more information.
    """

    GREEN_LIGHT_HORIZONTAL_1 = 0
    GREEN_LIGHT_VERTICAL_1 = 2
    GREEN_LIGHT_HORIZONTAL_2 = 2
    GREEN_LIGHT_VERTICAL_2 = 0

    DEFAULT_CONFIG = {
        'lane_length': 100,
        'max_speed': 30,
        'green_time': 30,
        'yellow_time': 3,
        'stop_generation_time': 1000,
        'flow_density': 600,
        'period_time': 300,
        'min_duration_tl': 30,
        'max_duration_tl': 60,
        'vehicle_threshold': 5,
        'simulation_duration': 1000,
        'boolean_detector_length': 7
    }

    CONFIG_PARAMETER_LIST = [
        'exp_name', 'lane_length', 'max_speed', 'green_time', 'yellow_time',
        'stop_generation_time', 'flow_density', 'period_time', 'load_vector', 'coeff_matrix', 'min_duration_tl',
        'max_duration_tl', 'vehicle_threshold', 'simulation_duration'
    ]

    ### Networks ###

    def generate_infrastructures(self, config={}):
        """
        Generate the sumo infrastructures for a network with two consecutive crossroads.
        The infrastructures can be customized with the config dict passed as parameter.
        A default configuration is set, and each modification in the config is modified in the default configuration.

        Valid parameters for config :
        - "lane_length" (int) : The default length for each lane (in meters)
        - "north_1_length" (int) : The north lane length (in meters) for intersection 1, override default
        - "north_2_length" (int) : The north lane length (in meters) for intersection 2, override default
        - "east_length" (int) : The east lane length (in meters), override default
        - "south_1_length" (int) : The south lane length (in meters) for intersection 1, override default
        - "south_2_length" (int) : The south lane length (in meters) for intersection 2, override default
        - "west_length" (int) : The west lane length (in meters), override default
        - "center_length" (int): The center lane length (in meters), override default
        - "green_time" (int) : The default green time for each phase (in seconds)
        - "green_time_north_south_1" (int) : The north-south phase green time (in seconds) for intersection 1, override default
        - "green_time_north_south_2" (int) : The north-south phase green time (in seconds) for intersection 2, override default
        - "green_time_west_east_1" (int) : The west-east phase green time (in seconds) for intersection 1, override default
        - "green_time_west_east_2" (int) : The west-east phase green time (in seconds) for intersection 2, override default
        - "yellow_time" (int) : The default yellow time for each phase (in seconds)
        - "yellow_time_north_south_1" (int) : The north-south phase yellow time (in seconds) for intersection 1, override default
        - "yellow_time_north_south_2" (int) : The north-south phase yellow time (in seconds) for intersection 2, override default
        - "yellow_time_west_east_1" (int) : The west-east phase yellow time (in seconds) for intersection 1, override default
        - "yellow_time_west_east_2" (int) : The west-east phase yellow time (in seconds) for intersection 2, override default
        - "max_speed" (float) : The max speed on each lane

        :param config: Customized network configuration. Check documentation to see all parameters.
        :type config: dict
        :return: All infrastructures in a NetworkBuilder object.
        :rtype: sumo_experiments.src.components.NetworkBuilder
        """

        # Get new parameters from config
        current_config = self.DEFAULT_CONFIG
        for key in config:
            if key not in self.CONFIG_PARAMETER_LIST:
                warnings.warn(f"The config parameter '{key}' is not a valid parameter.", stacklevel=2)
            current_config[key] = config[key]

        # Select parameters
        len_north_1 = current_config['north_1_length'] if 'north_1_length' in current_config else current_config['lane_length']
        len_north_2 = current_config['north_2_length'] if 'north_2_length' in current_config else current_config['lane_length']
        len_south_1 = current_config['south_1_length'] if 'south_1_length' in current_config else current_config['lane_length']
        len_south_2 = current_config['south_2_length'] if 'south_2_length' in current_config else current_config['lane_length']
        len_east = current_config['east_length'] if 'east_length' in current_config else current_config['lane_length']
        len_west = current_config['west_length'] if 'west_length' in current_config else current_config['lane_length']
        len_center = current_config['center_length'] if 'west_length' in current_config else current_config['lane_length']
        gt_north_south_1 = current_config['green_time_north_south_1'] if 'green_time_north_south_1' in current_config else current_config['green_time']
        gt_west_east_1 = current_config['green_time_west_east_1'] if 'green_time_west_east_1' in current_config else current_config['green_time']
        gt_north_south_2 = current_config['green_time_north_south_2'] if 'green_time_north_south_2' in current_config else current_config['green_time']
        gt_west_east_2 = current_config['green_time_west_east_2'] if 'green_time_west_east_2' in current_config else current_config['green_time']
        yt_north_south_1 = current_config['yellow_time_north_south_1'] if 'yellow_time_north_south_1' in current_config else current_config['yellow_time']
        yt_west_east_1 = current_config['yellow_time_west_east_1'] if 'yellow_time_west_east_1' in current_config else current_config['yellow_time']
        yt_north_south_2 = current_config['yellow_time_north_south_2'] if 'yellow_time_north_south_2' in current_config else current_config['yellow_time']
        yt_west_east_2 = current_config['yellow_time_west_east_2'] if 'yellow_time_west_east_2' in current_config else current_config['yellow_time']
        max_speed = current_config['max_speed']

        net = InfrastructureBuilder()

        net.add_node(id='c1', x=0, y=0, type='traffic_light', tl_program='c1')
        net.add_node(id='c2', x=len_center, y=0, type='traffic_light', tl_program='c2')

        net.add_node(id='w', x=-len_west, y=0)
        net.add_node(id='e', x=(len_center + len_east), y=0)

        net.add_node(id='s1', x=0, y=-len_south_1)
        net.add_node(id='n1', x=0, y=len_north_1)

        net.add_node(id='s2', x=len_center, y=-len_south_2)
        net.add_node(id='n2', x=len_center, y=len_north_2)

        net.add_edge_type(id='default', params={'numLanes': '1', 'speed': max_speed})

        # Edge entrant sur le premier carrefour
        net.add_edge(id='edge_wc1', from_node='w', to_node='c1', edge_type='default')
        net.add_edge(id='edge_s1c1', from_node='s1', to_node='c1', edge_type='default')
        net.add_edge(id='edge_n1c1', from_node='n1', to_node='c1', edge_type='default')
        net.add_edge(id='edge_c2c1', from_node='c2', to_node='c1', edge_type='default')
        # Edge sortant du premier carrefour
        net.add_edge(id='edge_c1c2', from_node='c1', to_node='c2', edge_type='default')
        net.add_edge(id='edge_c1n1', from_node='c1', to_node='n1', edge_type='default')
        net.add_edge(id='edge_c1w', from_node='c1', to_node='w', edge_type='default')
        net.add_edge(id='edge_c1s1', from_node='c1', to_node='s1', edge_type='default')
        # Edge ebtrant sur le second carrefour
        net.add_edge(id='edge_ec2', from_node='e', to_node='c2', edge_type='default')
        net.add_edge(id='edge_s2c2', from_node='s2', to_node='c2', edge_type='default')
        net.add_edge(id='edge_n2c2', from_node='n2', to_node='c2', edge_type='default')
        # Edge sortant du second carrefour
        net.add_edge(id='edge_c2e', from_node='c2', to_node='e', edge_type='default')
        net.add_edge(id='edge_c2n2', from_node='c2', to_node='n2', edge_type='default')
        net.add_edge(id='edge_c2s2', from_node='c2', to_node='s2', edge_type='default')

        # Connexion des arêtes du premier carrefour
        net.add_connection(from_edge='edge_wc1', to_edge='edge_c1c2')
        net.add_connection(from_edge='edge_wc1', to_edge='edge_c1n1')
        net.add_connection(from_edge='edge_wc1', to_edge='edge_c1s1')
        net.add_connection(from_edge='edge_n1c1', to_edge='edge_c1s1')
        net.add_connection(from_edge='edge_n1c1', to_edge='edge_c1c2')
        net.add_connection(from_edge='edge_n1c1', to_edge='edge_c1w')
        net.add_connection(from_edge='edge_s1c1', to_edge='edge_c1c2')
        net.add_connection(from_edge='edge_s1c1', to_edge='edge_c1n1')
        net.add_connection(from_edge='edge_s1c1', to_edge='edge_c1w')
        net.add_connection(from_edge='edge_c2c1', to_edge='edge_c1s1')
        net.add_connection(from_edge='edge_c2c1', to_edge='edge_c1n1')
        net.add_connection(from_edge='edge_c2c1', to_edge='edge_c1w')
        # Connexion des arêtes du second carrefour
        net.add_connection(from_edge='edge_c1c2', to_edge='edge_c2e')
        net.add_connection(from_edge='edge_c1c2', to_edge='edge_c2n2')
        net.add_connection(from_edge='edge_c1c2', to_edge='edge_c2s2')
        net.add_connection(from_edge='edge_s2c2', to_edge='edge_c2n2')
        net.add_connection(from_edge='edge_s2c2', to_edge='edge_c2e')
        net.add_connection(from_edge='edge_s2c2', to_edge='edge_c2c1')
        net.add_connection(from_edge='edge_n2c2', to_edge='edge_c2e')
        net.add_connection(from_edge='edge_n2c2', to_edge='edge_c2s2')
        net.add_connection(from_edge='edge_n2c2', to_edge='edge_c2c1')
        net.add_connection(from_edge='edge_ec2', to_edge='edge_c2n2')
        net.add_connection(from_edge='edge_ec2', to_edge='edge_c2s2')
        net.add_connection(from_edge='edge_ec2', to_edge='edge_c2c1')

        net.add_traffic_light_program(id='c1',
                                      phases=[{'duration': gt_west_east_1, 'state': 'rrrGGGrrrGGG'},
                                              {'duration': yt_west_east_1, 'state': 'rrryyyrrryyy'},
                                              {'duration': gt_north_south_1, 'state': 'GGGrrrGGGrrr'},
                                              {'duration': yt_north_south_1, 'state': 'yyyrrryyyrrr'}])

        net.add_traffic_light_program(id='c2',
                                      phases=[{'duration': gt_west_east_2, 'state': 'GGGrrrGGGrrr'},
                                              {'duration': yt_west_east_2, 'state': 'yyyrrryyyrrr'},
                                              {'duration': gt_north_south_2, 'state': 'rrrGGGrrrGGG'},
                                              {'duration': yt_north_south_2, 'state': 'rrryyyrrryyy'}])

        return net



    ### Routes ###

    def generate_flows_only_ahead(self, config={}):
        """
        Generate flows for a network with two consecutive crossroads.
        At the intersections, vehicles can not turn. They can only go ahead.
        The config parameter can contain more parameter to define the density of vehicles from each entry, and the
        simulation step where the flow will end.

        Valid parameters for config :
        - "stop_generation_time" (int) : The default simulation step when flows will end
        - "stop_generation_time_north_1" (int) : The simulation step when north flows will end for intersection 1, override default
        - "stop_generation_time_north_2" (int) : The simulation step when north flows will end for intersection 2, override default
        - "stop_generation_time_east" (int) : The simulation step when east flows will end, override default
        - "stop_generation_time_south_1" (int) : The simulation step when south flows will end for intersection 1, override default
        - "stop_generation_time_south_2" (int) : The simulation step when south flows will end for intersection 2, override default
        - "stop_generation_time_west" (int) : The simulation step when west flows will end, override default
        - "flow_density" (int) : The default flows density (in vehicles/hour)
        - "flow_density_north_1" (int) : The north flows density (in vehicles/hour) for intersection 1, override default
        - "flow_density_north_2" (int) : The north flows density (in vehicles/hour) for intersection 2, override default
        - "flow_density_east" (int) : The east flows density (in vehicles/hour), override default
        - "flow_density_south_1" (int) : The south flows density (in vehicles/hour) for intersection 1, override default
        - "flow_density_south_2" (int) : The south flows density (in vehicles/hour) for intersection 2, override default
        - "flow_density_west" (int) : The west flows density (in vehicles/hour), override default

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        :return: All flows in a FlowBuilder object.
        :rtype: sumo_experiments.src.components.FlowBuilder
        """
        # Get new parameters from config
        current_config = self.DEFAULT_CONFIG
        for key in config:
            current_config[key] = config[key]

        # Select parameters
        stop_generation_time_north_1 = current_config['stop_generation_time_north_1'] if 'stop_generation_time_north_1' in current_config else current_config['stop_generation_time']
        stop_generation_time_north_2 = current_config['stop_generation_time_north_2'] if 'stop_generation_time_north_2' in current_config else current_config['stop_generation_time']
        stop_generation_time_east = current_config['stop_generation_time_east'] if 'stop_generation_time_east' in current_config else current_config['stop_generation_time']
        stop_generation_time_south_1 = current_config['stop_generation_time_south_1'] if 'stop_generation_time_south_1' in current_config else current_config['stop_generation_time']
        stop_generation_time_south_2 = current_config['stop_generation_time_south_2'] if 'stop_generation_time_south_2' in current_config else current_config['stop_generation_time']
        stop_generation_time_west = current_config['stop_generation_time_west'] if 'stop_generation_time_west' in current_config else current_config['stop_generation_time']
        flow_density_north_1 = current_config['flow_density_north_1'] if 'flow_density_north_1' in current_config else current_config['flow_density']
        flow_density_north_2 = current_config['flow_density_north_2'] if 'flow_density_north_2' in current_config else current_config['flow_density']
        flow_density_east = current_config['flow_density_east'] if 'flow_density_east' in current_config else current_config['flow_density']
        flow_density_south_1 = current_config['flow_density_south_1'] if 'flow_density_south_1' in current_config else current_config['flow_density']
        flow_density_south_2 = current_config['flow_density_south_2'] if 'flow_density_south_2' in current_config else current_config['flow_density']
        flow_density_west = current_config['flow_density_west'] if 'flow_density_west' in current_config else current_config['flow_density']

        routes = FlowBuilder()

        routes.add_vType(id='car0')

        routes.add_flow(id='flow_we', from_edge='edge_wc1', to_edge='edge_c2e', end=stop_generation_time_west, density=flow_density_west, v_type='car0')
        routes.add_flow(id='flow_ew', from_edge='edge_ec2', to_edge='edge_c1w', end=stop_generation_time_east, density=flow_density_east, v_type='car0')
        routes.add_flow(id='flow_n1s1', from_edge='edge_n1c1', to_edge='edge_c1s1', end=stop_generation_time_north_1, density=flow_density_north_1, v_type='car0')
        routes.add_flow(id='flow_s1n1', from_edge='edge_s1c1', to_edge='edge_c1n1', end=stop_generation_time_south_1, density=flow_density_south_1, v_type='car0')
        routes.add_flow(id='flow_n2s2', from_edge='edge_n2c2', to_edge='edge_c2s2', end=stop_generation_time_north_2, density=flow_density_north_2, v_type='car0')
        routes.add_flow(id='flow_s2n2', from_edge='edge_s2c2', to_edge='edge_c2n2', end=stop_generation_time_south_2, density=flow_density_south_2, v_type='car0')

        return routes


    def generate_flows_all_directions(self, config={}):
        """
        Generate flows for a network with two consecutive crossroads.
        At the intersections, vehicles can go to any direction.
        The config parameter can contain more parameter to define the density of vehicles from each entry, and the
        simulation step where the flow will end.

        Valid parameters for config :
        - "stop_generation_time" (int) : The default simulation step when flows will end
        - "stop_generation_time_north_1" (int) : The simulation step when north flows will end for intersection 1, override default
        - "stop_generation_time_north_2" (int) : The simulation step when north flows will end for intersection 2, override default
        - "stop_generation_time_east" (int) : The simulation step when east flows will end, override default
        - "stop_generation_time_south_1" (int) : The simulation step when south flows will end for intersection 1, override default
        - "stop_generation_time_south_2" (int) : The simulation step when south flows will end for intersection 2, override default
        - "stop_generation_time_west" (int) : The simulation step when west flows will end, override default
        - "flow_density" (int) : The default flows density (in vehicles/hour)
        - "flow_density_north_1" (int) : The north flows density (in vehicles/hour) for intersection 1, override default
        - "flow_density_north_2" (int) : The north flows density (in vehicles/hour) for intersection 2, override default
        - "flow_density_east" (int) : The east flows density (in vehicles/hour), override default
        - "flow_density_south_1" (int) : The south flows density (in vehicles/hour) for intersection 1, override default
        - "flow_density_south_2" (int) : The south flows density (in vehicles/hour) for intersection 2, override default
        - "flow_density_west" (int) : The west flows density (in vehicles/hour), override default

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        :return: All flows in a FlowBuilder object.
        :rtype: sumo_experiments.src.components.FlowBuilder
        """
        # Get new parameters from config
        current_config = self.DEFAULT_CONFIG
        for key in config:
            current_config[key] = config[key]

        # Select parameters
        stop_generation_time_north_1 = current_config['stop_generation_time_north_1'] if 'stop_generation_time_north_1' in current_config else current_config['stop_generation_time']
        stop_generation_time_north_2 = current_config['stop_generation_time_north_2'] if 'stop_generation_time_north_2' in current_config else current_config['stop_generation_time']
        stop_generation_time_east = current_config['stop_generation_time_east'] if 'stop_generation_time_east' in current_config else current_config['stop_generation_time']
        stop_generation_time_south_1 = current_config['stop_generation_time_south_1'] if 'stop_generation_time_south_1' in current_config else current_config['stop_generation_time']
        stop_generation_time_south_2 = current_config['stop_generation_time_south_2'] if 'stop_generation_time_south_2' in current_config else current_config['stop_generation_time']
        stop_generation_time_west = current_config['stop_generation_time_west'] if 'stop_generation_time_west' in current_config else current_config['stop_generation_time']
        flow_density_north_1 = current_config['flow_density_north_1'] if 'flow_density_north_1' in current_config else current_config['flow_density']
        flow_density_north_2 = current_config['flow_density_north_2'] if 'flow_density_north_2' in current_config else current_config['flow_density']
        flow_density_east = current_config['flow_density_east'] if 'flow_density_east' in current_config else current_config['flow_density']
        flow_density_south_1 = current_config['flow_density_south_1'] if 'flow_density_south_1' in current_config else current_config['flow_density']
        flow_density_south_2 = current_config['flow_density_south_2'] if 'flow_density_south_2' in current_config else current_config['flow_density']
        flow_density_west = current_config['flow_density_west'] if 'flow_density_west' in current_config else current_config['flow_density']

        routes = FlowBuilder()

        routes.add_vType(id='car0')

        # From north 1
        routes.add_flow(id=f'flow_n1n2', from_edge='edge_n1c1', to_edge='edge_c2n2', end=stop_generation_time_north_1, density=flow_density_north_1 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_n1e', from_edge='edge_n1c1', to_edge='edge_c2e', end=stop_generation_time_north_1, density=flow_density_north_1 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_n1s2', from_edge='edge_n1c1', to_edge='edge_c2s2', end=stop_generation_time_north_1, density=flow_density_north_1 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_n1s1', from_edge='edge_n1c1', to_edge='edge_c1s1', end=stop_generation_time_north_1, density=flow_density_north_1 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_n1w', from_edge='edge_n1c1', to_edge='edge_c1w', end=stop_generation_time_north_1, density=flow_density_north_1 // 5, v_type='car0', distribution='binomial')

        # From north 2
        routes.add_flow(id=f'flow_n2e', from_edge='edge_n2c2', to_edge='edge_c2e', end=stop_generation_time_north_2, density=flow_density_north_2 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_n2s2', from_edge='edge_n2c2', to_edge='edge_c2s2', end=stop_generation_time_north_2, density=flow_density_north_2 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_n2s1', from_edge='edge_n2c2', to_edge='edge_c1s1', end=stop_generation_time_north_2, density=flow_density_north_2 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_n2w', from_edge='edge_n2c2', to_edge='edge_c1w', end=stop_generation_time_north_2, density=flow_density_north_2 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_n2n1', from_edge='edge_n2c2', to_edge='edge_c1n1', end=stop_generation_time_north_2, density=flow_density_north_2 // 5, v_type='car0', distribution='binomial')

        # From east
        routes.add_flow(id=f'flow_es2', from_edge='edge_ec2', to_edge='edge_c2s2', end=stop_generation_time_east, density=flow_density_east // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_es1', from_edge='edge_ec2', to_edge='edge_c1s1', end=stop_generation_time_east, density=flow_density_east // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_ew', from_edge='edge_ec2', to_edge='edge_c1w', end=stop_generation_time_east, density=flow_density_east // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_en1', from_edge='edge_ec2', to_edge='edge_c1n1', end=stop_generation_time_east, density=flow_density_east // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_en2', from_edge='edge_ec2', to_edge='edge_c2n2', end=stop_generation_time_east, density=flow_density_east // 5, v_type='car0', distribution='binomial')

        # From south 2
        routes.add_flow(id=f'flow_s2s1', from_edge='edge_s2c2', to_edge='edge_c1s1', end=stop_generation_time_south_2, density=flow_density_south_2 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_s2w', from_edge='edge_s2c2', to_edge='edge_c1w', end=stop_generation_time_south_2, density=flow_density_south_2 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_s2n1', from_edge='edge_s2c2', to_edge='edge_c1n1', end=stop_generation_time_south_2, density=flow_density_south_2 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_s2n2', from_edge='edge_s2c2', to_edge='edge_c2n2', end=stop_generation_time_south_2, density=flow_density_south_2 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_s2e', from_edge='edge_s2c2', to_edge='edge_c2e', end=stop_generation_time_south_2, density=flow_density_south_2 // 5, v_type='car0', distribution='binomial')

        # From south 1
        routes.add_flow(id=f'flow_s1w', from_edge='edge_s1c1', to_edge='edge_c1w', end=stop_generation_time_south_1, density=flow_density_south_1 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_s1n1', from_edge='edge_s1c1', to_edge='edge_c1n1', end=stop_generation_time_south_1, density=flow_density_south_1 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_s1n2', from_edge='edge_s1c1', to_edge='edge_c2n2', end=stop_generation_time_south_1, density=flow_density_south_1 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_s1e', from_edge='edge_s1c1', to_edge='edge_c2e', end=stop_generation_time_south_1, density=flow_density_south_1 // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_s1s2', from_edge='edge_s1c1', to_edge='edge_c2s2', end=stop_generation_time_south_1, density=flow_density_south_1 // 5, v_type='car0', distribution='binomial')

        # From west
        routes.add_flow(id=f'flow_wn1', from_edge='edge_wc1', to_edge='edge_c1n1', end=stop_generation_time_west, density=flow_density_west // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_wn2', from_edge='edge_wc1', to_edge='edge_c2n2', end=stop_generation_time_west, density=flow_density_west // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_we', from_edge='edge_wc1', to_edge='edge_c2e', end=stop_generation_time_west, density=flow_density_west // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_ws2', from_edge='edge_wc1', to_edge='edge_c2s2', end=stop_generation_time_west, density=flow_density_west // 5, v_type='car0', distribution='binomial')
        routes.add_flow(id=f'flow_ws1', from_edge='edge_wc1', to_edge='edge_c1s1', end=stop_generation_time_west, density=flow_density_west // 5, v_type='car0', distribution='binomial')

        return routes


    def generate_flows_with_matrix(self, config={}):
        """
        Generate flows for a network with two consecutive crossroads.
        At the intersections, vehicles can go to any direction.
        The vehicle density varies over time, following a scheme describe in a load vector and a coefficient matrix.
        The load vector describes, for each period, the density of vehicle entering the network.
        The coefficient matrix describes the proportion of load that will follow each route.
        Each scheme of density last a time defined in simulation steps.
        The config parameter can contain more parameter to define the density of vehicles from each entry, and the
        simulation step where the flow will end.

        Valid parameters for config :
        - "coeff_matrix" (numpy.ndarray) : The proportion of vehicles on each route
        - "load_vector" (numpy.ndarray) : The vehicle density on the network for each period
        - "period_time" (int) : The period duration (in simulation steps)

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        :return: All flows in a FlowBuilder object.
        :rtype: sumo_experiments.src.components.FlowBuilder
        """

        # Get new parameters from config
        current_config = self.DEFAULT_CONFIG
        for key in config:
            current_config[key] = config[key]

        # Select parameters
        coeffs_matrix = current_config['coeff_matrix']
        load_vector = current_config['load_vector']
        period_time = current_config['period_time']

        flows = FlowBuilder()

        flows.add_vType(id='car0')

        for i in range(coeffs_matrix.shape[1]):
            vecteur_coeffs = coeffs_matrix[:, i]
            flow_values = vecteur_coeffs * load_vector[i]
            flow_start = period_time * i
            flow_end = period_time * (i + 1)

            # From north 1
            flows.add_flow(id=f'{i}_flow_n1n2', from_edge='edge_n1c1', to_edge='edge_c2n2', begin=flow_start, end=flow_end,
                            density=flow_values[0], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_n1e', from_edge='edge_n1c1', to_edge='edge_c2e', begin=flow_start, end=flow_end,
                            density=flow_values[1], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_n1s2', from_edge='edge_n1c1', to_edge='edge_c2s2', begin=flow_start, end=flow_end,
                            density=flow_values[2], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_n1s1', from_edge='edge_n1c1', to_edge='edge_c1s1', begin=flow_start, end=flow_end,
                            density=flow_values[3], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_n1w', from_edge='edge_n1c1', to_edge='edge_c1w', begin=flow_start, end=flow_end,
                            density=flow_values[4], v_type='car0', distribution='binomial')

            # From north 2
            flows.add_flow(id=f'{i}_flow_n2e', from_edge='edge_n2c2', to_edge='edge_c2e', begin=flow_start, end=flow_end,
                            density=flow_values[5], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_n2s2', from_edge='edge_n2c2', to_edge='edge_c2s2', begin=flow_start, end=flow_end,
                            density=flow_values[6], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_n2s1', from_edge='edge_n2c2', to_edge='edge_c1s1', begin=flow_start, end=flow_end,
                            density=flow_values[7], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_n2w', from_edge='edge_n2c2', to_edge='edge_c1w', begin=flow_start, end=flow_end,
                            density=flow_values[8], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_n2n1', from_edge='edge_n2c2', to_edge='edge_c1n1', begin=flow_start, end=flow_end,
                            density=flow_values[9], v_type='car0', distribution='binomial')

            # From east
            flows.add_flow(id=f'{i}_flow_es2', from_edge='edge_ec2', to_edge='edge_c2s2', begin=flow_start, end=flow_end,
                            density=flow_values[10], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_es1', from_edge='edge_ec2', to_edge='edge_c1s1', begin=flow_start, end=flow_end,
                            density=flow_values[11], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_ew', from_edge='edge_ec2', to_edge='edge_c1w', begin=flow_start, end=flow_end,
                            density=flow_values[12], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_en1', from_edge='edge_ec2', to_edge='edge_c1n1', begin=flow_start, end=flow_end,
                            density=flow_values[13], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_en2', from_edge='edge_ec2', to_edge='edge_c2n2', begin=flow_start, end=flow_end,
                            density=flow_values[14], v_type='car0', distribution='binomial')

            # From south 2
            flows.add_flow(id=f'{i}_flow_s2s1', from_edge='edge_s2c2', to_edge='edge_c1s1', begin=flow_start, end=flow_end,
                            density=flow_values[15], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_s2w', from_edge='edge_s2c2', to_edge='edge_c1w', begin=flow_start, end=flow_end,
                            density=flow_values[16], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_s2n1', from_edge='edge_s2c2', to_edge='edge_c1n1', begin=flow_start, end=flow_end,
                            density=flow_values[17], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_s2n2', from_edge='edge_s2c2', to_edge='edge_c2n2', begin=flow_start, end=flow_end,
                            density=flow_values[18], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_s2e', from_edge='edge_s2c2', to_edge='edge_c2e', begin=flow_start, end=flow_end,
                            density=flow_values[19], v_type='car0', distribution='binomial')

            # From south 1
            flows.add_flow(id=f'{i}_flow_s1w', from_edge='edge_s1c1', to_edge='edge_c1w', begin=flow_start, end=flow_end,
                            density=flow_values[20], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_s1n1', from_edge='edge_s1c1', to_edge='edge_c1n1', begin=flow_start, end=flow_end,
                            density=flow_values[21], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_s1n2', from_edge='edge_s1c1', to_edge='edge_c2n2', begin=flow_start, end=flow_end,
                            density=flow_values[22], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_s1e', from_edge='edge_s1c1', to_edge='edge_c2e', begin=flow_start, end=flow_end,
                            density=flow_values[23], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_s1s2', from_edge='edge_s1c1', to_edge='edge_c2s2', begin=flow_start, end=flow_end,
                            density=flow_values[24], v_type='car0', distribution='binomial')

            # From west
            flows.add_flow(id=f'{i}_flow_wn1', from_edge='edge_wc1', to_edge='edge_c1n1', begin=flow_start, end=flow_end,
                            density=flow_values[25], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_wn2', from_edge='edge_wc1', to_edge='edge_c2n2', begin=flow_start, end=flow_end,
                            density=flow_values[26], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_we', from_edge='edge_wc1', to_edge='edge_c2e', begin=flow_start, end=flow_end,
                            density=flow_values[27], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_ws2', from_edge='edge_wc1', to_edge='edge_c2s2', begin=flow_start, end=flow_end,
                            density=flow_values[28], v_type='car0', distribution='binomial')
            flows.add_flow(id=f'{i}_flow_ws1', from_edge='edge_wc1', to_edge='edge_c1s1', begin=flow_start, end=flow_end,
                            density=flow_values[29], v_type='car0', distribution='binomial')

        return flows



    ### Detectors ###

    def generate_numerical_dectectors(self, config={}):
        """
        Generate a DetectorBuilder with a numerical detector for each lane going to an intersection.
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

        # First crossroad
        detectors.add_laneAreaDetector(id="wc1", lane="edge_wc1_0")
        detectors.add_laneAreaDetector(id="s1c1", lane="edge_s1c1_0")
        detectors.add_laneAreaDetector(id="n1c1", lane="edge_n1c1_0")
        detectors.add_laneAreaDetector(id="c2c1", lane="edge_c2c1_0")
        # Second crossroad
        detectors.add_laneAreaDetector(id="c1c2", lane="edge_c1c2_0")
        detectors.add_laneAreaDetector(id="ec2", lane="edge_ec2_0")
        detectors.add_laneAreaDetector(id="n2c2", lane="edge_n2c2_0")
        detectors.add_laneAreaDetector(id="s2c2", lane="edge_s2c2_0")

        return detectors

    def generate_boolean_detectors(self, config):
        """
        Generate a DetectorBuilder with a boolean detector for each lane going to an intersection.
        A boolean detector returns if a vehicle is on its scope or not. In SUMO, a boolean
        detector is represented with a lane area detector whose scope is the entire lane,
        from the beginning to the end.

        Valid parameters for config :
        - "lane_length" (int) : The default length for each lane (in meters)
        - "north_1_length" (int) : The north lane length (in meters) for intersection 1, override default
        - "north_2_length" (int) : The north lane length (in meters) for intersection 2, override default
        - "east_length" (int) : The east lane length (in meters), override default
        - "south_1_length" (int) : The south lane length (in meters) for intersection 1, override default
        - "south_2_length" (int) : The south lane length (in meters) for intersection 2, override default
        - "west_length" (int) : The west lane length (in meters), override default
        - "boolean_detector_length" (float) : The scope size of the detectors

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        :return: An empty DetectorBuilder object.
        :rtype: sumo_experiments.src.components.DetectorBuilder
        """

        # Get new parameters from config
        current_config = self.DEFAULT_CONFIG
        for key in config:
            if key not in self.CONFIG_PARAMETER_LIST:
                warnings.warn(f"The config parameter '{key}' is not a valid parameter.", stacklevel=2)
            current_config[key] = config[key]

        # Select parameters
        boolean_detector_length = current_config['boolean_detector_length']
        len_north_1 = current_config['north_1_length'] if 'north_1_length' in current_config else current_config['lane_length']
        len_north_2 = current_config['north_2_length'] if 'north_2_length' in current_config else current_config['lane_length']
        len_south_1 = current_config['south_1_length'] if 'south_1_length' in current_config else current_config['lane_length']
        len_south_2 = current_config['south_2_length'] if 'south_2_length' in current_config else current_config['lane_length']
        len_east = current_config['east_length'] if 'east_length' in current_config else current_config['lane_length']
        len_west = current_config['west_length'] if 'west_length' in current_config else current_config['lane_length']
        len_center = current_config['center_length'] if 'west_length' in current_config else current_config['lane_length']

        detectors = DetectorBuilder()

        # First crossroad
        detectors.add_laneAreaDetector(id="wc1", lane="edge_wc1_0", pos=(len_west - boolean_detector_length - 7.2))
        detectors.add_laneAreaDetector(id="s1c1", lane="edge_s1c1_0", pos=(len_south_1 - boolean_detector_length - 7.2))
        detectors.add_laneAreaDetector(id="n1c1", lane="edge_n1c1_0", pos=(len_north_1 - boolean_detector_length - 7.2))
        detectors.add_laneAreaDetector(id="c2c1", lane="edge_c2c1_0", pos=(len_center - boolean_detector_length - 7.2 * 2))
        # Second crossroad
        detectors.add_laneAreaDetector(id="c1c2", lane="edge_c1c2_0", pos=(len_center - boolean_detector_length - 7.2 * 2))
        detectors.add_laneAreaDetector(id="ec2", lane="edge_ec2_0", pos=(len_east - boolean_detector_length - 7.2))
        detectors.add_laneAreaDetector(id="n2c2", lane="edge_n2c2_0", pos=(len_north_2 - boolean_detector_length - 7.2))
        detectors.add_laneAreaDetector(id="s2c2", lane="edge_s2c2_0", pos=(len_south_2 - boolean_detector_length - 7.2))

        return detectors




    ### Strategies ###

    def boolean_detection(self, config={}):
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

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        """

        # Get new parameters from config
        current_config = self.DEFAULT_CONFIG
        for key in config:
            current_config[key] = config[key]

        # Select parameters
        min_duration_tl = current_config["min_duration_tl"]
        max_duration_tl = current_config["max_duration_tl"]
        simulation_duration = current_config['simulation_duration']

        step = 0
        cooldown_step_tl_1 = 0  # Current phase duration for intersection 1
        cooldown_step_tl_2 = 0  # Current phase duration for intersection 2

        while step < simulation_duration:

            traci.simulationStep()

            # First intersection
            if cooldown_step_tl_1 > min_duration_tl:
                if traci.trafficlight.getPhase("c1") == self.GREEN_LIGHT_HORIZONTAL_1:
                    quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(
                        "n1c1") >= 1 or traci.lanearea.getLastStepVehicleNumber("s1c1") >= 1)
                    autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(
                        "wc1") == 0 and traci.lanearea.getLastStepVehicleNumber("c2c1") == 0)
                    if (quelqun_en_attente and autre_voie_vide) or cooldown_step_tl_1 > max_duration_tl:
                        traci.trafficlight.setPhase("c1", self.GREEN_LIGHT_HORIZONTAL_1 + 1)  # Passage au orange
                        cooldown_step_tl_1 = 0

                elif traci.trafficlight.getPhase("c1") == self.GREEN_LIGHT_VERTICAL_1:
                    quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(
                        "wc1") >= 1 or traci.lanearea.getLastStepVehicleNumber("c2c1") >= 1)
                    autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(
                        "n1c1") == 0 and traci.lanearea.getLastStepVehicleNumber("s1c1") == 0)
                    if (quelqun_en_attente and autre_voie_vide) or cooldown_step_tl_1 > max_duration_tl:
                        traci.trafficlight.setPhase("c1", self.GREEN_LIGHT_VERTICAL_1 + 1)
                        cooldown_step_tl_1 = 0

            # Second intersection
            if cooldown_step_tl_2 > min_duration_tl:
                if traci.trafficlight.getPhase("c2") == self.GREEN_LIGHT_HORIZONTAL_2:
                    quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(
                        "n2c2") >= 1 or traci.lanearea.getLastStepVehicleNumber("s2c2") >= 1)
                    autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(
                        "ec2") == 0 and traci.lanearea.getLastStepVehicleNumber("c1c2") == 0)
                    if (quelqun_en_attente and autre_voie_vide) or cooldown_step_tl_2 > max_duration_tl:
                        traci.trafficlight.setPhase("c2", self.GREEN_LIGHT_HORIZONTAL_2 + 1)
                        cooldown_step_tl_2 = 0

                elif traci.trafficlight.getPhase("c2") == self.GREEN_LIGHT_VERTICAL_2:
                    quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(
                        "ec2") >= 1 or traci.lanearea.getLastStepVehicleNumber("c1c2") >= 1)
                    autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(
                        "n2c2") == 0 and traci.lanearea.getLastStepVehicleNumber("s2c2") == 0)
                    if (quelqun_en_attente and autre_voie_vide) or cooldown_step_tl_2 > max_duration_tl:
                        traci.trafficlight.setPhase("c2", self.GREEN_LIGHT_VERTICAL_2 + 1)
                        cooldown_step_tl_2 = 0

            step += 1
            cooldown_step_tl_1 += 1
            cooldown_step_tl_2 += 1

    def numerical_detection_all_vehicles(self, config={}):
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

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        """

        # Get new parameters from config
        current_config = self.DEFAULT_CONFIG
        for key in config:
            current_config[key] = config[key]

        # Select parameters
        min_duration_tl = current_config["min_duration_tl"]
        max_duration_tl = current_config["max_duration_tl"]
        vehicle_threshold = current_config["vehicle_threshold"]
        simulation_duration = current_config['simulation_duration']

        step = 0
        cooldown_step_tl_1 = 0  # Current phase duration for intersection 1
        cooldown_step_tl_2 = 0  # Current phase duration for intersection 2

        while step < simulation_duration:
            traci.simulationStep()

            # Premier carrefour
            if cooldown_step_tl_1 > min_duration_tl:
                if traci.trafficlight.getPhase("c1") == self.GREEN_LIGHT_VERTICAL_1:
                    if traci.lanearea.getLastStepVehicleNumber(
                            "wc1") >= vehicle_threshold or traci.lanearea.getLastStepVehicleNumber(
                            "c2c1") >= vehicle_threshold or cooldown_step_tl_1 > max_duration_tl:
                        traci.trafficlight.setPhase("c1", self.GREEN_LIGHT_VERTICAL_1 + 1)  # Passage au orange
                        cooldown_step_tl_1 = 0

                elif traci.trafficlight.getPhase("c1") == self.GREEN_LIGHT_HORIZONTAL_1:
                    if traci.lanearea.getLastStepVehicleNumber(
                            "s1c1") >= vehicle_threshold or traci.lanearea.getLastStepVehicleNumber(
                            "n1c1") >= vehicle_threshold or cooldown_step_tl_1 > max_duration_tl:
                        traci.trafficlight.setPhase("c1", self.GREEN_LIGHT_HORIZONTAL_1 + 1)
                        cooldown_step_tl_1 = 0

            # Deuxième carrefour
            if cooldown_step_tl_2 > min_duration_tl:
                if traci.trafficlight.getPhase("c2") == self.GREEN_LIGHT_VERTICAL_2:
                    if traci.lanearea.getLastStepVehicleNumber(
                            "c1c2") >= vehicle_threshold or traci.lanearea.getLastStepVehicleNumber(
                            "ec2") >= vehicle_threshold or cooldown_step_tl_2 > max_duration_tl:
                        traci.trafficlight.setPhase("c2", self.GREEN_LIGHT_VERTICAL_2 + 1)
                        cooldown_step_tl_2 = 0

                elif traci.trafficlight.getPhase("c2") == self.GREEN_LIGHT_HORIZONTAL_1:
                    if traci.lanearea.getLastStepVehicleNumber(
                            "s2c2") >= vehicle_threshold or traci.lanearea.getLastStepVehicleNumber(
                            "n2c2") >= vehicle_threshold or cooldown_step_tl_2 > max_duration_tl:
                        traci.trafficlight.setPhase("c2", self.GREEN_LIGHT_HORIZONTAL_1 + 1)
                        cooldown_step_tl_2 = 0

            step += 1
            cooldown_step_tl_1 += 1
            cooldown_step_tl_2 += 1

    def numerical_detections_stopped_vehicles(self, config={}):
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

        :param config: Customized flows configuration. Check documentation to see all parameters.
        :type config: dict
        """

        # Get new parameters from config
        current_config = self.DEFAULT_CONFIG
        for key in config:
            current_config[key] = config[key]

        # Select parameters
        min_duration_tl = current_config["min_duration_tl"]
        max_duration_tl = current_config["max_duration_tl"]
        vehicle_threshold = current_config["vehicle_threshold"]
        simulation_duration = current_config['simulation_duration']

        step = 0
        cooldown_step_tl_1 = 0  # Nombre de pas depuis la dernière actualisation du feu 1
        cooldown_step_tl_2 = 0  # Nombre de pas depuis la dernière actualisation du feu 2

        while step < simulation_duration:
            traci.simulationStep()

            # Premier carrefour
            if cooldown_step_tl_1 > min_duration_tl:
                if traci.trafficlight.getPhase("c1") == self.GREEN_LIGHT_VERTICAL_1:
                    if traci.lanearea.getJamLengthVehicle("wc1") >= vehicle_threshold or traci.lanearea.getJamLengthVehicle(
                            "c2c1") >= vehicle_threshold or cooldown_step_tl_1 > max_duration_tl:
                        traci.trafficlight.setPhase("c1", self.GREEN_LIGHT_VERTICAL_1 + 1)  # Passage au orange
                        cooldown_step_tl_1 = 0

                elif traci.trafficlight.getPhase("c1") == self.GREEN_LIGHT_HORIZONTAL_1:
                    if traci.lanearea.getJamLengthVehicle("s1c1") >= vehicle_threshold or traci.lanearea.getJamLengthVehicle(
                            "n1c1") >= vehicle_threshold or cooldown_step_tl_1 > max_duration_tl:
                        traci.trafficlight.setPhase("c1", self.GREEN_LIGHT_HORIZONTAL_1 + 1)
                        cooldown_step_tl_1 = 0

            # Deuxième carrefour
            if cooldown_step_tl_2 > min_duration_tl:
                if traci.trafficlight.getPhase("c2") == self.GREEN_LIGHT_VERTICAL_2:
                    if traci.lanearea.getJamLengthVehicle("c1c2") >= vehicle_threshold or traci.lanearea.getJamLengthVehicle(
                            "ec2") >= vehicle_threshold or cooldown_step_tl_2 > max_duration_tl:
                        traci.trafficlight.setPhase("c2", self.GREEN_LIGHT_VERTICAL_2 + 1)
                        cooldown_step_tl_2 = 0

                elif traci.trafficlight.getPhase("c2") == self.GREEN_LIGHT_HORIZONTAL_2:
                    if traci.lanearea.getJamLengthVehicle("s2c2") >= vehicle_threshold or traci.lanearea.getJamLengthVehicle(
                            "n2c2") >= vehicle_threshold or cooldown_step_tl_2 > max_duration_tl:
                        traci.trafficlight.setPhase("c2", self.GREEN_LIGHT_HORIZONTAL_2 + 1)  # WE orange, SN red, then normal transition to phase 0
                        cooldown_step_tl_2 = 0

            step += 1
            cooldown_step_tl_1 += 1
            cooldown_step_tl_2 += 1