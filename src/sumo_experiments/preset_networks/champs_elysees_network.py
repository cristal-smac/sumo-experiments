import os
import random
import sys
import xml.etree.ElementTree as ET

from sumo_experiments.components import FlowBuilder, DetectorBuilder
import networkx as nx
import libsumo as traci

from sumo_experiments.preset_networks import Network


class ChampsElyseesNetwork(Network):
    """
    Create the SUMO network and flows for the Champs-Elysées.
    """

    THIS_FILE_PATH = os.path.abspath(os.path.dirname(__file__))

    FLOWS = {
        # From Etoile
        ('642901742#0', '-4293992#19'): 8,  # Étoile - George V
        ('642901742#0', '-365158944#4'): 86, # Etoile - La Boétie
        ('642901742#0', '4294014#2'): 24, # Etoile - Marignan
        ('642901742#0', '1087767628#1'): 787, # Etoile - Rond Point
        ('642901742#0', '27245904#2'): 43,  # Etoile - Colisée
        ('642901742#0', '4941940#4'): 4,  # Etoile - Berri
        ('642901742#0', '4941938#2'): 28,  # Etoile - Washington
        # From Rond-Point
        ('-1087767628#3', '27245904#2'): 5,  # Rond Point - Colisée
        ('-1087767628#3', '4941940#4'): 36,  # Rond Point - Berri
        ('-1087767628#3', '4941938#2'): 48,  # Rond Point - Washington
        ('-1087767628#3', '-642901742#0'): 496,  # Rond Point - Etoile
        ('-1087767628#3', '-4293992#19'): 74,  # Rond Point - George V
        ('-1087767628#3', '-365158944#4'): 65,  # Rond Point - La Boétie
        ('-1087767628#3', '4294014#2'): 5,  # Rond Point - Marignan
        # Small entries to Etoile
        ('1089398064#1', '-642901742#0'): 4,  # Galilée - Etoile
        ('4293992#19', '-642901742#0'): 6,  # George V - Etoile
        ('69114043#0', '-642901742#0'): 4,  # Bauchart - Etoile
        ('365158944#4', '-642901742#0'): 10,  # Charron - Etoile
        ('4294007#12', '-642901742#0'): 39,  # Marbeuf - Etoile
        ('374056161#1', '-642901742#0'): 9,  # La Boétie - Etoile
        ('566475371#1', '-642901742#0'): 5,  # Balzac - Etoile
        # Small entries to Rond-Point
        ('1089398064#1', '1087767628#1'): 17,  # Galilée - Rond Point
        ('4293992#19', '1087767628#1'): 16,  # George V - Rond Point
        ('69114043#0', '1087767628#1'): 23,  # Bauchart - Rond Point
        ('365158944#4', '1087767628#1'): 12,  # Charron - Rond Point
        ('4294007#12', '1087767628#1'): 10,  # Marbeuf - Rond Point
        ('374056161#1', '1087767628#1'): 22,  # La Boétie - Rond Point
        ('566475371#1', '1087767628#1'): 23,  # Balzac - Rond Point
    }

    TL_IDS = ['joinedS_5996498080_830668051_cluster_11010382373_12206024496_1534568286_1554682681_#5more',
              'joinedS_cluster_11012446359_1534568240_21660107_260928395_#3more_cluster_6931233709_830665981',
              'joinedS_9966559410_cluster_11012446339_1534591908_1757030783_5996498049_#3more',
              'joinedS_5996498052_5996498054_cluster_11012446338_12189151799_1534591840_1534591859_#5more_cluster_830665987_9966563525',
              'joinedS_5996498057_5996498065_5996498066_cluster_11010103683_11012446411_1534591802_1534591819_#8more']




    def __init__(self, intensity=1, starting_time=0, ending_time=24, seed=42):
        """
        Init of class
        :param intensity: The intensity of the normal flow. A coefficient to multiply the number of vehicle for each flow.
        :type intensity: float
        :param starting_time: The hour of the day at which the flow starts. 0 is midnight, 12 is midday and 23 is 11PM. Must be between 0 and 23.
        :type starting_time: int
        :param ending_time: The hour of the day at which the flow ends. Must be between 1 and 24, and greater than starting time.
        :type ending_time: int
        """
        self.exp_name = random.randint(0, 100000)
        #self.exp_name = 1
        self.CONFIG_FILE = os.path.join(self.THIS_FILE_PATH, f"champs_elysees/run.sumocfg")
        self.NEW_CONFIG_FILE = os.path.join(self.THIS_FILE_PATH, f"champs_elysees/run_{self.exp_name}.sumocfg")
        self.FULL_LINE_COMMAND = f"sumo -c {self.THIS_FILE_PATH}/champs_elysees/run_{self.exp_name}.sumocfg"
        self.FULL_LINE_COMMAND_GUI = f"sumo-gui -c {self.THIS_FILE_PATH}/champs_elysees/run_{self.exp_name}.sumocfg"
        self.NET_FILE = os.path.join(self.THIS_FILE_PATH, 'champs_elysees/champs_elysees.net.xml')
        self.ENTRIES_FILE = os.path.join(self.THIS_FILE_PATH, 'champs_elysees/liste_entrees.txt')
        self.EXITS_FILE = os.path.join(self.THIS_FILE_PATH, 'champs_elysees/liste_sorties.txt')
        self.FORBID_EXITS_FILE = os.path.join(self.THIS_FILE_PATH, 'champs_elysees/exit_forbidden.txt')
        self.FORBID_STARTING_EDGES_FILE = os.path.join(self.THIS_FILE_PATH, 'champs_elysees/starting_edges_forbidden.txt')
        self.FLOW_FILE = f'{self.THIS_FILE_PATH}/champs_elysees/champs_elysees_{self.exp_name}.rou.xml'
        self.TL_JUNCTIONS = self.get_tl_junctions()
        self.EDGES_TO_TL = self.get_edges_to_tl()
        self.EDGES_FROM_TL = self.get_edges_from_tl()
        self.GRAPH = self.net_to_graph()
        self.flows = FlowBuilder()
        self.flows.add_v_type(id='car0')
        self.generate_flows(intensity, seed)
        self.generate_detectors()
        self.generate_config_file()
        self.TLS_DETECTORS = {
            'joinedS_5996498080_830668051_cluster_11010382373_12206024496_1534568286_1554682681_#5more': self.DETECTORS_joinedS_5996498080,
            'joinedS_cluster_11012446359_1534568240_21660107_260928395_#3more_cluster_6931233709_830665981': self.DETECTORS_joinedS_cluster_11012446359,
            'joinedS_9966559410_cluster_11012446339_1534591908_1757030783_5996498049_#3more': self.DETECTORS_joinedS_9966559410,
            'joinedS_5996498052_5996498054_cluster_11012446338_12189151799_1534591840_1534591859_#5more_cluster_830665987_9966563525': self.DETECTORS_joinedS_5996498052,
            'joinedS_5996498057_5996498065_5996498066_cluster_11010103683_11012446411_1534591802_1534591819_#8more': self.DETECTORS_joinedS_5996498057,
        }


    def run(self, traci_function, simulation_duration=None, gui=False, seed=None, no_warnings=True, nb_threads=1, time_to_teleport=150):
        """
        Run the network.
        :param traci_function: The function using TraCi package and that can control infrastructures.
        :type: function
        :param gui: True to run SUMO in graphical mode. False otherwise.
        :type gui: bool
        :param seed: The seed of the simulation. Same seeds = same simulations.
        :type seed: int
        :param no_warnings: If set to True, no warnings when executing SUMO.
        :type no_warnings: bool
        :param nb_threads: Number of thread to run SUMO
        :type nb_threads: int
        :param time_to_teleport: The time for a vehicle to teleport when the network is blocked
        :type time_to_teleport: int
        """
        try:
            if seed is not None:
                seed_text = f'--seed {seed} '
            else:
                seed_text = '--random '
            threads_text = f'--threads {nb_threads} '
            no_warnings_text = ''
            if no_warnings:
                no_warnings_text = '--no-warnings '
            if gui:
                traci.start((self.FULL_LINE_COMMAND_GUI + f' --time-to-teleport {time_to_teleport} ' + threads_text + seed_text + no_warnings_text).split())
            else:
                traci.start((self.FULL_LINE_COMMAND + f' --time-to-teleport {time_to_teleport} ' + threads_text + seed_text + no_warnings_text).split())
            res = traci_function(traci)
            traci.close()
        except Exception as err:
            print("Error during simulation :", sys.exc_info()[0])
            print("OS error: {0}".format(err))
            res = None
        self.clean_files()
        return res

    def generate_flows(self, intensity=1, seed=42):
        """
        Generate flows for the network.
        :param intensity: The intensity of the normal flow. A coefficient to multiply the number of vehicle for each flow.
        :type intensity: float
        :param seed: The seed of the random generator to create flows
        :type seed: int
        :return: The flows
        :rtype: FlowBuilder
        """
        cpt = 0
        for route in self.FLOWS:
            entry = route[0]
            exit = route[1]
            freq = self.FLOWS[route] * intensity#) / 3600
            self.flows.add_flow(id=str(cpt),
                                begin=0,
                                end=3600,
                                from_edge=entry,
                                to_edge=exit,
                                frequency=freq,
                                v_type='car0',
                                distribution='binomial')
            cpt += 1
        self.flows.build({'routes': self.FLOW_FILE})
        return self.flows


    def generate_config_file(self):
        """
        Create a new config file for the bologna network.
        """
        if os.path.isfile(self.NEW_CONFIG_FILE):
            os.remove(self.NEW_CONFIG_FILE)
        with open(self.CONFIG_FILE, 'r') as config_file:
            with open(self.NEW_CONFIG_FILE, 'a') as new_config_file:
                line = config_file.readline()
                while line != '':
                    if line != '\n' and line.split()[0] == '<route-files':
                        first_part = line.split('.')[0]
                        second_part = line.split('.')[1:]
                        first_part = ['"'.join(first_part.split('"')[:-1] + [f"champs_elysees_{self.exp_name}"])]
                        new_line = '.'.join(first_part + second_part)
                        new_config_file.write(new_line)
                    else:
                        new_config_file.write(line)
                    line = config_file.readline()


    def clean_files(self):
        """
        Delete all the files generated by the instance.
        """
        if os.path.isfile(self.NEW_CONFIG_FILE):
            os.remove(self.NEW_CONFIG_FILE)
        if os.path.isfile(self.FLOW_FILE):
            os.remove(self.FLOW_FILE)


    def get_tl_junctions(self):
        """
        Get all junctions managed by a traffic light.
        :return: The list of all junctions managed by a traffic light
        :rtype: list
        """
        tree = ET.parse(self.NET_FILE)
        junctions = tree.iter('junction')
        traffic_lights = []
        for junction in junctions:
            if junction.get('type') == 'traffic_light':
                traffic_lights.append(junction)
        return traffic_lights

    def get_edges_to_tl(self):
        """
        Get all edges ending in a traffic light.
        :return: The edges ending into each traffic light node
        :rtype: dict
        """
        tree = ET.parse(self.NET_FILE)
        edges = tree.iter('edge')
        tl_to_edges = {}
        for junction in self.TL_JUNCTIONS:
            tl_to_edges[junction.get('id')] = []
        for edge in edges:
            if edge.get('to') in tl_to_edges:
                tl_to_edges[edge.get('to')].append(edge)
        return tl_to_edges

    def get_edges_from_tl(self):
        """
        Get all edges starting from a traffic light.
        :return: The edges strating from each traffic light node
        :rtype: dict
        """
        tree = ET.parse(self.NET_FILE)
        edges = tree.iter('edge')
        tl_to_edges = {}
        for junction in self.TL_JUNCTIONS:
            tl_to_edges[junction.get('id')] = []
        for edge in edges:
            if edge.get('from') in tl_to_edges:
                tl_to_edges[edge.get('from')].append(edge)
        return tl_to_edges


    def get_entries(self):
        """
        Return the list of all the id of the entry edges of the network.
        :return: The list of all the entries of the network.
        :rtype: list
        """
        nodes_id = []
        with open(self.ENTRIES_FILE, 'r') as f:
            id = f.readline()[:-1]
            while id != "":
                nodes_id.append(id)
                id = f.readline()[:-1]
        tree = ET.parse(self.NET_FILE)
        edges = tree.iter('edge')
        entries_id = []
        for edge in edges:
            if edge.get('from') in nodes_id:
                entries_id.append(edge)
        return entries_id

    def get_exits(self):
        """
        Return the list of all the id of the exit edges of the network.
        :return: The list of all the exits of the network.
        :rtype: list
        """
        nodes_id = []
        with open(self.EXITS_FILE, 'r') as f:
            id = f.readline()[:-1]
            while id != "":
                nodes_id.append(id)
                id = f.readline()[:-1]
        tree = ET.parse(self.NET_FILE)
        edges = tree.iter('edge')
        exits_id = []
        for edge in edges:
            if edge.get('to') in nodes_id:
                exits_id.append(edge)
        return exits_id

    def get_forbidden_exits(self):
        """
        Return the list of all the id of the forbidden exits edges of the network.
        :return: The list of all the forbidden exits of the network.
        :rtype: list
        """
        edges_id = []
        with open(self.FORBID_EXITS_FILE, 'r') as f:
            id = f.readline()[:-1]
            while id != "":
                edges_id.append(id)
                id = f.readline()[:-1]
        return edges_id

    def get_forbidden_starts(self):
        """
        Return the list of all the id of the forbidden starting edges of the network.
        :return: The list of all the forbidden starting edges of the network.
        :rtype: list
        """
        edges_id = []
        with open(self.FORBID_STARTING_EDGES_FILE, 'r') as f:
            id = f.readline()
            while id != "":
                edges_id.append(id)
                id = f.readline()
        return edges_id

    def net_to_graph(self):
        """
        Convert the network in xml format to a graph where edges are the nodes, and connections are the links.
        :return: The graph representation of the network
        :rtype: networkx.Graph
        """
        G = nx.DiGraph()
        tree = ET.parse(self.NET_FILE)
        edges = tree.iter('edge')
        for edge in edges:
            G.add_node(edge.get('id'))
        connections = tree.iter('connection')
        for connection in connections:
            G.add_edge(connection.get('from'), connection.get('to'))
        return G


    def generate_detectors(self):
        """
        Generate the detectors of the network.
        :return: The detectors of the network
        :rtype: DetectorBuilder
        """
        det = DetectorBuilder()
        # TLS joinedS_5996498080
        det.add_lane_area_detector(id='joinedS_5996498080_b1', edge='566475371#3', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498080_b2', edge='566475371#3', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498080_b3', edge='566475371#2', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498080_b4', edge='566475371#2', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498080_b5', edge='566475371#1', lane=0, pos=73, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498080_b6', edge='566475371#1', lane=1, pos=73, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498080_b7', edge='-626668696#7', lane=0, pos=39, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498080_b8', edge='-626668696#7', lane=1, pos=39, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498080_b9', edge='-626668696#7', lane=2, pos=39, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498080_b10', edge='-626668696#7', lane=3, pos=39, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498080_b11', edge='841484527#1', lane=0, pos=26, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498080_b12', edge='626668696#0', lane=0, pos=23, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498080_b13', edge='626668696#0', lane=1, pos=23, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498080_b14', edge='626668696#0', lane=2, pos=23, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498080_s1', edge='566475371#1', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498080_s2', edge='566475371#1', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498080_s3', edge='-643131004#1', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498080_s4', edge='-643131004#1', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498080_s5', edge='-643131004#1', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498080_s6', edge='-643131004#1', lane=3, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498080_s7', edge='1089398064#1', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498080_s8', edge='642901742#0', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498080_s9', edge='642901742#0', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498080_s10', edge='642901742#0', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498080_n1', edge='566475371#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n2', edge='566475371#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n3', edge='566475371#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n4', edge='566475371#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n5', edge='566475371#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n6', edge='566475371#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n7', edge='-626668696#7', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n8', edge='-626668696#7', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n9', edge='-626668696#7', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n10', edge='-626668696#7', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n11', edge='-643131004#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n12', edge='-643131004#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n13', edge='-643131004#1', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n14', edge='-643131004#1', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n15', edge='841484527#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n16', edge='1089398064#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n17', edge='626668696#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n18', edge='626668696#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n19', edge='626668696#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n20', edge='642901742#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n21', edge='642901742#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n22', edge='642901742#1', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n23', edge='642901742#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n24', edge='642901742#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498080_n25', edge='642901742#0', lane=2, end_pos=-1, type='numerical')
        # joinedS_cluster_11012446359
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_b1', edge='-642910715#4', lane=0, pos=47, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_b2', edge='-642910715#4', lane=1, pos=47, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_b3', edge='-642910715#4', lane=2, pos=47, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_b4', edge='1421892863#4', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_b5', edge='1421892863#1', lane=0, pos=40, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_b6', edge='643131004#0', lane=0, pos=9, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_b7', edge='643131004#0', lane=1, pos=9, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_b8', edge='643131004#0', lane=2, pos=9, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_s1', edge='-642910715#5', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_s2', edge='-642910715#5', lane=1, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_s3', edge='-642910715#5', lane=2, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_s4', edge='4293992#19', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_s5', edge='626668696#5', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_s6', edge='626668696#5', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_s7', edge='626668696#5', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n1', edge='-642910715#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n2', edge='-642910715#4', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n3', edge='-642910715#4', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n4', edge='-642910715#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n5', edge='-642910715#5', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n6', edge='-642910715#5', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n7', edge='1421892863#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n8', edge='1421892863#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n9', edge='1421892863#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n10', edge='4293992#20', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n11', edge='4293992#19', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n12', edge='643131004#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n13', edge='643131004#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n14', edge='643131004#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n15', edge='626668696#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n16', edge='626668696#5', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_cluster_11012446359_n17', edge='626668696#5', lane=2, end_pos=-1, type='numerical')
        # joinedS_9966559410
        det.add_lane_area_detector(id='joinedS_9966559410_b1', edge='-642910715#9', lane=0, pos=42, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9966559410_b2', edge='-642910715#9', lane=1, pos=42, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9966559410_b3', edge='-642910715#9', lane=2, pos=42, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9966559410_b4', edge='69114043#4', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9966559410_b5', edge='69114043#2', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9966559410_b6', edge='69114043#0', lane=0, pos=67, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9966559410_b7', edge='642910715#5', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9966559410_b8', edge='642910715#5', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9966559410_b9', edge='642910715#5', lane=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9966559410_b10', edge='642910715#5', lane=3, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9966559410_b11', edge='642910715#2', lane=0, pos=63, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9966559410_b12', edge='642910715#2', lane=1, pos=63, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9966559410_b13', edge='642910715#2', lane=2, pos=63, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9966559410_b14', edge='642910715#2', lane=3, pos=63, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9966559410_s1', edge='-643134324#4', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_9966559410_s2', edge='-643134324#4', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_9966559410_s3', edge='-643134324#4', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_9966559410_s4', edge='69114043#0', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_9966559410_s5', edge='642910715#2', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_9966559410_s6', edge='642910715#2', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_9966559410_s7', edge='642910715#2', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_9966559410_s8', edge='642910715#2', lane=3, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_9966559410_n1', edge='-642910715#9', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n2', edge='-642910715#9', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n3', edge='-642910715#9', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n4', edge='-643134324#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n5', edge='-643134324#4', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n6', edge='-643134324#4', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n7', edge='69114043#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n8', edge='69114043#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n9', edge='69114043#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n10', edge='642910715#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n11', edge='642910715#5', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n12', edge='642910715#5', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n13', edge='642910715#5', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n14', edge='642910715#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n15', edge='642910715#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n16', edge='642910715#2', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9966559410_n17', edge='642910715#2', lane=3, end_pos=-1, type='numerical')
        # joinedS_5996498052
        det.add_lane_area_detector(id='joinedS_5996498052_b1', edge='374056161#1', lane=0, pos=122, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498052_b2', edge='374056161#1', lane=1, pos=122, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498052_b3', edge='-642913230#3', lane=0, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498052_b4', edge='-642913230#3', lane=1, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498052_b5', edge='-642913230#3', lane=2, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498052_b6', edge='-642913230#3', lane=3, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498052_b7', edge='365158944#9', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498052_b8', edge='365158944#6', lane=0, pos=10, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498052_b9', edge='643134324#2', lane=0, pos=85, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498052_b10', edge='643134324#2', lane=1, pos=85, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498052_b11', edge='643134324#2', lane=2, pos=85, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498052_b12', edge='643134324#2', lane=3, pos=85, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498052_s1', edge='374056161#1', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498052_s2', edge='374056161#1', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498052_s3', edge='-642913230#3', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498052_s4', edge='-642913230#3', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498052_s5', edge='-642913230#3', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498052_s6', edge='-642913230#3', lane=3, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498052_s7', edge='365158944#4', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498052_s8', edge='642910715#8', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498052_s9', edge='642910715#8', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498052_s10', edge='642910715#8', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498052_s11', edge='642910715#8', lane=3, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498052_n1', edge='374056161#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498052_n2', edge='374056161#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498052_n3', edge='-642913230#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498052_n4', edge='-642913230#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498052_n5', edge='-642913230#3', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498052_n6', edge='-642913230#3', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498052_n7', edge='365158944#9', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498052_n8', edge='365158944#6', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498052_n9', edge='365158944#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498052_n10', edge='365158944#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498052_n11', edge='643134324#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498052_n12', edge='643134324#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498052_n13', edge='643134324#2', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498052_n14', edge='643134324#2', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5996498052_n15', edge='642910715#8', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498052_n16', edge='642910715#8', lane=1, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498052_n17', edge='642910715#8', lane=2, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498052_n18', edge='642910715#8', lane=3, end_pos=-1, type='saturation')
        # joinedS_5996498057
        det.add_lane_area_detector(id='joinedS_5996498057_b1', edge='27245904#2', lane=0, pos=132, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_b2', edge='-1087767628#3', lane=0, pos=158, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_b3', edge='-1087767628#3', lane=1, pos=158, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_b4', edge='-1087767628#3', lane=2, pos=158, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_b5', edge='-1087767628#3', lane=3, pos=158, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_b6', edge='4294007#12', lane=0, pos=122, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_b7', edge='642913230#1', lane=0, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_b8', edge='642913230#1', lane=1, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_b9', edge='642913230#1', lane=2, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_b10', edge='642913230#1', lane=3, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_s1', edge='27245904#2', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498057_s2', edge='-1087767628#3', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498057_s3', edge='-1087767628#3', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498057_s4', edge='-1087767628#3', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498057_s5', edge='-1087767628#3', lane=3, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498057_s6', edge='4294007#12', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498057_s7', edge='642913230#1', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498057_s8', edge='642913230#1', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498057_s9', edge='642913230#1', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498057_s10', edge='642913230#1', lane=3, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5996498057_n1', edge='27245904#2', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_n2', edge='-1087767628#3', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_n3', edge='-1087767628#3', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_n4', edge='-1087767628#3', lane=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_n5', edge='-1087767628#3', lane=3, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_n6', edge='4294007#12', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_n7', edge='642913230#1', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_n8', edge='642913230#1', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_n9', edge='642913230#1', lane=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5996498057_n10', edge='642913230#1', lane=3, end_pos=-1, type='boolean')

        det.build({'detectors': os.path.join(self.THIS_FILE_PATH,'champs_elysees/champs_elysees_detectors.add.xml')})
        return det

    DETECTORS_joinedS_5996498080 = {
        0: {
            'boolean': ['joinedS_5996498080_b7', 'joinedS_5996498080_b8', 'joinedS_5996498080_b9', 'joinedS_5996498080_b10', 'joinedS_5996498080_b12', 'joinedS_5996498080_b13', 'joinedS_5996498080_b14'],
            'saturation': ['joinedS_5996498080_s3', 'joinedS_5996498080_s4', 'joinedS_5996498080_s5', 'joinedS_5996498080_s6', 'joinedS_5996498080_s8', 'joinedS_5996498080_s9', 'joinedS_5996498080_s10'],
            'numerical': ['joinedS_5996498080_n7', 'joinedS_5996498080_n8', 'joinedS_5996498080_n9', 'joinedS_5996498080_n10', 'joinedS_5996498080_n11', 'joinedS_5996498080_n12', 'joinedS_5996498080_n13', 'joinedS_5996498080_n14', 'joinedS_5996498080_n17', 'joinedS_5996498080_n18', 'joinedS_5996498080_n19', 'joinedS_5996498080_n20', 'joinedS_5996498080_n21', 'joinedS_5996498080_n22', 'joinedS_5996498080_n23', 'joinedS_5996498080_n24', 'joinedS_5996498080_n25'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_5996498080_b1', 'joinedS_5996498080_b2', 'joinedS_5996498080_b3', 'joinedS_5996498080_b4', 'joinedS_5996498080_b5', 'joinedS_5996498080_b6', 'joinedS_5996498080_b11'],
            'saturation': ['joinedS_5996498080_s1', 'joinedS_5996498080_s2', 'joinedS_5996498080_s7'],
            'numerical': ['joinedS_5996498080_n1', 'joinedS_5996498080_n2', 'joinedS_5996498080_n3', 'joinedS_5996498080_n4', 'joinedS_5996498080_n5', 'joinedS_5996498080_n6', 'joinedS_5996498080_n15', 'joinedS_5996498080_n16'],
            'exit': []
        }
    }

    DETECTORS_joinedS_cluster_11012446359 = {
        0: {
            'boolean': ['joinedS_cluster_11012446359_b1', 'joinedS_cluster_11012446359_b2', 'joinedS_cluster_11012446359_b3', 'joinedS_cluster_11012446359_b6', 'joinedS_cluster_11012446359_b7', 'joinedS_cluster_11012446359_b8'],
            'saturation': ['joinedS_cluster_11012446359_s1', 'joinedS_cluster_11012446359_s2', 'joinedS_cluster_11012446359_s3', 'joinedS_cluster_11012446359_s5', 'joinedS_cluster_11012446359_s6', 'joinedS_cluster_11012446359_s7'],
            'numerical': ['joinedS_cluster_11012446359_n1', 'joinedS_cluster_11012446359_n2', 'joinedS_cluster_11012446359_n3', 'joinedS_cluster_11012446359_n4', 'joinedS_cluster_11012446359_n5', 'joinedS_cluster_11012446359_n6', 'joinedS_cluster_11012446359_n12', 'joinedS_cluster_11012446359_n13', 'joinedS_cluster_11012446359_n14', 'joinedS_cluster_11012446359_n15', 'joinedS_cluster_11012446359_n16', 'joinedS_cluster_11012446359_n17'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_cluster_11012446359_b4', 'joinedS_cluster_11012446359_b5'],
            'saturation': ['joinedS_cluster_11012446359_s4'],
            'numerical': ['joinedS_cluster_11012446359_n7', 'joinedS_cluster_11012446359_n8', 'joinedS_cluster_11012446359_n9', 'joinedS_cluster_11012446359_n10', 'joinedS_cluster_11012446359_n11'],
            'exit': []
        }
    }

    DETECTORS_joinedS_9966559410 = {
        0: {
            'boolean': ['joinedS_9966559410_b1', 'joinedS_9966559410_b2', 'joinedS_9966559410_b3', 'joinedS_9966559410_b7', 'joinedS_9966559410_b8', 'joinedS_9966559410_b9', 'joinedS_9966559410_b10', 'joinedS_9966559410_b11', 'joinedS_9966559410_b12', 'joinedS_9966559410_b13', 'joinedS_9966559410_b14'],
            'saturation': ['joinedS_9966559410_s1', 'joinedS_9966559410_s2', 'joinedS_9966559410_s3', 'joinedS_9966559410_s5', 'joinedS_9966559410_s6', 'joinedS_9966559410_s7', 'joinedS_9966559410_s8'],
            'numerical': ['joinedS_9966559410_n1', 'joinedS_9966559410_n2', 'joinedS_9966559410_n3', 'joinedS_9966559410_n4', 'joinedS_9966559410_n5', 'joinedS_9966559410_n6', 'joinedS_9966559410_n10', 'joinedS_9966559410_n11', 'joinedS_9966559410_n12', 'joinedS_9966559410_n13', 'joinedS_9966559410_n14', 'joinedS_9966559410_n15', 'joinedS_9966559410_n16', 'joinedS_9966559410_n17'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_9966559410_b4', 'joinedS_9966559410_b5', 'joinedS_9966559410_b6'],
            'saturation': ['joinedS_9966559410_s4'],
            'numerical': ['joinedS_9966559410_n7', 'joinedS_9966559410_n8', 'joinedS_9966559410_n9'],
            'exit': []
        }
    }

    DETECTORS_joinedS_5996498052 = {
        0: {
            'boolean': ['joinedS_5996498052_b3', 'joinedS_5996498052_b4', 'joinedS_5996498052_b5', 'joinedS_5996498052_b6', 'joinedS_5996498052_b9', 'joinedS_5996498052_b10', 'joinedS_5996498052_b11', 'joinedS_5996498052_b12'],
            'saturation': ['joinedS_5996498052_s3', 'joinedS_5996498052_s4', 'joinedS_5996498052_s5', 'joinedS_5996498052_s6', 'joinedS_5996498052_s8', 'joinedS_5996498052_s9', 'joinedS_5996498052_s10', 'joinedS_5996498052_s11'],
            'numerical': ['joinedS_5996498052_n3', 'joinedS_5996498052_n4', 'joinedS_5996498052_n5', 'joinedS_5996498052_n6', 'joinedS_5996498052_n11', 'joinedS_5996498052_n12', 'joinedS_5996498052_n13', 'joinedS_5996498052_n14', 'joinedS_5996498052_n15', 'joinedS_5996498052_n16', 'joinedS_5996498052_n17', 'joinedS_5996498052_n18'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_5996498052_b1', 'joinedS_5996498052_b2', 'joinedS_5996498052_b7', 'joinedS_5996498052_b8'],
            'saturation': ['joinedS_5996498052_s1', 'joinedS_5996498052_s2', 'joinedS_5996498052_s7'],
            'numerical': ['joinedS_5996498052_n1', 'joinedS_5996498052_n2', 'joinedS_5996498052_n7', 'joinedS_5996498052_n8', 'joinedS_5996498052_n9', 'joinedS_5996498052_n10'],
            'exit': []
        }
    }

    DETECTORS_joinedS_5996498057 = {
        0: {
            'boolean': ['joinedS_5996498057_b2', 'joinedS_5996498057_b3', 'joinedS_5996498057_b4', 'joinedS_5996498057_b5', 'joinedS_5996498057_b7', 'joinedS_5996498057_b8', 'joinedS_5996498057_b9', 'joinedS_5996498057_b10'],
            'saturation': ['joinedS_5996498057_s2', 'joinedS_5996498057_s3', 'joinedS_5996498057_s4', 'joinedS_5996498057_s5', 'joinedS_5996498057_s7', 'joinedS_5996498057_s8', 'joinedS_5996498057_s9', 'joinedS_5996498057_s10'],
            'numerical': ['joinedS_5996498057_n2', 'joinedS_5996498057_n3', 'joinedS_5996498057_n4', 'joinedS_5996498057_n5', 'joinedS_5996498057_n7', 'joinedS_5996498057_n8', 'joinedS_5996498057_n9', 'joinedS_5996498057_n10'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_5996498057_b1', 'joinedS_5996498057_b6'],
            'saturation': ['joinedS_5996498057_s1', 'joinedS_5996498057_s6'],
            'numerical': ['joinedS_5996498057_n1', 'joinedS_5996498057_n6'],
            'exit': []
        }
    }



