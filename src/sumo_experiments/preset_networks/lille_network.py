import os
import random
import xml.etree.ElementTree as ET
from sumo_experiments.components import FlowBuilder, DetectorBuilder
import networkx as nx


class LilleNetwork:
    """
    Create the SUMO network and flows for the city of Lille.
    """

    THIS_FILE_PATH = os.path.abspath(os.path.dirname(__file__))

    NB_ROUTES_EACH_FLOW = 100

    FLOWS_ENTRIES = { # * 3/5 car tout le territoire lillois n'est pas représenté par le réseau
        'TOURQUENNOIS_LILLE': 12600 * 3/5,
        'ROUBAISIEN_LILLE': 23100 * 3/5,
        'EST_LILLE': 32400 * 3/5,
        'SUD_LILLE': 32400 * 3/5,
        'WEPPES_LILLE': 13300 * 3/5,
        'NORD_LILLE': 45600 * 3/5,
        'LYS_LILLE': 9100 * 3/5
    }

    FLOWS_EXITS = {
        'LILLE_TOURQUENNOIS': 13300 * 3/5,
        'LILLE_ROUBAISIEN': 22400 * 3/5,
        'LILLE_EST': 33600 * 3/5,
        'LILLE_SUD': 31800 * 3/5,
        'LILLE_WEPPES': 13300 * 3/5,
        'LILLE_NORD': 44400 * 3/5,
        'LILLE_LYS': 8400 * 3/5,
    }

    PROPORTIONS_TO_TIME = [
        0.3, 0.1, 0.1, 0.1, 0.2, 0.4, 1.2, 6.4, 10.2, 4.3, 5, 6.5, 7.5, 7.6, 5, 6.7, 10, 9.5, 7.7, 5, 3, 1.5, 1, 0.7
    ]

    ENTRIES_EACH_FLOW = {
        'TOURQUENNOIS_LILLE': {'114800016#1': 0.2, '691053721-AddedOffRampEdge': 0.8},
        'ROUBAISIEN_LILLE': {'114800016#1': 0.2, '691053721-AddedOffRampEdge': 0.8},
        'EST_LILLE': {'674454570': 0.05, '32391880': 0.95},
        'SUD_LILLE': {'986419403#3': 0.1, '670522617#0': 0.45, '669441008#0': 0.45},
        'WEPPES_LILLE': {'842421984': 0.95, '986419403#3': 0.05},
        'NORD_LILLE': {'114800016#1': 0.1, '1252597600#0': 0.3, '95426996#0': 0.2, '220886686': 0.4},
        'LYS_LILLE': {'180818488#0': 0.25, '1184172235': 0.25, '220886686': 0.4, '95426996#0': 0.1}
    }

    EXITS_EACH_FLOW = {
        'LILLE_TOURQUENNOIS': {'181634862': 0.2, '326114344': 0.8},
        'LILLE_ROUBAISIEN': {'181634862': 0.2, '326114344': 0.8},
        'LILLE_EST': {'151484632': 0.05, '30834157': 0.95},
        'LILLE_SUD': {'149408034#0': 0.1, '670522615#0': 0.45, '23298866#0': 0.45},
        'LILLE_WEPPES': {'237476482': 0.95, '149408034#0': 0.05},
        'LILLE_NORD': {'181634862': 0.1, '1137309858#0': 0.3, '19906584#3': 0.2, '177547335': 0.4},
        'LILLE_LYS': {'-39437728': 0.25, '1102613333#0': 0.25, '177547335': 0.4, '19906584#3': 0.1},
    }

    NB_LANES_ENTRIES = {
        'TOURQUENNOIS_LILLE': {'114800016#1': 2, '691053721-AddedOffRampEdge': 4},
        'ROUBAISIEN_LILLE': {'114800016#1': 2, '691053721-AddedOffRampEdge': 4},
        'EST_LILLE': {'674454570': 2, '32391880': 1},
        'SUD_LILLE': {'986419403#3': 2, '670522617#0': 2, '669441008#0': 2},
        'WEPPES_LILLE': {'842421984': 3, '986419403#3': 2},
        'NORD_LILLE': {'114800016#1': 2, '1252597600#0': 2, '95426996#0': 1, '220886686': 2},
        'LYS_LILLE': {'180818488#0': 1, '1184172235': 2, '220886686': 2, '95426996#0': 1}
    }

    def __init__(self):
        """
        Init of class
        """
        self.exp_name = random.randint(0, 100000)
        self.CONFIG_FILE = os.path.join(self.THIS_FILE_PATH, f"lille/run.sumocfg")
        self.NEW_CONFIG_FILE = os.path.join(self.THIS_FILE_PATH, f"lille/run_{self.exp_name}.sumocfg")
        self.FULL_LINE_COMMAND = f"sumo -c {self.THIS_FILE_PATH}/lille/run_{self.exp_name}.sumocfg"
        self.FULL_LINE_COMMAND_GUI = f"sumo-gui -c {self.THIS_FILE_PATH}/lille/run_{self.exp_name}.sumocfg"
        self.NET_FILE = os.path.join(self.THIS_FILE_PATH, 'lille/lille.net.xml')
        self.ENTRIES_FILE = os.path.join(self.THIS_FILE_PATH, 'lille/liste_entrees.txt')
        self.EXITS_FILE = os.path.join(self.THIS_FILE_PATH, 'lille/liste_sorties.txt')
        self.FORBID_EXITS_FILE = os.path.join(self.THIS_FILE_PATH, 'lille/exit_forbidden.txt')
        self.FORBID_STARTING_EDGES_FILE = os.path.join(self.THIS_FILE_PATH, 'lille/starting_edges_forbidden.txt')
        self.FLOW_FILE = f'{self.THIS_FILE_PATH}/lille/lille_{self.exp_name}.rou.xml'
        self.TL_JUNCTIONS = self.get_tl_junctions()
        self.EDGES_TO_TL = self.get_edges_to_tl()
        self.EDGES_FROM_TL = self.get_edges_from_tl()
        self.GRAPH = self.net_to_graph()
        self.flows = FlowBuilder()
        self.flows.add_v_type(id='car0')
        self.generate_detectors()
        self.generate_config_file()

    def generate_flows(self, intensity=1, starting_time=0, ending_time=24):
        """
        Generate flows for the network.
        :param intensity: The intensity of the normal flow. A coefficient to multiply the number of vehicle for each flow.
        :type intensity: float
        :param starting_time: The hour of the day at which the flow starts. 0 is midnight, 12 is midday and 23 is 11PM. Must be between 0 and 23.
        :type starting_time: int
        :param ending_time: The hour of the day at which the flow ends. Must be between 1 and 24, and greater than starting time.
        :type ending_time: int
        :return: The flows
        :rtype: FlowBuilder
        """
        forbid_exits = self.get_forbidden_exits()
        forbid_start_edges = self.get_forbidden_starts()
        tree = ET.parse(self.NET_FILE)
        edges = [e.get('id') for e in tree.iter('edge')]
        hours = [3600 * i for i in range(ending_time + 1 - starting_time)]
        cpt = 0
        for time in range(len(self.PROPORTIONS_TO_TIME[starting_time:ending_time])):
            current_time = time + starting_time
            total_ratio = 0
            # From outside to Lille
            for flow in self.FLOWS_ENTRIES:
                for entry in self.ENTRIES_EACH_FLOW[flow]:
                    c = 0
                    while c < self.NB_ROUTES_EACH_FLOW:
                        exit = random.choices(edges)[0]
                        if exit[0] != ':' and nx.has_path(self.GRAPH, entry, exit) and exit not in forbid_exits:
                            freq = int((self.FLOWS_ENTRIES[flow] * self.PROPORTIONS_TO_TIME[current_time] * self.ENTRIES_EACH_FLOW[flow][entry]) // self.NB_ROUTES_EACH_FLOW)
                            freq *= intensity
                            total_ratio += freq
                            if freq == 0:
                                freq = 1
                            self.flows.add_flow(id=f"{cpt}",
                                                begin=hours[time],
                                                end=hours[time+1],
                                                from_edge=entry,
                                                to_edge=exit,
                                                frequency=freq,
                                                v_type='car0',
                                                distribution='binomial')
                            c += 1
                            cpt += 1
            # From Lille to outside
            for flow in self.FLOWS_EXITS:
                for exit in self.EXITS_EACH_FLOW[flow]:
                    c = 0
                    while c < self.NB_ROUTES_EACH_FLOW:
                        entry = random.choices(edges)[0]
                        while str(entry) in forbid_start_edges:
                            entry = random.choices(edges)[0]
                        if entry[0] != ':' and nx.has_path(self.GRAPH, entry, exit):
                            freq = int((self.FLOWS_EXITS[flow] * self.PROPORTIONS_TO_TIME[time] * self.EXITS_EACH_FLOW[flow][exit]) // self.NB_ROUTES_EACH_FLOW)
                            freq *= intensity
                            total_ratio += freq
                            if freq == 0:
                                freq = 1
                            self.flows.add_flow(id=f"{cpt}",
                                                begin=hours[time],
                                                end=hours[time+1],
                                                from_edge=entry,
                                                to_edge=exit,
                                                frequency=freq,
                                                v_type='car0',
                                                distribution='binomial')
                            c += 1
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
                        first_part = ['"'.join(first_part.split('"')[:-1] + [f"lille_{self.exp_name}"])]
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





    def generate_flows_intra_city(self, n):
        """
        Generate flows that start and end inside the city, and not from entry to exit.
        :param n: The number of flows to create
        :type n: int
        :return: The flows
        :rtype: FlowBuilder
        """
        tree = ET.parse(self.NET_FILE)
        edges = [e.get('id') for e in tree.iter('edge')]
        c = 0
        forbid_exits = self.get_forbidden_exits()
        while c < n:
            couple = random.choices(edges, k=2)
            if nx.has_path(self.GRAPH, couple[0], couple[1]) and couple[0][0] != ':' and couple[1][0] != ':' and couple[1] not in forbid_exits:
                self.flows.add_flow(id=f"{couple[0]}-{couple[1]}",
                                    end=3600,
                                    from_edge=couple[0],
                                    to_edge=couple[1],
                                    frequency=1,
                                    v_type='car0',
                                    distribution='binomial')
                c += 1
        return self.flows


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
        # TLS joinedS_1
        det.add_lane_area_detector(id='joinedS_1_b1', edge='95633523#0', lane=0, end_pos=12.75, type='boolean')
        det.add_lane_area_detector(id='joinedS_1_b2', edge='95633523#0', lane=1, end_pos=12.75, type='boolean')
        det.add_lane_area_detector(id='joinedS_1_b3', edge='23298867#0', lane=0, end_pos=10.32, type='boolean')
        det.add_lane_area_detector(id='joinedS_1_b4', edge='23298867#0', lane=1, end_pos=10.32, type='boolean')
        det.add_lane_area_detector(id='joinedS_1_b5', edge='155273789#0', lane=0, end_pos=14.13, type='boolean')
        det.add_lane_area_detector(id='joinedS_1_b6', edge='155273789#0', lane=1, end_pos=14.13, type='boolean')
        det.add_lane_area_detector(id='joinedS_1_s1', edge='34297833', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_1_s2', edge='34297833', lane=3, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_1_s3', edge='669441008#0', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_1_s4', edge='669441008#0', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_1_n1', edge='95633523#0', lane=0, end_pos=12.75, type='numerical')
        det.add_lane_area_detector(id='joinedS_1_n2', edge='95633523#0', lane=1, end_pos=12.75, type='numerical')
        det.add_lane_area_detector(id='joinedS_1_n3', edge='23298867#0', lane=0, end_pos=10.32, type='numerical')
        det.add_lane_area_detector(id='joinedS_1_n4', edge='23298867#0', lane=1, end_pos=10.32, type='numerical')
        det.add_lane_area_detector(id='joinedS_1_n5', edge='155273789#0', lane=0, end_pos=14.13, type='numerical')
        det.add_lane_area_detector(id='joinedS_1_n6', edge='155273789#0', lane=1, end_pos=14.13, type='numerical')
        det.add_lane_area_detector(id='joinedS_1_n7', edge='34297833', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_1_n8', edge='34297833', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_1_n9', edge='669441008#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_1_n10', edge='669441008#0', lane=1, end_pos=-1, type='numerical')
        # TLS 6301940736
        det.add_lane_area_detector(id='6301940736_b1', edge='672957493#1', lane=0, pos=67, end_pos=87, type='boolean')
        det.add_lane_area_detector(id='6301940736_b2', edge='672957493#1', lane=1, pos=67, end_pos=87, type='boolean')
        det.add_lane_area_detector(id='6301940736_b3', edge='672957495', lane=0, pos=17, end_pos=27, type='boolean')
        det.add_lane_area_detector(id='6301940736_b4', edge='672957495', lane=1, pos=17, end_pos=27, type='boolean')
        det.add_lane_area_detector(id='6301940736_b5', edge='686637175', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6301940736_b6', edge='672957492#2', lane=0, end_pos=14, type='boolean')
        det.add_lane_area_detector(id='6301940736_b7', edge='672957492#2', lane=1, end_pos=14, type='boolean')
        det.add_lane_area_detector(id='6301940736_b8', edge='1136980857#0', lane=0, pos=458, end_pos=478, type='boolean')
        det.add_lane_area_detector(id='6301940736_b9', edge='672957494', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6301940736_b10', edge='672957494', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6301940736_s1', edge='672957493#1', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6301940736_s2', edge='672957493#1', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6301940736_s3', edge='933868039#0', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6301940736_s4', edge='1136980857#0', lane=0, pos=258, end_pos=278, type='saturation')
        det.add_lane_area_detector(id='6301940736_n1', edge='672957493#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6301940736_n2', edge='672957493#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6301940736_n3', edge='672957495', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6301940736_n4', edge='672957495', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6301940736_n5', edge='686637175', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6301940736_n6', edge='933868039#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6301940736_n7', edge='672957492#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6301940736_n8', edge='672957492#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6301940736_n9', edge='1136980857#0', lane=0, pos=278, end_pos=478, type='numerical')
        det.add_lane_area_detector(id='6301940736_n10', edge='672957494', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6301940736_n11', edge='672957494', lane=1, end_pos=-1, type='numerical')
        # TLS cluster1681715927_250883340_274897939_6301940714
        det.add_lane_area_detector(id='cluster6301940714_b1', edge='155814450#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster6301940714_b2', edge='23197996#0', lane=0, pos=9, end_pos=29, type='boolean')
        det.add_lane_area_detector(id='cluster6301940714_b3', edge='23197996#0', lane=1, pos=9, end_pos=29, type='boolean')
        det.add_lane_area_detector(id='cluster6301940714_b4', edge='289351515#3', lane=0, pos=200, end_pos=220, type='boolean')
        det.add_lane_area_detector(id='cluster6301940714_b5', edge='-302704693#2', lane=0, pos=55, end_pos=75, type='boolean')
        det.add_lane_area_detector(id='cluster6301940714_b6', edge='25211576#0', lane=0, pos=77, end_pos=97, type='boolean')
        det.add_lane_area_detector(id='cluster6301940714_s1', edge='966986026', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster6301940714_s2', edge='966986026', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster6301940714_s3', edge='966986026', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster6301940714_s4', edge='289351515#3', lane=0, pos=20, end_pos=40, type='saturation')
        det.add_lane_area_detector(id='cluster6301940714_s5', edge='-302704693#2', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster6301940714_s6', edge='25211576#0', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster6301940714_n1', edge='155814450#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster6301940714_n2', edge='23197996#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster6301940714_n3', edge='23197996#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster6301940714_n4', edge='966986026', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster6301940714_n5', edge='966986026', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster6301940714_n6', edge='966986026', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster6301940714_n7', edge='289351515#3', lane=0, pos=20, end_pos=220, type='numerical')
        det.add_lane_area_detector(id='cluster6301940714_n8', edge='-302704693#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster6301940714_n9', edge='-302704693#3', lane=0, pos=19, end_pos=119, type='numerical')
        det.add_lane_area_detector(id='cluster6301940714_n10', edge='25211576#0', lane=0, end_pos=-1, type='numerical')
        # TLS 6316129114
        det.add_lane_area_detector(id='6316129114_b1', edge='231933248#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b2', edge='231933248#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b3', edge='36700826', lane=0, pos=173, end_pos=183, type='boolean')
        det.add_lane_area_detector(id='6316129114_b4', edge='36700826', lane=1, pos=173, end_pos=183, type='boolean')
        det.add_lane_area_detector(id='6316129114_b5', edge='231933252#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b6', edge='231933252#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b7', edge='231933218', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b8', edge='231933218', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b9', edge='674434835#3', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b10', edge='674434835#3', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b11', edge='674434835#3', lane=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b12', edge='295265888#0', lane=0, pos=141, end_pos=161, type='boolean')
        det.add_lane_area_detector(id='6316129114_b13', edge='295265888#0', lane=1, pos=141, end_pos=161, type='boolean')
        det.add_lane_area_detector(id='6316129114_b14', edge='935166810', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b15', edge='935166810', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b16', edge='935166810', lane=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b17', edge='295241795#0', lane=0, pos=128, end_pos=148, type='boolean')
        det.add_lane_area_detector(id='6316129114_b18', edge='295241795#0', lane=1, pos=128, end_pos=148, type='boolean')
        det.add_lane_area_detector(id='6316129114_b19', edge='674434832', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b20', edge='674434832', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b21', edge='674434832', lane=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b22', edge='267674184#7', lane=0, pos=30, end_pos=50, type='boolean')
        det.add_lane_area_detector(id='6316129114_b23', edge='267674184#7', lane=1, pos=30, end_pos=50, type='boolean')
        det.add_lane_area_detector(id='6316129114_b24', edge='674434835#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b25', edge='674434835#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_b26', edge='674434835#0', lane=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6316129114_s1', edge='36700826', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6316129114_s2', edge='36700826', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6316129114_s3', edge='122445725', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6316129114_s4', edge='122445725', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6316129114_s5', edge='295265888#0', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6316129114_s6', edge='295265888#0', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6316129114_s7', edge='295241795#0', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6316129114_s8', edge='295241795#0', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6316129114_s9', edge='267674184#5', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6316129114_s10', edge='267674184#5', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6316129114_n1', edge='231933248#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n2', edge='231933248#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n3', edge='36700826', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n4', edge='36700826', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n5', edge='231933252#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n6', edge='231933252#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n7', edge='231933218', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n8', edge='231933218', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n9', edge='231933235', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n10', edge='231933235', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n11', edge='122445725', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n12', edge='122445725', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n13', edge='674434835#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n14', edge='674434835#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n15', edge='674434835#3', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n16', edge='295265888#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n17', edge='295265888#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n18', edge='935166810', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n19', edge='935166810', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n20', edge='935166810', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n21', edge='295241795#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n22', edge='295241795#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n23', edge='674434832', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n24', edge='674434832', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n25', edge='674434832', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n26', edge='267674184#7', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n27', edge='267674184#7', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n28', edge='267674184#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n29', edge='267674184#5', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n30', edge='674434835#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n31', edge='674434835#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n32', edge='674434835#0', lane=2, end_pos=-1, type='numerical')
        # TLS GS_cluster_1615590751_3305892794
        det.add_lane_area_detector(id='cluster_1615590751_b1', edge='141068498#0', lane=0, pos=132, end_pos=152, type='boolean')
        det.add_lane_area_detector(id='cluster_1615590751_b2', edge='-448216348#1', lane=0, pos=11, end_pos=31, type='boolean')
        det.add_lane_area_detector(id='cluster_1615590751_b3', edge='167965593#4', lane=0, pos=94, end_pos=114, type='boolean')
        det.add_lane_area_detector(id='cluster_1615590751_b4', edge='167965593#4', lane=1, pos=94, end_pos=114, type='boolean')
        det.add_lane_area_detector(id='cluster_1615590751_b5', edge='-437617768#3', lane=0, pos=52, end_pos=72, type='boolean')
        det.add_lane_area_detector(id='cluster_1615590751_s1', edge='141068498#0', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1615590751_s2', edge='41235312#0', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1615590751_s3', edge='167965593#4', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1615590751_s4', edge='167965593#4', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1615590751_s5', edge='148460335#0', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1615590751_n1', edge='141068498#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1615590751_n2', edge='-448216348#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1615590751_n3', edge='-448216348#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1615590751_n4', edge='41235312#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1615590751_n5', edge='167965593#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1615590751_n6', edge='167965593#4', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1615590751_n7', edge='-437617768#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1615590751_n8', edge='148460335#0', lane=0, end_pos=-1, type='numerical')
        # TLS GS_cluster_1544209593_1615582086
        det.add_lane_area_detector(id='cluster_1544209593_b1', edge='284000162#0', lane=0, pos=94, end_pos=114, type='boolean')
        det.add_lane_area_detector(id='cluster_1544209593_b2', edge='284000162#0', lane=1, pos=94, end_pos=114, type='boolean')
        det.add_lane_area_detector(id='cluster_1544209593_b3', edge='165742420#0', lane=0, pos=141, end_pos=161, type='boolean')
        det.add_lane_area_detector(id='cluster_1544209593_b4', edge='167965593#2', lane=0, pos=216, end_pos=236, type='boolean')
        det.add_lane_area_detector(id='cluster_1544209593_b5', edge='167965593#2', lane=1, pos=216, end_pos=236, type='boolean')
        det.add_lane_area_detector(id='cluster_1544209593_b6', edge='-40848256#1', lane=0, pos=121, end_pos=141, type='boolean')
        det.add_lane_area_detector(id='cluster_1544209593_s1', edge='284000162#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1544209593_s2', edge='284000162#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1544209593_s3', edge='165742420#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1544209593_s4', edge='167965593#2', lane=0, pos=36, end_pos=56, type='saturation')
        det.add_lane_area_detector(id='cluster_1544209593_s5', edge='167965593#2', lane=1, pos=36, end_pos=56, type='saturation')
        det.add_lane_area_detector(id='cluster_1544209593_s6', edge='-40848256#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1544209593_n1', edge='284000162#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1544209593_n2', edge='284000162#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1544209593_n3', edge='165742420#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1544209593_n4', edge='167965593#2', lane=0, pos=36, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1544209593_n5', edge='167965593#2', lane=1, pos=36, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1544209593_n6', edge='-40848256#1', lane=0, end_pos=-1, type='numerical')
        # TLS joinedS_13
        det.add_lane_area_detector(id='joinedS_13_b1', edge='148457269#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_13_b2', edge='148457269#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_13_b3', edge='284000162#3', lane=0, pos=252, end_pos=262, type='boolean')
        det.add_lane_area_detector(id='joinedS_13_b4', edge='284000162#3', lane=1, pos=252, end_pos=262, type='boolean')
        det.add_lane_area_detector(id='joinedS_13_b5', edge='220892105#0', lane=0, pos=4, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_13_b6', edge='220892105#0', lane=1, pos=4, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_13_b7', edge='14038617#0', lane=0, pos=88, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_13_b8', edge='144523699#0', lane=0, pos=5, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_13_b9', edge='144523699#0', lane=1, pos=5, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_13_b10', edge='242633870', lane=0, pos=17, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_13_b11', edge='242633870', lane=1, pos=17, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_13_b12', edge='221328244#2', lane=0, pos=55, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_13_b13', edge='221328244#2', lane=1, pos=55, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_13_s1', edge='284000162#3', lane=0, pos=82, end_pos=102, type='saturation')
        det.add_lane_area_detector(id='joinedS_13_s2', edge='284000162#3', lane=1, pos=82, end_pos=102, type='saturation')
        det.add_lane_area_detector(id='joinedS_13_s3', edge='14038617#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_13_s4', edge='691714921#0', lane=0, pos=80, end_pos=100, type='saturation')
        det.add_lane_area_detector(id='joinedS_13_s5', edge='691714921#0', lane=1, pos=80, end_pos=100, type='saturation')
        det.add_lane_area_detector(id='joinedS_13_s6', edge='221328244#0', lane=0, pos=92, end_pos=112, type='saturation')
        det.add_lane_area_detector(id='joinedS_13_s7', edge='221328244#0', lane=1, pos=92, end_pos=112, type='saturation')
        det.add_lane_area_detector(id='joinedS_13_n1', edge='148457269#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n2', edge='148457269#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n3', edge='284000162#3', lane=0, pos=82, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n4', edge='284000162#3', lane=1, pos=82, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n5', edge='220892105#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n6', edge='220892105#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n7', edge='14038617#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n8', edge='144523699#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n9', edge='144523699#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n10', edge='691714921#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n11', edge='691714921#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n12', edge='691714921#0', lane=0, pos=100, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n13', edge='691714921#0', lane=1, pos=100, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n14', edge='242633870', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n15', edge='242633870', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n16', edge='221328244#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n17', edge='221328244#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n18', edge='221328244#0', lane=0, pos=100, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_13_n19', edge='221328244#0', lane=1, pos=100, end_pos=-1, type='numerical')
        # TLS GS_cluster_1773321434_198873202
        det.add_lane_area_detector(id='cluster_1773321434_b1', edge='142893788#0', lane=0, pos=47, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1773321434_b2', edge='142893788#0', lane=1, pos=47, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1773321434_b3', edge='84690884#0', lane=0, pos=72, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1773321434_b4', edge='288267108#0', lane=0, pos=139, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1773321434_b5', edge='288267108#0', lane=1, pos=139, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1773321434_s1', edge='168570431#3', lane=0, pos=38, end_pos=58, type='saturation')
        det.add_lane_area_detector(id='cluster_1773321434_s2', edge='168570431#3', lane=1, pos=38, end_pos=58, type='saturation')
        det.add_lane_area_detector(id='cluster_1773321434_s3', edge='84690884#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1773321434_s4', edge='288267108#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1773321434_s5', edge='288267108#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1773321434_n1', edge='142893788#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1773321434_n2', edge='142893788#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1773321434_n3', edge='168570431#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1773321434_n4', edge='168570431#5', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1773321434_n5', edge='168570431#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1773321434_n6', edge='168570431#4', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1773321434_n7', edge='168570431#3', lane=0, pos=38, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1773321434_n8', edge='168570431#3', lane=1, pos=38, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1773321434_n9', edge='84690884#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1773321434_n10', edge='288267108#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1773321434_n11', edge='288267108#0', lane=1, end_pos=-1, type='numerical')
        # TLS cluster1589650044_cluster1589650045_1793003425
        det.add_lane_area_detector(id='cluster1589650044_b1', edge='142893788#4', lane=0, pos=30, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1589650044_b2', edge='142893788#4', lane=1, pos=30, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1589650044_b3', edge='22662370#0', lane=0, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1589650044_b4', edge='22662370#0', lane=1, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1589650044_b5', edge='578651772#0', lane=0, pos=64, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1589650044_b6', edge='578651772#0', lane=1, pos=64, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1589650044_s1', edge='142893788#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1589650044_s2', edge='142893788#2', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1589650044_s3', edge='88899672#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1589650044_s4', edge='88899672#1', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1589650044_s5', edge='88899672#1', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1589650044_s6', edge='578651772#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1589650044_s7', edge='578651772#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1589650044_n1', edge='142893788#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1589650044_n2', edge='142893788#4', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1589650044_n3', edge='142893788#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1589650044_n4', edge='142893788#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1589650044_n5', edge='22662370#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1589650044_n6', edge='22662370#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1589650044_n7', edge='88899672#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1589650044_n8', edge='88899672#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1589650044_n9', edge='578651772#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1589650044_n10', edge='578651772#0', lane=1, end_pos=-1, type='numerical')
        # TLS GS_cluster_506774252_8568267669_8568267670_8568267671
        det.add_lane_area_detector(id='cluster_506774252_b1', edge='999679665#0', lane=0, pos=27, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_506774252_b2', edge='999679665#0', lane=1, pos=27, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_506774252_b3', edge='999679665#0', lane=2, pos=27, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_506774252_b4', edge='41432795', lane=0, pos=30, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_506774252_b5', edge='168430552#7', lane=0, pos=99, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_506774252_b6', edge='168430552#7', lane=1, pos=99, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_506774252_b7', edge='-168275984#4', lane=0, pos=74, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_506774252_s1', edge='999679668#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_506774252_s2', edge='999679668#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_506774252_s3', edge='65976786', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_506774252_s4', edge='168430552#7', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_506774252_s5', edge='168430552#7', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_506774252_s6', edge='-168275984#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_506774252_n1', edge='999679665#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774252_n2', edge='999679665#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774252_n3', edge='999679665#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774252_n4', edge='999679668#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774252_n5', edge='999679668#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774252_n6', edge='41432795', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774252_n7', edge='42533745#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774252_n8', edge='42533745#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774252_n9', edge='65976786', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774252_n10', edge='168430552#7', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774252_n11', edge='168430552#7', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774252_n12', edge='-168275984#4', lane=0, end_pos=-1, type='numerical')
        # TLS GS_cluster_506774249_5371110830
        det.add_lane_area_detector(id='cluster_506774249_b1', edge='168430553#0', lane=0, pos=99, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_506774249_b2', edge='168430553#0', lane=1, pos=99, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_506774249_b3', edge='26484578#3', lane=0, pos=48, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_506774249_b4', edge='168430552#3', lane=0, pos=55, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_506774249_b5', edge='168430552#3', lane=1, pos=55, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_506774249_b6', edge='-88899637#2', lane=0, pos=81, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_506774249_s1', edge='168430553#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_506774249_s2', edge='168430553#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_506774249_s3', edge='26484578#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_506774249_s4', edge='168430552#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_506774249_s5', edge='168430552#3', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_506774249_s6', edge='-88899637#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_506774249_n1', edge='168430553#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774249_n2', edge='168430553#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774249_n3', edge='26484578#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774249_n4', edge='26484578#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774249_n5', edge='168430552#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774249_n6', edge='168430552#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_506774249_n7', edge='-88899637#2', lane=0, end_pos=-1, type='numerical')
        # TLS GS_cluster_1589656419_1770465596
        det.add_lane_area_detector(id='cluster_1589656419_b1', edge='168430553#2', lane=0, pos=55, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1589656419_b2', edge='168430553#2', lane=1, pos=55, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1589656419_b3', edge='168430552#0', lane=0, pos=116, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1589656419_b4', edge='168430552#0', lane=1, pos=116, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1589656419_b5', edge='41432784#7', lane=0, pos=88, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1589656419_s1', edge='168430553#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1589656419_s2', edge='168430553#2', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1589656419_s3', edge='168430552#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1589656419_s4', edge='168430552#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1589656419_s5', edge='41432784#7', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1589656419_n1', edge='168430553#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1589656419_n2', edge='168430553#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1589656419_n3', edge='168430552#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1589656419_n4', edge='168430552#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1589656419_n5', edge='41432784#7', lane=0, end_pos=-1, type='numerical')
        # TLS GS_cluster_1589659457_1912530468
        det.add_lane_area_detector(id='cluster_1589659457_b1', edge='168430553#6', lane=0, pos=116, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1589659457_b2', edge='168430553#6', lane=1, pos=116, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1589659457_b3', edge='-673365068', lane=0, pos=40, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1589659457_b4', edge='306136479#9', lane=0, pos=89, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1589659457_b5', edge='306136479#9', lane=1, pos=89, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1589659457_b6', edge='833002645#0', lane=0, pos=44, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1589659457_s1', edge='168430553#6', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1589659457_s2', edge='168430553#6', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1589659457_s3', edge='-484396454#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1589659457_s4', edge='306136479#9', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1589659457_s5', edge='306136479#9', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1589659457_s6', edge='936924400#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1589659457_n1', edge='168430553#6', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1589659457_n2', edge='168430553#6', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1589659457_n3', edge='-673365068', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1589659457_n4', edge='-126504934#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1589659457_n5', edge='-484396454#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1589659457_n6', edge='306136479#9', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1589659457_n7', edge='306136479#9', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1589659457_n8', edge='833002645#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1589659457_n9', edge='936742387#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1589659457_n10', edge='936742387#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1589659457_n11', edge='936924400#0', lane=0, end_pos=-1, type='numerical')
        # TLS GS_cluster_1770465595_4365093914
        det.add_lane_area_detector(id='cluster_1770465595_b1', edge='180790923#0', lane=0, pos=89, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1770465595_b2', edge='180790923#0', lane=1, pos=89, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1770465595_b3', edge='673365071#0', lane=0, pos=48, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1770465595_b4', edge='306136479#7', lane=0, pos=36, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1770465595_b5', edge='306136479#7', lane=1, pos=36, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1770465595_s1', edge='180790923#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1770465595_s2', edge='180790923#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1770465595_s3', edge='34096527#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1770465595_s4', edge='306136479#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1770465595_s5', edge='306136479#2', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1770465595_n1', edge='180790923#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1770465595_n2', edge='180790923#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1770465595_n3', edge='673365071#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1770465595_n4', edge='673365075#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1770465595_n5', edge='34096527#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1770465595_n6', edge='306136479#7', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1770465595_n7', edge='306136479#7', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1770465595_n8', edge='306136479#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1770465595_n9', edge='306136479#5', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1770465595_n10', edge='306136479#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1770465595_n11', edge='306136479#2', lane=1, end_pos=-1, type='numerical')
        # TLS GS_cluster_149374929_1770465614_305929185
        det.add_lane_area_detector(id='cluster_149374929_b1', edge='992570224#0', lane=0, pos=23, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_149374929_b2', edge='992570224#0', lane=1, pos=23, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_149374929_b3', edge='992570224#0', lane=2, pos=23, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_149374929_b4', edge='E0', lane=0, pos=106, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_149374929_b5', edge='E0', lane=1, pos=106, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_149374929_b6', edge='180787910#7', lane=0, pos=109, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_149374929_b7', edge='180787910#7', lane=1, pos=109, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_149374929_s1', edge='180790923#6', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_149374929_s2', edge='180790923#6', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_149374929_s3', edge='E0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_149374929_s4', edge='E0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_149374929_s5', edge='180787910#4', lane=0, pos=80, end_pos=100, type='saturation')
        det.add_lane_area_detector(id='cluster_149374929_s6', edge='180787910#4', lane=1, pos=80, end_pos=100, type='saturation')
        det.add_lane_area_detector(id='cluster_149374929_n1', edge='992570224#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_149374929_n2', edge='992570224#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_149374929_n3', edge='992570224#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_149374929_n4', edge='180790923#8', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_149374929_n5', edge='180790923#8', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_149374929_n6', edge='180790923#6', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_149374929_n7', edge='180790923#6', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_149374929_n8', edge='E0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_149374929_n9', edge='E0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_149374929_n10', edge='180787910#7', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_149374929_n11', edge='180787910#7', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_149374929_n12', edge='180787910#4', lane=0, pos=80, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_149374929_n13', edge='180787910#4', lane=1, pos=80, end_pos=-1, type='numerical')
        # TLS GS_cluster_1845506036_305930099
        det.add_lane_area_detector(id='cluster_1845506036_b1', edge='1216793745#0', lane=0, pos=111, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1845506036_b2', edge='1216793745#0', lane=1, pos=111, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1845506036_b3', edge='208876350#0', lane=0, pos=181, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1845506036_b4', edge='208876350#0', lane=1, pos=181, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1845506036_b5', edge='180790926#1', lane=0, pos=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1845506036_b6', edge='180790926#1', lane=1, pos=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1845506036_b7', edge='180787905#12', lane=0, pos=57, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1845506036_b8', edge='180787905#12', lane=1, pos=57, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1845506036_s1', edge='1216793745#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1845506036_s2', edge='1216793745#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1845506036_s3', edge='208876350#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1845506036_s4', edge='208876350#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1845506036_s5', edge='165600269#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1845506036_s6', edge='165600269#3', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1845506036_s7', edge='180787905#9', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1845506036_s8', edge='180787905#9', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1845506036_n1', edge='1216793745#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1845506036_n2', edge='1216793745#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1845506036_n3', edge='208876350#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1845506036_n4', edge='208876350#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1845506036_n5', edge='180790926#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1845506036_n6', edge='180790926#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1845506036_n7', edge='180790926#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1845506036_n8', edge='180790926#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1845506036_n9', edge='165600269#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1845506036_n10', edge='165600269#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1845506036_n11', edge='180787905#12', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1845506036_n12', edge='180787905#12', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1845506036_n13', edge='180787905#9', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1845506036_n14', edge='180787905#9', lane=1, end_pos=-1, type='numerical')
        # TLS GS_cluster_507288390_508056193
        det.add_lane_area_detector(id='cluster_507288390_b1', edge='46834761#4', lane=0, pos=38, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_507288390_b2', edge='46834761#4', lane=1, pos=38, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_507288390_b3', edge='41467496#0', lane=0, pos=143, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_507288390_b4', edge='180787905#4', lane=0, pos=132, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_507288390_b5', edge='180787905#4', lane=1, pos=132, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_507288390_s1', edge='46834761#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_507288390_s2', edge='46834761#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_507288390_s3', edge='41467496#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_507288390_s4', edge='180787905#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_507288390_s5', edge='180787905#4', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_507288390_n1', edge='46834761#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_507288390_n2', edge='46834761#4', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_507288390_n3', edge='46834761#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_507288390_n4', edge='46834761#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_507288390_n5', edge='41467496#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_507288390_n6', edge='180787905#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_507288390_n7', edge='180787905#4', lane=1, end_pos=-1, type='numerical')
        # TLS cluster_485133449_485133452
        det.add_lane_area_detector(id='cluster_485133449_b1', edge='392793713', lane=0, pos=135, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_485133449_b2', edge='46834761#7', lane=0, pos=132, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_485133449_b3', edge='46834761#7', lane=1, pos=132, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_485133449_b4', edge='180787905#0', lane=0, pos=147, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_485133449_b5', edge='180787905#0', lane=1, pos=147, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_485133449_s1', edge='392793713', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_485133449_s2', edge='46834761#7', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_485133449_s3', edge='46834761#7', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_485133449_s4', edge='180787905#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_485133449_s5', edge='180787905#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_485133449_n1', edge='392793713', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_485133449_n2', edge='46834761#7', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_485133449_n3', edge='46834761#7', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_485133449_n4', edge='180787905#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_485133449_n5', edge='180787905#0', lane=1, end_pos=-1, type='numerical')
        # TLS GS_cluster_485133387_485133391
        det.add_lane_area_detector(id='cluster_485133387_b1', edge='180790921#0', lane=0, pos=146, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_485133387_b2', edge='180790921#0', lane=1, pos=146, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_485133387_b3', edge='668861332#6', lane=0, pos=59, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_485133387_b4', edge='27863026#0', lane=0, pos=149, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_485133387_b5', edge='27863026#0', lane=1, pos=149, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_485133387_s1', edge='180790921#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_485133387_s2', edge='180790921#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_485133387_s3', edge='668861332#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_485133387_s4', edge='27863026#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_485133387_s5', edge='27863026#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_485133387_n1', edge='180790921#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_485133387_n2', edge='180790921#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_485133387_n3', edge='668861332#6', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_485133387_n4', edge='668861332#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_485133387_n5', edge='27863026#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_485133387_n6', edge='27863026#0', lane=1, end_pos=-1, type='numerical')
        # TLS cluster1638086122_cluster_251058778_32828632
        det.add_lane_area_detector(id='cluster1638086122_b1', edge='40554470#1', lane=0, pos=63, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_b2', edge='288290338#0', lane=0, pos=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_b3', edge='288290338#0', lane=1, pos=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_b4', edge='288290338#0', lane=2, pos=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_b5', edge='180790922#0', lane=0, pos=149, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_b6', edge='180790922#0', lane=1, pos=149, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_b7', edge='131751139#0', lane=0, pos=140, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_b8', edge='131751139#0', lane=1, pos=140, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_b9', edge='288267110#0', lane=0, pos=142, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_b10', edge='288267110#0', lane=1, pos=142, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_s1', edge='40554470#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638086122_s2', edge='180790922#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638086122_s3', edge='180790922#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638086122_s4', edge='131751139#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638086122_s5', edge='131751139#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638086122_s6', edge='288267110#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638086122_s7', edge='288267110#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638086122_n1', edge='40554470#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n2', edge='288290338#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n3', edge='288290338#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n4', edge='288290338#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n5', edge='180790922#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n6', edge='180790922#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n7', edge='131751139#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n8', edge='131751139#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n9', edge='288267110#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n10', edge='288267110#0', lane=1, end_pos=-1, type='numerical')
        # TLS cluster1638013968_cluster_1638013992_1656149900_4052112289
        det.add_lane_area_detector(id='cluster1638086122_b1', edge='366657788#0', lane=0, pos=140, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_b2', edge='366657788#0', lane=1, pos=140, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_b3', edge='40554478#0', lane=0, pos=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_b4', edge='40554478#0', lane=1, pos=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_b5', edge='23209684#0', lane=0, pos=134, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_b6', edge='23209684#0', lane=1, pos=134, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638086122_s1', edge='366657788#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638086122_s2', edge='366657788#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638086122_s3', edge='40237005', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638086122_s4', edge='23209684#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638086122_s5', edge='23209684#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638086122_n1', edge='366657788#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n2', edge='366657788#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n3', edge='40554478#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n4', edge='40554478#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n5', edge='40237005', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n6', edge='23209684#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638086122_n7', edge='23209684#0', lane=1, end_pos=-1, type='numerical')
        # TLS GS_cluster_1635760230_1635775294_3706468062
        det.add_lane_area_detector(id='cluster_1635760230_b1', edge='671285244', lane=0, pos=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635760230_b2', edge='671285244', lane=1, pos=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635760230_b3', edge='806495095#0', lane=0, pos=15, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635760230_b4', edge='806495095#0', lane=1, pos=15, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635760230_b5', edge='23209682#0', lane=0, pos=6, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635760230_b6', edge='23209682#0', lane=1, pos=6, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635760230_b7', edge='147396980#19', lane=0, pos=12, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635760230_s1', edge='150605513#24', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635760230_s2', edge='150605513#24', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635760230_s3', edge='671285243#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635760230_s4', edge='671285243#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635760230_s5', edge='-1050785991#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635760230_s6', edge='147396980#16', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635760230_n1', edge='671285244', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n2', edge='671285244', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n3', edge='150605513#25', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n4', edge='150605513#25', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n5', edge='150605513#24', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n6', edge='150605513#24', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n7', edge='806495095#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n8', edge='806495095#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n9', edge='671285243#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n10', edge='671285243#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n11', edge='23209682#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n12', edge='23209682#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n13', edge='-1050786416#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n14', edge='-1050785991#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n15', edge='147396980#19', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635760230_n16', edge='147396980#16', lane=0, end_pos=-1, type='numerical')
        # TLS GS_cluster_1635701799_1635701803
        det.add_lane_area_detector(id='cluster_1635701799_b1', edge='150605513#19', lane=0, pos=42, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635701799_b2', edge='150605513#19', lane=1, pos=42, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635701799_b3', edge='-150682737#1', lane=0, pos=83, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635701799_b4', edge='802835940#4', lane=0, pos=42, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635701799_b5', edge='802835940#4', lane=1, pos=42, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635701799_b6', edge='-150682438#1', lane=0, pos=89, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635701799_s1', edge='150605513#12', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635701799_s2', edge='150605513#12', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635701799_s3', edge='-766710290#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635701799_s4', edge='802835940#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635701799_s5', edge='802835940#1', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635701799_s6', edge='-150682438#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635701799_n1', edge='150605513#19', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n2', edge='150605513#19', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n3', edge='150605513#17', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n4', edge='150605513#17', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n5', edge='150605513#15', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n6', edge='150605513#15', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n7', edge='150605513#12', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n8', edge='150605513#12', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n9', edge='-150682737#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n10', edge='-707591052', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n11', edge='-766710290#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n12', edge='802835940#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n13', edge='802835940#4', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n14', edge='802835940#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n15', edge='802835940#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n16', edge='802835940#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n17', edge='802835940#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n18', edge='-150682438#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635701799_n19', edge='-150682438#4', lane=0, end_pos=-1, type='numerical')
        # TLS GS_cluster_1635660842_1635660844
        det.add_lane_area_detector(id='cluster_1635660842_b1', edge='150605513#5', lane=0, pos=80, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635660842_b2', edge='150605513#5', lane=1, pos=80, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635660842_b3', edge='-150677328#4', lane=0, pos=87, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635660842_b4', edge='802835940#15', lane=0, pos=22, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635660842_b5', edge='802835940#15', lane=1, pos=22, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1635660842_s1', edge='150605513#2', lane=0, pos=70, end_pos=90, type='saturation')
        det.add_lane_area_detector(id='cluster_1635660842_s2', edge='150605513#2', lane=1, pos=70, end_pos=90, type='saturation')
        det.add_lane_area_detector(id='cluster_1635660842_s3', edge='-53755635#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635660842_s4', edge='802835940#9', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635660842_s5', edge='802835940#9', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1635660842_n1', edge='150605513#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n2', edge='150605513#5', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n3', edge='150605513#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n4', edge='150605513#4', lane=1,end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n5', edge='150605513#2', lane=0, pos=70, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n6', edge='150605513#2', lane=1, pos=70, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n7', edge='-150677328#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n8', edge='-150677328#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n9', edge='-53755635#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n10', edge='802835940#15', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n11', edge='802835940#15', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n12', edge='802835940#13', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n13', edge='802835940#13', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n14', edge='802835940#11', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n15', edge='802835940#11', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n16', edge='802835940#9', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1635660842_n17', edge='802835940#9', lane=1, end_pos=-1, type='numerical')
        # TLS 6267747172
        det.add_lane_area_detector(id='6267747172_b1', edge='602002722', lane=0, pos=129, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6267747172_b2', edge='1001742269#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6267747172_b3', edge='672957488#16', lane=0, pos=129, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6267747172_b4', edge='672957488#16', lane=1, pos=129, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6267747172_b5', edge='155602427#16', lane=0, pos=23, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6267747172_b6', edge='802835940#26', lane=0, pos=99, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6267747172_b7', edge='802835940#26', lane=1, pos=99, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6267747172_b8', edge='691939529#1', lane=0, pos=30, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6267747172_b9', edge='30248522#0', lane=0, pos=39, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6267747172_b10', edge='276423926', lane=0, pos=208, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6267747172_b11', edge='276423926', lane=1, pos=208, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='6267747172_s1', edge='967228276', lane=0, pos=38, end_pos=58, type='saturation')
        det.add_lane_area_detector(id='6267747172_s2', edge='672957488#15', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6267747172_s3', edge='672957488#15', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6267747172_s4', edge='155602427#8', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6267747172_s5', edge='802835940#22', lane=0, pos=60, end_pos=80, type='saturation')
        det.add_lane_area_detector(id='6267747172_s6', edge='802835940#22', lane=1, pos=60, end_pos=80, type='saturation')
        det.add_lane_area_detector(id='6267747172_s7', edge='288290335#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='6267747172_s8', edge='-84399501#5', lane=0, pos=140, end_pos=160, type='saturation')
        det.add_lane_area_detector(id='6267747172_s9', edge='276423926', lane=0, pos=28, end_pos=48, type='saturation')
        det.add_lane_area_detector(id='6267747172_s10', edge='276423926', lane=1, pos=28, end_pos=48, type='saturation')
        det.add_lane_area_detector(id='6267747172_n1', edge='602002722', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n2', edge='967228276', lane=0, pos=38, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n3', edge='1001742269#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n4', edge='672957488#16', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n5', edge='672957488#16', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n6', edge='672957488#15', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n7', edge='672957488#15', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n8', edge='155602427#16', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n9', edge='155602427#11', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n10', edge='155602427#8', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n11', edge='802835940#26', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n12', edge='802835940#26', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n13', edge='802835940#22', lane=0, pos=60, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n14', edge='802835940#22', lane=1, pos=60, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n15', edge='691939529#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n16', edge='288290335#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n17', edge='30248522#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n18', edge='-84399501#5', lane=0, pos=140, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n19', edge='276423926', lane=0, pos=28, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6267747172_n20', edge='276423926', lane=1, pos=28, end_pos=-1, type='numerical')
        # TLS GS_cluster_1650597986_1650597989_1652594655
        det.add_lane_area_detector(id='cluster_1650597986_b1', edge='672957488#0', lane=0, pos=59, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1650597986_b2', edge='672957488#0', lane=1, pos=59, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1650597986_b3', edge='26980716#2', lane=0, pos=50, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1650597986_b4', edge='767793616#0', lane=0, pos=63, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1650597986_b5', edge='767793616#0', lane=1, pos=63, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1650597986_s1', edge='672957488#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1650597986_s2', edge='672957488#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1650597986_s3', edge='26980716#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1650597986_s4', edge='145709108#7', lane=0, pos=74, end_pos=94, type='saturation')
        det.add_lane_area_detector(id='cluster_1650597986_s5', edge='145709108#7', lane=1, pos=74, end_pos=94, type='saturation')
        det.add_lane_area_detector(id='cluster_1650597986_n1', edge='672957488#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1650597986_n2', edge='672957488#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1650597986_n3', edge='26980716#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1650597986_n4', edge='26980716#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1650597986_n5', edge='767793616#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1650597986_n6', edge='767793616#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1650597986_n7', edge='145709108#7', lane=0, pos=74, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1650597986_n8', edge='145709108#7', lane=1, pos=74, end_pos=-1, type='numerical')
        # TLS GS_cluster_1648564005_1648564008
        det.add_lane_area_detector(id='cluster_1648564005_b1', edge='990829576#0', lane=0, pos=74, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1648564005_b2', edge='990829576#0', lane=1, pos=74, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1648564005_b3', edge='152025223#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1648564005_b4', edge='95631681#6', lane=0, pos=40, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1648564005_b5', edge='767793616#3', lane=0, pos=59, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1648564005_b6', edge='767793616#3', lane=1, pos=59, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1648564005_b7', edge='-524492800#0', lane=0, pos=13, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1648564005_s1', edge='800965695#3', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='cluster_1648564005_s2', edge='800965695#3', lane=1, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='cluster_1648564005_s3', edge='95631681#6', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1648564005_s4', edge='767793616#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1648564005_s5', edge='767793616#3', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1648564005_s6', edge='-524492805#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1648564005_n1', edge='990829576#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1648564005_n2', edge='990829576#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1648564005_n3', edge='800965695#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1648564005_n4', edge='800965695#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1648564005_n5', edge='152025223#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1648564005_n6', edge='95631681#6', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1648564005_n7', edge='767793616#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1648564005_n8', edge='767793616#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1648564005_n9', edge='-524492800#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1648564005_n10', edge='-524492800#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1648564005_n11', edge='-524492800#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1648564005_n12', edge='-524492805#1', lane=0, end_pos=-1, type='numerical')
        # TLS cluster1647146953_1647146957
        det.add_lane_area_detector(id='cluster1647146953_b1', edge='800965695#0', lane=0, pos=35, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1647146953_b2', edge='800965695#0', lane=1, pos=35, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1647146953_b3', edge='151991095#11', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1647146953_b4', edge='151991095#9', lane=0, pos=80, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1647146953_b5', edge='374928702#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1647146953_b6', edge='374928702#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1647146953_b7', edge='87607975#4', lane=0, pos=26, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1647146953_s1', edge='288267106#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1647146953_s2', edge='288267106#4', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1647146953_s3', edge='151991095#9', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1647146953_s4', edge='672957491#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1647146953_s5', edge='672957491#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1647146953_s6', edge='87607975#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1647146953_n1', edge='800965695#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1647146953_n2', edge='800965695#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1647146953_n3', edge='288267106#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1647146953_n4', edge='288267106#4', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1647146953_n5', edge='151991095#11', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1647146953_n6', edge='151991095#9', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1647146953_n7', edge='374928702#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1647146953_n8', edge='374928702#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1647146953_n9', edge='672957491#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1647146953_n10', edge='672957491#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1647146953_n11', edge='87607975#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1647146953_n12', edge='87607975#3', lane=0, end_pos=-1, type='numerical')
        # TLS joinedS_4
        det.add_lane_area_detector(id='joinedS_4_b1', edge='144524227#1', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_4_b2', edge='144524227#1', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_4_b3', edge='-1229697628#1', lane=0, pos=7, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_4_b4', edge='800965696#3', lane=0, pos=49, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_4_b5', edge='800965696#3', lane=1, pos=49, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_4_b6', edge='23298862#0', lane=0, pos=24, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_4_s1', edge='144524227#1', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='joinedS_4_s2', edge='144524227#1', lane=1, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='joinedS_4_s3', edge='-152815123', lane=0, pos=39, end_pos=59, type='saturation')
        det.add_lane_area_detector(id='joinedS_4_s4', edge='800965696#0', lane=0, pos=40, end_pos=60, type='saturation')
        det.add_lane_area_detector(id='joinedS_4_s5', edge='800965696#0', lane=1, pos=40, end_pos=60, type='saturation')
        det.add_lane_area_detector(id='joinedS_4_s6', edge='-152814632#0', lane=0, pos=88, end_pos=108, type='saturation')
        det.add_lane_area_detector(id='joinedS_4_n1', edge='144524227#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_4_n2', edge='144524227#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_4_n3', edge='-1229697628#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_4_n4', edge='-152815123', lane=0, pos=39, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_4_n5', edge='800965696#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_4_n6', edge='800965696#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_4_n7', edge='800965696#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_4_n8', edge='800965696#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_4_n9', edge='800965696#0', lane=0, pos=40, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_4_n10', edge='800965696#0', lane=1, pos=40, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_4_n11', edge='23298862#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_4_n12', edge='-624225682', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_4_n13', edge='-152814632#0', lane=0, pos=88, end_pos=-1, type='numerical')
        # TLS J11
        det.add_lane_area_detector(id='J11_b1', edge='91863445#0', lane=0, pos=46, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J11_b2', edge='246426373#6', lane=0, pos=139, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J11_b3', edge='246426373#6', lane=1, pos=139, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J11_s1', edge='91863445#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J11_s2', edge='246426373#6', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J11_s3', edge='246426373#6', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J11_n1', edge='91863445#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J11_n2', edge='246426373#6', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J11_n3', edge='246426373#6', lane=1, end_pos=-1, type='numerical')
        # TLS GS_1713983096
        det.add_lane_area_detector(id='1713983096_b1', edge='-665850365', lane=0, pos=68, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1713983096_b2', edge='295116863#0', lane=0, pos=47, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1713983096_b3', edge='295116863#0', lane=1, pos=47, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1713983096_b4', edge='41847975#0', lane=0, pos=53, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1713983096_s1', edge='-665850365', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1713983096_s2', edge='295116863#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1713983096_s3', edge='295116863#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1713983096_s4', edge='41847975#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1713983096_n1', edge='-665850365', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1713983096_n2', edge='295116863#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1713983096_n3', edge='295116863#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1713983096_n4', edge='41847975#0', lane=0, end_pos=-1, type='numerical')
        # TLS GS_1713983087
        det.add_lane_area_detector(id='1713983087_b1', edge='-94527177#3', lane=0, pos=155, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1713983087_b2', edge='94527189#0', lane=0, pos=68, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1713983087_b3', edge='665850366#0', lane=0, pos=141, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1713983087_b4', edge='665850366#0', lane=1, pos=141, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1713983087_s1', edge='-94527177#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1713983087_s2', edge='94527189#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1713983087_s3', edge='159296353#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1713983087_n1', edge='-94527177#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1713983087_n2', edge='94527189#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1713983087_n3', edge='665850366#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1713983087_n4', edge='665850366#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1713983087_n5', edge='159296353#0', lane=0, end_pos=-1, type='numerical')
        # TLS cluster1436583672_1713998722_4401347167_4401347179_#2more
        det.add_lane_area_detector(id='cluster1436583672_b1', edge='807128062', lane=0, pos=72, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1436583672_b2', edge='807128062', lane=1, pos=72, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1436583672_b3', edge='442453197#0', lane=0, pos=52, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1436583672_b4', edge='674442362', lane=0, pos=50, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1436583672_b5', edge='674442362', lane=1, pos=50, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1436583672_b6', edge='677090958', lane=0, pos=29, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1436583672_b7', edge='677090958', lane=1, pos=29, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1436583672_b8', edge='442453198#0', lane=0, pos=36, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1436583672_b9', edge='442453194#0', lane=0, pos=36, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1436583672_b10', edge='442453194#0', lane=1, pos=36, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1436583672_s1', edge='807128061', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1436583672_s2', edge='807128061', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1436583672_s3', edge='267674249-AddedOffRampEdge', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1436583672_s4', edge='267674249-AddedOffRampEdge', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1436583672_s5', edge='267674249-AddedOffRampEdge', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1436583672_s6', edge='94527177#6', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1436583672_s7', edge='120970425#12', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1436583672_s8', edge='120970425#12', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1436583672_n1', edge='807128062', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n2', edge='807128062', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n3', edge='807128061', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n4', edge='807128061', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n5', edge='442453197#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n6', edge='674442362', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n7', edge='674442362', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n8', edge='267674249-AddedOffRampEdge', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n9', edge='267674249-AddedOffRampEdge', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n10', edge='267674249-AddedOffRampEdge', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n11', edge='677090958', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n12', edge='677090958', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n13', edge='-288290339', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n14', edge='94527177#6', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n15', edge='442453198#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n16', edge='442453194#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n17', edge='442453194#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n18', edge='562902411#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n19', edge='562902411#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n20', edge='562902411#1', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n21', edge='120970425#12', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1436583672_n22', edge='120970425#12', lane=1, end_pos=-1, type='numerical')
        # TLS GS_cluster_426634816_6316128557_6316128558_801903207_801903212
        det.add_lane_area_detector(id='cluster_426634816_b1', edge='782106423#0', lane=0, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_426634816_b2', edge='782106423#0', lane=1, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_426634816_b3', edge='782106423#0', lane=2, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_426634816_b4', edge='40876928#5', lane=0, pos=29, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_426634816_b5', edge='40876928#5', lane=1, pos=29, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_426634816_b6', edge='159297373#3', lane=0, pos=73, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_426634816_b7', edge='159297373#3', lane=1, pos=73, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_426634816_s1', edge='106836508', lane=0, pos=126, end_pos=146, type='saturation')
        det.add_lane_area_detector(id='cluster_426634816_s2', edge='40876928#0', lane=0, pos=114, end_pos=134, type='saturation')
        det.add_lane_area_detector(id='cluster_426634816_s3', edge='40876928#0', lane=1, pos=114, end_pos=134, type='saturation')
        det.add_lane_area_detector(id='cluster_426634816_s4', edge='159297373#0', lane=0, pos=42, end_pos=62, type='saturation')
        det.add_lane_area_detector(id='cluster_426634816_s5', edge='159297373#0', lane=1, pos=42, end_pos=62, type='saturation')
        det.add_lane_area_detector(id='cluster_426634816_n1', edge='782106423#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n2', edge='782106423#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n3', edge='782106423#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n4', edge='150634140#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n5', edge='150634140#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n6', edge='150634140#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n7', edge='106836508', lane=0, pos=126, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n8', edge='40876928#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n9', edge='40876928#5', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n10', edge='40876928#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n11', edge='40876928#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n12', edge='40876928#0', lane=0, pos=114, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n13', edge='40876928#0', lane=1, pos=114, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n14', edge='159297373#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n15', edge='159297373#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n16', edge='159297373#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n17', edge='159297373#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n18', edge='159297373#0', lane=0, pos=42, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_426634816_n19', edge='159297373#0', lane=1, pos=42, end_pos=-1, type='numerical')
        # TLS GS_cluster_243072211_250894314
        det.add_lane_area_detector(id='cluster_243072211_b1', edge='180913152#1', lane=0, pos=85, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_243072211_b2', edge='180913152#1', lane=1, pos=85, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_243072211_b3', edge='84045910#0', lane=0, pos=54, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_243072211_b4', edge='84045910#0', lane=1, pos=54, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_243072211_b5', edge='23197994#0', lane=0, pos=140, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_243072211_b6', edge='23197994#0', lane=1, pos=140, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_243072211_s1', edge='180913152#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_243072211_s2', edge='180913152#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_243072211_s3', edge='84045910#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_243072211_s4', edge='84045910#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_243072211_s5', edge='23197994#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_243072211_s6', edge='23197994#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_243072211_n1', edge='180913152#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_243072211_n2', edge='180913152#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_243072211_n3', edge='180913152#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_243072211_n4', edge='180913152#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_243072211_n5', edge='84045910#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_243072211_n6', edge='84045910#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_243072211_n7', edge='23197994#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_243072211_n8', edge='23197994#0', lane=1, end_pos=-1, type='numerical')
        # TLS GS_cluster_1800186885_1800186886_243072210_3404227323_494986739
        det.add_lane_area_detector(id='cluster_1800186885_b1', edge='180913156#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1800186885_b2', edge='180913156#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1800186885_b3', edge='664910824#0', lane=0, pos=218, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1800186885_b4', edge='664910824#0', lane=1, pos=218, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1800186885_b5', edge='180913154', lane=0, pos=53, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1800186885_b6', edge='180913154', lane=1, pos=53, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1800186885_b7', edge='180913213#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1800186885_b8', edge='180913213#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1800186885_b9', edge='22662377#0', lane=0, pos=202, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1800186885_b10', edge='22662377#0', lane=1, pos=202, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1800186885_s1', edge='664910824#0', lane=0, pos=18, end_pos=38, type='saturation')
        det.add_lane_area_detector(id='cluster_1800186885_s2', edge='664910824#0', lane=1, pos=18, end_pos=38, type='saturation')
        det.add_lane_area_detector(id='cluster_1800186885_s3', edge='45899274#0', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='cluster_1800186885_s4', edge='45899274#0', lane=1, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='cluster_1800186885_s5', edge='22662377#0', lane=0, pos=24, end_pos=44, type='saturation')
        det.add_lane_area_detector(id='cluster_1800186885_s6', edge='22662377#0', lane=1, pos=24, end_pos=44, type='saturation')
        det.add_lane_area_detector(id='cluster_1800186885_n1', edge='180913156#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1800186885_n2', edge='180913156#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1800186885_n3', edge='664910824#0', lane=0, pos=18, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1800186885_n4', edge='664910824#0', lane=1, pos=18, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1800186885_n5', edge='180913154', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1800186885_n6', edge='180913154', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1800186885_n7', edge='180913213#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1800186885_n8', edge='180913213#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1800186885_n9', edge='22662377#0', lane=0, pos=24, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1800186885_n10', edge='22662377#0', lane=1, pos=24, end_pos=-1, type='numerical')
        # TLS GS_cluster_1302128733_423805246
        det.add_lane_area_detector(id='cluster_1302128733_b1', edge='241418811#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1302128733_b2', edge='288267111#2', lane=0, pos=185, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1302128733_b3', edge='288267111#2', lane=1, pos=185, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1302128733_b4', edge='23198655#0', lane=0, pos=216, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1302128733_b5', edge='23198655#0', lane=1, pos=216, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1302128733_s1', edge='303247928#0', lane=0, pos=13, end_pos=33, type='saturation')
        det.add_lane_area_detector(id='cluster_1302128733_s2', edge='288267111#2', lane=0, pos=5, end_pos=25, type='saturation')
        det.add_lane_area_detector(id='cluster_1302128733_s3', edge='288267111#2', lane=1, pos=5, end_pos=25, type='saturation')
        det.add_lane_area_detector(id='cluster_1302128733_s4', edge='23198655#0', lane=0, pos=36, end_pos=56, type='saturation')
        det.add_lane_area_detector(id='cluster_1302128733_s5', edge='23198655#0', lane=1, pos=36, end_pos=56, type='saturation')
        det.add_lane_area_detector(id='cluster_1302128733_n1', edge='241418811#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1302128733_n2', edge='936141522', lane=0, pos=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1302128733_n3', edge='303247928#0', lane=0, pos=13, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1302128733_n4', edge='288267111#2', lane=0, pos=5, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1302128733_n5', edge='288267111#2', lane=1, pos=5, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1302128733_n6', edge='23198655#0', lane=0, pos=36, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1302128733_n7', edge='23198655#0', lane=1, pos=36, end_pos=-1, type='numerical')
        # TLS GS_cluster_267375483_3077077589
        det.add_lane_area_detector(id='cluster_267375483_b1', edge='756213877#0', lane=0, pos=51, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_267375483_b2', edge='84818889', lane=0, pos=156, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_267375483_b3', edge='84818889', lane=1, pos=156, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_267375483_b4', edge='664910825#0', lane=0, pos=213, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_267375483_b5', edge='664910825#0', lane=1, pos=213, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_267375483_s1', edge='756213877#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_267375483_s2', edge='755442586#0', lane=0, pos=33, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='cluster_267375483_s3', edge='755442586#0', lane=1, pos=33, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='cluster_267375483_s4', edge='664910825#0', lane=0, pos=33, end_pos=53, type='saturation')
        det.add_lane_area_detector(id='cluster_267375483_s5', edge='664910825#0', lane=1, pos=33, end_pos=53, type='saturation')
        det.add_lane_area_detector(id='cluster_267375483_n1', edge='756213877#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_267375483_n2', edge='84818889', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_267375483_n3', edge='84818889', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_267375483_n4', edge='755442586#0', lane=0, pos=33, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_267375483_n5', edge='755442586#0', lane=1, pos=33, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_267375483_n6', edge='664910825#0', lane=0, pos=33, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_267375483_n7', edge='664910825#0', lane=1, pos=33, end_pos=-1, type='numerical')
        # TLS cluster_250889004_250896553_3076475495_3076475499
        det.add_lane_area_detector(id='cluster_250889004_b1', edge='684604123#5', lane=0, pos=45, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_250889004_b2', edge='221337528', lane=0, pos=51, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_250889004_b3', edge='221337528', lane=1, pos=51, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_250889004_s1', edge='684604123#5', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_250889004_s2', edge='221337528', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_250889004_s3', edge='221337528', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_250889004_n1', edge='684604123#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_250889004_n2', edge='221337528', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_250889004_n3', edge='221337528', lane=1, end_pos=-1, type='numerical')
        # TLS cluster1299481290_252000834_4063893143
        det.add_lane_area_detector(id='cluster_250889004_b1', edge='672941449', lane=0, pos=20, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_250889004_b2', edge='672941449', lane=1, pos=20, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_250889004_b3', edge='276335209#5', lane=0, pos=94, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_250889004_b4', edge='276335209#5', lane=1, pos=94, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_250889004_s1', edge='672941449', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_250889004_s2', edge='672941449', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_250889004_s3', edge='276335209#2', lane=0, pos=155, end_pos=175, type='saturation')
        det.add_lane_area_detector(id='cluster_250889004_s4', edge='276335209#2', lane=1, pos=155, end_pos=175, type='saturation')
        det.add_lane_area_detector(id='cluster_250889004_n1', edge='672941449', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_250889004_n2', edge='672941449', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_250889004_n3', edge='276335209#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_250889004_n4', edge='276335209#5', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_250889004_n5', edge='276335209#2', lane=0, pos=155, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_250889004_n6', edge='276335209#2', lane=1, pos=155, end_pos=-1, type='numerical')
        # TLS J02
        det.add_lane_area_detector(id='J02_b1', edge='934860520#0', lane=0, pos=52, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_b2', edge='934860520#0', lane=1, pos=52, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_b3', edge='934860520#0', lane=2, pos=52, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_b4', edge='897625760#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_b5', edge='897625760#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_s1', edge='1149903351', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J02_s2', edge='1149903351', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J02_s3', edge='1149903351', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J02_s4', edge='1149903351', lane=3, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J02_s5', edge='897625760#0', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='J02_s6', edge='897625760#0', lane=1, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='J02_n1', edge='934860520#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n2', edge='934860520#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n3', edge='934860520#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n4', edge='1149903351', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n5', edge='1149903351', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n6', edge='1149903351', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n7', edge='1149903351', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n8', edge='897625760#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n9', edge='897625760#0', lane=1, end_pos=-1, type='numerical')
        # TLS J0333
        det.add_lane_area_detector(id='J0333_b1', edge='897625771', lane=0, pos=14, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J0333_b2', edge='897625771', lane=1, pos=14, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J0333_b3', edge='897625775', lane=1, pos=54, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J0333_b4', edge='897625775', lane=2, pos=54, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J0333_b5', edge='897625775', lane=3, pos=54, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J0333_b6', edge='897625775', lane=4, pos=54, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J0333_s1', edge='897625771', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J0333_s2', edge='897625771', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J0333_s3', edge='897625775', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J0333_s4', edge='897625775', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J0333_s5', edge='897625775', lane=3, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J0333_s6', edge='897625775', lane=4, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J0333_n1', edge='897625771', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J0333_n2', edge='897625771', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J0333_n3', edge='897625775', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J0333_n4', edge='897625775', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J0333_n5', edge='897625775', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J0333_n6', edge='897625775', lane=4, end_pos=-1, type='numerical')
        # TLS GS_3075917655
        det.add_lane_area_detector(id='3075917655_b1', edge='671150573#2', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='3075917655_b2', edge='671150573#2', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='3075917655_b3', edge='1149903352#0', lane=1, pos=78, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='3075917655_b4', edge='1149903352#0', lane=2, pos=78, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='3075917655_s1', edge='671150573#2', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='3075917655_s2', edge='671150573#2', lane=1, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='3075917655_s3', edge='69682368', lane=0, pos=38, end_pos=58, type='saturation')
        det.add_lane_area_detector(id='3075917655_s4', edge='69682368', lane=1, pos=38, end_pos=58, type='saturation')
        det.add_lane_area_detector(id='3075917655_n1', edge='671150573#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='3075917655_n2', edge='671150573#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='3075917655_n3', edge='1149903352#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='3075917655_n4', edge='1149903352#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='3075917655_n5', edge='69682368', lane=0, pos=38, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='3075917655_n6', edge='69682368', lane=1, pos=38, end_pos=-1, type='numerical')
        # TLS joinedS_12
        det.add_lane_area_detector(id='joinedS_12_b1', edge='276335210#0', lane=0, pos=118, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b2', edge='276335210#0', lane=1, pos=118, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b3', edge='276335210#0', lane=2, pos=118, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b4', edge='91675900', lane=0, pos=5, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b5', edge='91675900', lane=1, pos=5, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b6', edge='91675900', lane=2, pos=5, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b7', edge='694271872#4', lane=0, pos=45, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b8', edge='694271872#4', lane=1, pos=45, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b9', edge='22662398', lane=0, pos=14, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b10', edge='204817782#0', lane=0, pos=124, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b11', edge='204817782#0', lane=1, pos=124, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b12', edge='204817782#0', lane=2, pos=124, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b13', edge='671150562#0', lane=0, pos=15, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b14', edge='126505997', lane=0, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b15', edge='126505997', lane=1, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b16', edge='126505997', lane=2, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_s1', edge='177433778-AddedOffRampEdge', lane=0, pos=19, end_pos=39, type='saturation')
        det.add_lane_area_detector(id='joinedS_12_s2', edge='177433778-AddedOffRampEdge', lane=1, pos=19, end_pos=39, type='saturation')
        det.add_lane_area_detector(id='joinedS_12_s3', edge='177433778-AddedOffRampEdge', lane=2, pos=19, end_pos=39, type='saturation')
        det.add_lane_area_detector(id='joinedS_12_s4', edge='694271872#0', lane=0, pos=78, end_pos=98, type='saturation')
        det.add_lane_area_detector(id='joinedS_12_s5', edge='694271872#0', lane=1, pos=78, end_pos=98, type='saturation')
        det.add_lane_area_detector(id='joinedS_12_s6', edge='177480212', lane=0, pos=75, end_pos=95, type='saturation')
        det.add_lane_area_detector(id='joinedS_12_s7', edge='177480212', lane=1, pos=75, end_pos=95, type='saturation')
        det.add_lane_area_detector(id='joinedS_12_s8', edge='177480212', lane=2, pos=75, end_pos=95, type='saturation')
        det.add_lane_area_detector(id='joinedS_12_s9', edge='681552697', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_12_s10', edge='681552697', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_12_n1', edge='276335210#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n2', edge='276335210#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n3', edge='276335210#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n4', edge='177433778-AddedOffRampEdge', lane=0, pos=19, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n5', edge='177433778-AddedOffRampEdge', lane=1, pos=19, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n6', edge='177433778-AddedOffRampEdge', lane=2, pos=19, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n7', edge='91675900', lane=0, pos=5, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n8', edge='91675900', lane=1, pos=5, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n9', edge='91675900', lane=2, pos=5, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n10', edge='694271872#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n11', edge='694271872#4', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n12', edge='694271872#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n13', edge='694271872#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n14', edge='694271872#3', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n15', edge='694271872#0', lane=0, pos=78, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n16', edge='694271872#0', lane=1, pos=78, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n17', edge='22662398', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n18', edge='204817782#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n19', edge='204817782#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n20', edge='204817782#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n21', edge='177480212', lane=0, pos=75, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n22', edge='177480212', lane=1, pos=75, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n23', edge='177480212', lane=2, pos=75, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n24', edge='671150562#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n25', edge='681552697', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n26', edge='681552697', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n27', edge='126505997', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n28', edge='126505997', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n29', edge='126505997', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n30', edge='32258211', lane=0, end_pos=-1, type='numerical')
        # TLS 652409498
        det.add_lane_area_detector(id='652409498_b1', edge='84465448', lane=0, pos=31, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='652409498_b2', edge='84465448', lane=1, pos=31, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='652409498_b3', edge='177625820', lane=0, pos=12, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='652409498_b4', edge='177625820', lane=1, pos=12, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='652409498_s1', edge='84465448', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='652409498_s2', edge='84465448', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='652409498_s3', edge='177625820', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='652409498_s4', edge='177625820', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='652409498_n1', edge='84465448', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='652409498_n2', edge='84465448', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='652409498_n3', edge='177625820', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='652409498_n4', edge='177625820', lane=1, end_pos=-1, type='numerical')
        # TLS 269243965
        det.add_lane_area_detector(id='269243965_b1', edge='177624329', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='269243965_b2', edge='177624329', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='269243965_b3', edge='992420028', lane=0, pos=57, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='269243965_b4', edge='992420028', lane=1, pos=57, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='269243965_s1', edge='992420029', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='269243965_s2', edge='992420029', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='269243965_s3', edge='992420028', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='269243965_s4', edge='992420028', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='269243965_n1', edge='177624329', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='269243965_n2', edge='177624329', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='269243965_n3', edge='992420029', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='269243965_n4', edge='992420029', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='269243965_n5', edge='992420028', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='269243965_n6', edge='992420028', lane=1, end_pos=-1, type='numerical')
        # TLS joinedS_2
        det.add_lane_area_detector(id='joinedS_2_b1', edge='220781190#2', lane=0, pos=51, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_2_b2', edge='51137831', lane=0, pos=99, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_2_b3', edge='51137831', lane=1, pos=99, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_2_b4', edge='-1125413928', lane=0, pos=84, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_2_b5', edge='84454162#5', lane=0, pos=341, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_2_b6', edge='84454162#5', lane=1, pos=341, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_2_b7', edge='115202969#9', lane=0, pos=98, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_2_s1', edge='220781190#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_2_s2', edge='1021069944', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_2_s3', edge='1021069944', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_2_s4', edge='-1125413928', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_2_s5', edge='84454162#5', lane=0, pos=161, end_pos=181, type='saturation')
        det.add_lane_area_detector(id='joinedS_2_s6', edge='84454162#5', lane=1, pos=161, end_pos=181, type='saturation')
        det.add_lane_area_detector(id='joinedS_2_s7', edge='115202969#7', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_2_n1', edge='220781190#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_2_n2', edge='220781190#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_2_n3', edge='51137831', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_2_n4', edge='51137831', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_2_n5', edge='1021069944', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_2_n6', edge='1021069944', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_2_n7', edge='-1125413928', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_2_n8', edge='84454162#5', lane=0, pos=161, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_2_n9', edge='84454162#5', lane=1, pos=161, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_2_n10', edge='115202969#9', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_2_n11', edge='115202969#7', lane=0, end_pos=-1, type='numerical')
        # TLS cluster1302567586_149374901_305918149
        det.add_lane_area_detector(id='cluster1302567586_b1', edge='880949218#0', lane=0, pos=11, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1302567586_b2', edge='665619697#1', lane=0, pos=337, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1302567586_b3', edge='665619697#1', lane=1, pos=337, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1302567586_b4', edge='84454162#0', lane=0, pos=213, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1302567586_b5', edge='84454162#0', lane=1, pos=213, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1302567586_b6', edge='115202969#0', lane=0, pos=69, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1302567586_s1', edge='24204753#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1302567586_s2', edge='665619697#1', lane=0, pos=157, end_pos=177, type='saturation')
        det.add_lane_area_detector(id='cluster1302567586_s3', edge='665619697#1', lane=1, pos=157, end_pos=177, type='saturation')
        det.add_lane_area_detector(id='cluster1302567586_s4', edge='84454162#0', lane=0, pos=33, end_pos=53, type='saturation')
        det.add_lane_area_detector(id='cluster1302567586_s5', edge='84454162#0', lane=1, pos=33, end_pos=53, type='saturation')
        det.add_lane_area_detector(id='cluster1302567586_s6', edge='931984874#0', lane=0, pos=43, end_pos=63, type='saturation')
        det.add_lane_area_detector(id='cluster1302567586_n1', edge='880949218#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567586_n2', edge='24204753#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567586_n3', edge='665619697#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567586_n4', edge='665619697#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567586_n5', edge='84454162#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567586_n6', edge='84454162#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567586_n7', edge='115202969#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567586_n8', edge='931984874#0', lane=0, pos=43, end_pos=-1, type='numerical')
        # TLS cluster1302567574_149374913_305918532
        det.add_lane_area_detector(id='cluster1302567574_b1', edge='673657150#0', lane=0, pos=8, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1302567574_b2', edge='673365083#0', lane=0, pos=213, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1302567574_b3', edge='673365083#0', lane=1, pos=213, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1302567574_b4', edge='115202976#0', lane=0, pos=109, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1302567574_b5', edge='115202976#0', lane=1, pos=109, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1302567574_b6', edge='115202962#0', lane=0, pos=115, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1302567574_s1', edge='24204747#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1302567574_s2', edge='673365083#0', lane=0, pos=33, end_pos=53, type='saturation')
        det.add_lane_area_detector(id='cluster1302567574_s3', edge='673365083#0', lane=1, pos=33, end_pos=53, type='saturation')
        det.add_lane_area_detector(id='cluster1302567574_s4', edge='42507289#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1302567574_s5', edge='42507289#4', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1302567574_s6', edge='115202962#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1302567574_n1', edge='673657150#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567574_n2', edge='24204747#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567574_n3', edge='673365083#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567574_n4', edge='673365083#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567574_n5', edge='115202976#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567574_n6', edge='115202976#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567574_n7', edge='115202962#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567574_n8', edge='42507289#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1302567574_n9', edge='42507289#4', lane=1, end_pos=-1, type='numerical')
        # TLS GS_cluster_2298692584_7170315064
        det.add_lane_area_detector(id='cluster_2298692584_b1', edge='24204751#0', lane=0, pos=46, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_2298692584_b2', edge='86444419#0', lane=0, pos=136, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_2298692584_b3', edge='86444419#0', lane=1, pos=136, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_2298692584_b4', edge='205667037', lane=0, pos=75, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_2298692584_b5', edge='205667037', lane=1, pos=75, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_2298692584_s1', edge='24204751#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_2298692584_s2', edge='86444419#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_2298692584_s3', edge='86444419#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_2298692584_s4', edge='86444401#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_2298692584_s5', edge='86444401#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_2298692584_n1', edge='24204751#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_2298692584_n2', edge='86444419#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_2298692584_n3', edge='86444419#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_2298692584_n4', edge='205667037', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_2298692584_n5', edge='205667037', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_2298692584_n6', edge='86444401#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_2298692584_n7', edge='86444401#0', lane=1, end_pos=-1, type='numerical')
        # TLS joinedS_5
        det.add_lane_area_detector(id='joinedS_5_b1', edge='40666176#2', lane=0, pos=71, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b2', edge='40666176#2', lane=1, pos=71, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b3', edge='179264937#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b4', edge='179264937#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b5', edge='1133747103#0', lane=0, pos=5, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b6', edge='1081841963#0', lane=0, pos=35, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b7', edge='1081841963#0', lane=1, pos=35, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b8', edge='40527483#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b9', edge='40527483#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b10', edge='40527483#0', lane=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b11', edge='40527483#0', lane=3, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b12', edge='927571006#0', lane=0, pos=37, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b13', edge='927571006#0', lane=1, pos=37, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b14', edge='927571006#0', lane=2, pos=37, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b15', edge='97564142#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b16', edge='97564142#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b17', edge='787899593#0', lane=0, pos=21, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b18', edge='787899593#0', lane=1, pos=21, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b19', edge='306136472#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b20', edge='306136472#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_s1', edge='40666176#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5_s2', edge='40666176#2', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5_s3', edge='-147396980#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5_s4', edge='40401871#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_5_s5', edge='39725932#1', lane=0, pos=34, end_pos=54, type='saturation')
        det.add_lane_area_detector(id='joinedS_5_s6', edge='39725932#1', lane=1, pos=34, end_pos=54, type='saturation')
        det.add_lane_area_detector(id='joinedS_5_s7', edge='306136474#0', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='joinedS_5_s8', edge='306136474#0', lane=1, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='joinedS_5_s9', edge='306136474#0', lane=2, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='joinedS_5_s10', edge='306136474#0', lane=3, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='joinedS_5_n1', edge='40666176#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n2', edge='40666176#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n3', edge='179264937#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n4', edge='179264937#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n5', edge='1133747103#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n6', edge='-147396980#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n7', edge='-147396980#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n8', edge='1081841963#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n9', edge='1081841963#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n10', edge='40401871#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n11', edge='40527483#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n12', edge='40527483#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n13', edge='40527483#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n14', edge='40527483#0', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n15', edge='927571006#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n16', edge='927571006#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n17', edge='927571006#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n18', edge='39725932#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n19', edge='39725932#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n20', edge='39725932#1', lane=0, pos=34, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n21', edge='39725932#1', lane=1, pos=34, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n22', edge='97564142#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n23', edge='97564142#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n24', edge='787899593#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n25', edge='787899593#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n26', edge='306136474#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n27', edge='306136474#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n28', edge='306136474#2', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n29', edge='306136474#2', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n30', edge='306136474#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n31', edge='306136474#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n32', edge='306136474#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n33', edge='306136474#0', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n34', edge='306136472#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n35', edge='306136472#0', lane=1, end_pos=-1, type='numerical')
        # TLS 491543745
        det.add_lane_area_detector(id='491543745_b1', edge='630227327#0', lane=0, pos=73, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b2', edge='630227327#0', lane=1, pos=73, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b3', edge='630227327#0', lane=2, pos=73, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b4', edge='630227327#0', lane=3, pos=73, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b5', edge='288290333#0', lane=0, pos=172, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b6', edge='288290333#0', lane=1, pos=172, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b7', edge='288290333#0', lane=2, pos=172, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b8', edge='288290333#0', lane=3, pos=172, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b9', edge='1022060571', lane=0, pos=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b10', edge='1022060571', lane=1, pos=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b11', edge='1022060571', lane=2, pos=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b12', edge='1022060571', lane=3, pos=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b13', edge='40527454#3', lane=0, pos=26, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b14', edge='238600644#0', lane=0, pos=20, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b15', edge='238600644#0', lane=1, pos=20, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b16', edge='40527491#0', lane=0, pos=30, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b17', edge='40527491#0', lane=1, pos=30, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b18', edge='149407640#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b19', edge='149407640#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b20', edge='671269689#0', lane=0, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b21', edge='671269689#0', lane=1, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b22', edge='671269689#0', lane=2, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b23', edge='671269689#0', lane=3, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b24', edge='40527487#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_b25', edge='40527487#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='491543745_s1', edge='136277111', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='491543745_s2', edge='136277111', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='491543745_s3', edge='136277111', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='491543745_s4', edge='288290333#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='491543745_s5', edge='288290333#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='491543745_s6', edge='288290333#0', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='491543745_s7', edge='288290333#0', lane=3, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='491543745_s8', edge='40527454#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='491543745_s9', edge='986419403#3', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='491543745_s10', edge='986419403#3', lane=1, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='491543745_n1', edge='630227327#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n2', edge='630227327#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n3', edge='630227327#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n4', edge='630227327#0', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n5', edge='136277111', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n6', edge='136277111', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n7', edge='136277111', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n8', edge='288290333#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n9', edge='288290333#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n10', edge='288290333#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n11', edge='288290333#0', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n12', edge='1022060571', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n13', edge='1022060571', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n14', edge='1022060571', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n15', edge='1022060571', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n16', edge='40527454#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n17', edge='40527454#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n18', edge='40527454#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n19', edge='238600644#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n20', edge='238600644#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n21', edge='40527491#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n22', edge='40527491#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n23', edge='40527485', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n24', edge='40527485', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n25', edge='986419403#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n26', edge='986419403#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n27', edge='149407640#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n28', edge='149407640#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n29', edge='671269689#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n30', edge='671269689#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n31', edge='671269689#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n32', edge='671269689#0', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n33', edge='40527487#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='491543745_n34', edge='40527487#0', lane=1, end_pos=-1, type='numerical')






        det.build({'detectors': os.path.join(self.THIS_FILE_PATH,'lille/lille_detectors.add.xml')})
        return det






if __name__ == '__main__':
    lille = LilleNetwork()
    print(lille.generate_flows_intra_city(100))


