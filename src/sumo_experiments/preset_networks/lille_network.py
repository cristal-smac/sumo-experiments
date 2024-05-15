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

    def __init__(self):
        """
        Init of class
        """
        self.NET_FILE = os.path.join(self.THIS_FILE_PATH, 'lille/lille.net.xml')
        self.ENTRIES_FILE = os.path.join(self.THIS_FILE_PATH, 'lille/liste_entrees.txt')
        self.EXITS_FILE = os.path.join(self.THIS_FILE_PATH, 'lille/liste_sorties.txt')
        self.TL_JUNCTIONS = self.get_tl_junctions()
        self.EDGES_TO_TL = self.get_edges_to_tl()
        self.EDGES_FROM_TL = self.get_edges_from_tl()
        self.GRAPH = self.net_to_graph()
        self.flows = FlowBuilder()
        self.flows.add_v_type(id='car0')

    def generate_flows_entries_exits(self):
        """
        Generate flows for the network.
        :return: The flows
        :rtype: FlowBuilder
        """
        entries = self.get_entries()
        exits = self.get_exits()
        for entry in entries:
            for exit in exits:
                if entry != exit:
                    self.flows.add_flow(id=f"{entry.get('id')}-{exit.get('id')}",
                                   end=3600,
                                   from_edge=entry.get('id'),
                                   to_edge=exit.get('id'),
                                   frequency=50,
                                   v_type='car0',
                                   distribution='binomial')
        return self.flows

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
        while c < n:
            couple = random.choices(edges, k=2)
            if nx.has_path(self.GRAPH, couple[0], couple[1]) and couple[0][0] != ':' and couple[1][0] != ':':
                self.flows.add_flow(id=f"{couple[0]}-{couple[1]}",
                                    end=3600,
                                    from_edge=couple[0],
                                    to_edge=couple[1],
                                    frequency=10,
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







if __name__ == '__main__':
    lille = LilleNetwork()
    print(lille.generate_flows_intra_city(100))


