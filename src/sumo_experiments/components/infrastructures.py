import xml.etree.ElementTree as ET
import os

class InfrastructureBuilder:
    """
    The InfrastructureBuilder class is used to create the physical infrastructures of a SUMO network. The various
    infrastructures making up the network are added one by one, before being built using the build function.

    The class is composed of two types of functions:
    - The 'add_XXX' functions add different types of infrastructures to the object.
    - The 'build_XXX' functions are used to build SUMO configuration XML files based on the infrastructures added to the
    object beforehand. The build function generates the XML file.
    """

    def __init__(self):
        """
        Init of class
        """
        self.nodes = {}
        self.edges = {}
        self.types = {}
        self.connections = []
        self.tlprograms = {}

    def build(self, filenames, no_warning=True):
        """
        Generate the XML configuration files with all infrastructures.
        :param filenames: The dictionnary with all filenames for configuration.
        :type filenames: dict
        """
        self.build_nodes(filenames['nodes'])
        self.build_edges(filenames['edges'])
        self.build_edge_types(filenames['types'])
        self.build_connections(filenames['connections'])
        self.build_traffic_light_programs(filenames['trafic_light_programs'])

        # Crée le réseau à partir des éléments construits précédemment
        cmd = f'netconvert -n {filenames["nodes"]} -e {filenames["edges"]} -x {filenames["connections"]} -i {filenames["trafic_light_programs"]} -t {filenames["types"]} -o {filenames["network"]}'
        if no_warning:
            cmd += ' --no-warnings'
        os.system(cmd)

    def add_node(self, id, x, y, type='', tl_program=''):
        """
        Add a node to the infrastructures.
        :param id: ID of node
        :type id: str
        :param x: X-coordinate of
        :type x: float
        :param y: Y-coordinate of node
        :type y: float
        :param type: Node type. Fill with 'traffic_light' if the node is a traffic light, with an empty str otherwise.
        :type type: str
        :param tl_program: ID of corresponding traffic light program if the node is of type traffic light.
        :type tl_program: str
        """
        node = Node(id, x, y, type, tl_program)
        self.nodes[id] = node

    def build_nodes(self, filename):
        """
        Build all nodes in the nodes file.
        :param filename: The file where to build nodes.
        :type filename: str
        """
        xml_nodes = ET.Element('nodes')
        for node in self.nodes:
            self.nodes[node].build(xml_nodes)
        tree = ET.ElementTree(xml_nodes)
        tree.write(filename)

    def add_edge(self, id, from_node, to_node, edge_type):
        """
        Add an edge to the infrastructures.
        :param id: ID of edge
        :type id: str
        :param from_node: Starting node of edge
        :type from_node: str
        :param to_node: Ending node of edge
        :type to_node: str
        :param edge_type: Type of edge
        :type edge_type: str
        :raises AttributeError: if from_node or to_node refers to a non-existing node
        """
        if from_node not in self.nodes:
            raise AttributeError("The from_node attribute refers to a non-existing node.")
        elif to_node not in self.nodes:
            raise AttributeError("The to_node attribute refers to a non-existing node.")
        edge = Edge(id, from_node, to_node, edge_type)
        self.edges[id] = edge

    def build_edges(self, filename):
        """
        Build all edges in the edges file.
        :param filename: The file where to build edges.
        :type filename: str
        """
        xml_edges = ET.Element('edges')
        for edge in self.edges:
            self.edges[edge].build(xml_edges)
        tree = ET.ElementTree(xml_edges)
        tree.write(filename)

    def add_edge_type(self, id, params):
        """
        Add a type of edge to the object, that can used to define an edge.
        :param id: ID of the type of edge
        :type id: str
        :param params: Parameters of the edge type.
        :type params: dict
        """
        type = EdgeType(id, params)
        self.types[id] = type

    def build_edge_types(self, filename):
        """
        Build all edge types in the types file.
        :param filename: The file where to build edge types.
        :type filename: str
        """
        xml_types = ET.Element('types')
        for type in self.types:
            self.types[type].build(xml_types)
        tree = ET.ElementTree(xml_types)
        tree.write(filename)

    def add_connection(self, from_edge, to_edge, from_lane=0, to_lane=0):
        """
        Add a connection between to edge.
        :param from_edge: ID of starting edge of connection
        :type from_edge: str
        :param to_edge: ID of ending edge of connection
        :type to_edge: str
        :param from_lane: The origin lane of the connection
        :type from_lane: int
        :param to_lane: The destination lane of the connection
        :type to_lane: int
        :raises AttributeError: if from_edge or to_edge refers to a non-existing edge
        """
        if from_edge not in self.edges:
            raise AttributeError("The from_edge attribute refers to a non-existing edge.")
        elif to_edge not in self.edges:
            raise AttributeError("The to_edge attribute refers to a non-existing edge.")
        self.connections.append(Connection(from_edge, to_edge, from_lane, to_lane))

    def build_connections(self, filename):
        """
        Build all connections in the connections file.
        :param filename: The file where to build connections.
        :type filename: str
        """
        xml_connections = ET.Element('connections')
        for connection in self.connections:
            connection.build(xml_connections)
        tree = ET.ElementTree(xml_connections)
        tree.write(filename)

    def add_traffic_light_program(self, id, phases):
        """
        Add a traffic light program to the object, that can be used to define a node of type 'traffic_light'.
        :param id: ID of traffic light program
        :type id: str
        :param phases: A list containing phases of traffic light program as dicts.
        :type phases: list
        """
        self.tlprograms[id] = TrafficLightProgram(id, phases)

    def build_traffic_light_programs(self, filename):
        """
        Build traffic light programs in the corresponding file.
        :param filename: The file where to build traffic light programs.
        :type filename: str
        """
        xml_tlprograms = ET.Element('additional')
        for tl in self.tlprograms:
            self.tlprograms[tl].build(xml_tlprograms)
        tree = ET.ElementTree(xml_tlprograms)
        tree.write(filename)


class Node:
    """
    Class representing a node of a SUMO network.
    """

    def __init__(self, id, x, y, type='', tl_program=''):
        """
        Init of class.
        :param id: ID of node
        :type id: str
        :param x: X-coordinate of
        :type x: float
        :param y: Y-coordinate of node
        :type y: float
        :param type: Node type. Fill with 'traffic_light' if the node is a traffic light, with an empty str otherwise.
        :type type: str
        :param tl_program: ID of corresponding traffic light program if the node is of type traffic light.
        :type tl_program: str
        """
        self.id = id
        self.x = x
        self.y = y
        self.type = type
        self.tl = tl_program

    def build(self, xml_nodes):
        """
        Build the node in the xml object xml_nodes.
        :param xml_nodes: XML object where to build the node.
        :type xml_nodes: xml.etree.ElementTree.Element
        """
        node = ET.SubElement(xml_nodes, 'node', {'id': self.id, 'x': str(self.x), 'y': str(self.y)})
        if self.type != '':
            node.set('type', self.type)
        if self.tl != '':
            node.set('tl', self.tl)


class Edge:
    """
    Class representing an edge of a SUMO network.
    """

    def __init__(self, id, from_node, to_node, edge_type):
        """
        Init of class.
        :param id: ID of edge
        :type id: str
        :param from_node: Starting node of edge
        :type from_node: str
        :param to_node: Ending node of edge
        :type to_node: str
        :param edge_type: Type of edge
        :type edge_type: str
        """
        self.id = id
        self.from_node = from_node
        self.to_node = to_node
        self.type = edge_type

    def build(self, xml_edges):
        """
        Build the edge in the xml object xml_edges.
        :param xml_edges: XML object where to build the edge.
        :type xml_edges: xml.etree.ElementTree.Element
        """
        ET.SubElement(xml_edges, 'edge', {'id': self.id, 'from': self.from_node, 'to': self.to_node, 'type': self.type})


class EdgeType:
    """
    Class representing a type of edge of a SUMO network.
    """

    def __init__(self, id, params):
        """
        Init of class.
        :param id: ID of the type of edge
        :type id: str
        :param params: Parameters of the edge type.
        :type params: dict
        """
        self.id = id
        self.params = params

    def build(self, xml_types):
        """
        Build the edge type in the xml object xml_types.
        :param xml_types: XML object where to build the edge types.
        :type xml_types: xml.etree.ElementTree.Element
        """
        type = ET.SubElement(xml_types, 'type', {'id': self.id})
        for value in self.params:
            type.set(value, str(self.params[value]))


class Connection:
    """
    Classe representing a connection between two edges of a SUMO network.
    """

    def __init__(self, from_edge, to_edge, from_lane, to_lane):
        """
        Init of class.
        :param from_edge: ID of starting edge of connection
        :type from_edge: str
        :param to_edge: ID of ending edge of connection
        :type to_edge: str
        :param from_lane: The origin lane of the connection
        :type from_lane: int
        :param to_lane: The destination lane of the connection
        :type to_lane: int
        """
        self.from_edge = from_edge
        self.to_edge = to_edge
        self.from_lane = from_lane
        self.to_lane = to_lane

    def build(self, xml_connections):
        """
        Build the connection in the xml object xml_connections.
        :param xml_connections: XML object where to build the connections.
        :type xml_connections: xml.etree.ElementTree.Element
        """
        ET.SubElement(xml_connections, 'connection', {'from': self.from_edge, 'to': self.to_edge, 'fromLane': self.from_lane, 'toLane': self.to_lane})


class TrafficLightProgram:
    """
    Class representing a traffic light program in a SUMO network.
    """

    def __init__(self, id, phases):
        """
        Init of class.
        :param id: ID of traffic light program
        :type id: str
        :param phases: A list containing phases of traffic light program as dicts.
        :type phases: list
        """
        self.id = id
        self.phases = phases

    def build(self, xml_additionals):
        """
        Build the traffic light program in the xml object xml_additionals.
        :param xml_additionals: XML object where to build the traffic light program.
        :type xml_additionals: xml.etree.ElementTree.Element
        """
        tl = ET.SubElement(xml_additionals, 'tlLogic',
                           {'id': self.id, 'type': 'static', 'programID': self.id, 'offset': '0'})
        for phase in self.phases:
            ET.SubElement(tl, 'phase', {'duration': str(phase['duration']), 'state': phase['state']})
