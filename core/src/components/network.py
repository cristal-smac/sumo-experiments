import xml.etree.ElementTree as ET
import os

class NetworkBuilder:
    """
    Classe Network
    Représente un réseau routier utilisable dans le logiciel SUMO
    """

    def __init__(self, nodes={}, edges={}, types={}, connections=[], tlprograms={}):
        """
        Constructeur de la classe
        Crée un réseau avec les éléments passés en paramètre.
        Si rien n'est passé en paramètre, crée un réseau vide.
        :param nodes: Noeuds du réseau
        :param edges: Arêtes du réseau
        :param types: Types de voies du réseau
        :param connections: Connexions entre arêtes du réseau
        :param tlprograms: Programmes de feux routiers du réseau
        """
        self.nodes = nodes
        self.edges = edges
        self.types = types
        self.connections = connections
        self.tlprograms = tlprograms

    def build(self, filenames):
        """
        Construit le réseau routier dans les fichiers passés en paramètre.
        :param filenames: Liste des fichiers où construire le réseau
        """
        self.build_nodes(filenames['nodes'])
        self.build_edges(filenames['edges'])
        self.build_types(filenames['types'])
        self.build_connections(filenames['connections'])
        self.build_traffic_light_programs(filenames['trafic_light_programs'])

        # Crée le réseau à partir des éléments construits précédemment
        os.system(
            f'netconvert -n {filenames["nodes"]} -e {filenames["edges"]} -x {filenames["connections"]} -i {filenames["trafic_light_programs"]} -t {filenames["types"]} -o {filenames["network"]}')

    def add_node(self, id, x, y, type='', tl=''):
        """
        Ajoute un noeud au réseau
        :param id: Identifiant du noeud
        :param x: Abscisse du noeud
        :param y: Ordonnée du noeud
        :param type: Type du noeud
        :param tl: Identifiant du feu routier associé au noeud
        """
        node = Node(id, x, y, type, tl)
        self.nodes[id] = node
        return self

    def build_nodes(self, filename):
        """
        Construit les noeuds du réseau dans le fichier passé en paramètre.
        :param filename: Fichier où construire les noeuds
        """
        xml_nodes = ET.Element('nodes')
        for node in self.nodes:
            self.nodes[node].build(xml_nodes)
        tree = ET.ElementTree(xml_nodes)
        tree.write(filename)

    def add_edge(self, id, from_node, to_node, type):
        """
        Ajoute une arête au réseau
        :param id: Identifiant de l'arête
        :param from_node: Noeud de départ de l'arête
        :param to_node: Noeud d'arrivée de l'arête
        :param type: Type de l'arête
        """
        if from_node not in self.nodes or to_node not in self.nodes:
            return False
        edge = Edge(id, from_node, to_node, type)
        self.edges[id] = edge
        return self

    def build_edges(self, filename):
        """
        Construit les arêtes du réseau dans le fichier passé en paramètre.
        :param filename: Fichier où construire les arêtes
        """
        xml_edges = ET.Element('edges')
        for edge in self.edges:
            self.edges[edge].build(xml_edges)
        tree = ET.ElementTree(xml_edges)
        tree.write(filename)

    def add_type(self, id, values):
        """
        Ajoute un type de voie au réseau
        :param id: Identifiant du type de voie
        :param values: Valeurs du type de voie
        """
        type = Type(id, values)
        self.types[id] = type
        return self

    def build_types(self, filename):
        """
        Construit les types de voies du réseau dans le fichier passé en paramètre.
        :param filename: Fichier où construire les types de voies
        """
        xml_types = ET.Element('types')
        for type in self.types:
            self.types[type].build(xml_types)
        tree = ET.ElementTree(xml_types)
        tree.write(filename)

    def add_connection(self, from_edge, to_edge):
        """
        Ajoute une connexion entre deux arêtes du réseau
        :param from_edge: Arête de départ de la connexion
        :param to_edge: Arête d'arrivée de la connexion
        """
        if from_edge not in self.edges or to_edge not in self.edges:
            return False
        self.connections.append(Connection(from_edge, to_edge))
        return self

    def build_connections(self, filename):
        """
        Construit les connexions entre arêtes du réseau dans le fichier passé en paramètre.
        :param filename: Fichier où construire les connexions
        """
        xml_connections = ET.Element('connections')
        for connection in self.connections:
            connection.build(xml_connections)
        tree = ET.ElementTree(xml_connections)
        print(type(tree))
        tree.write(filename)

    def add_traffic_light_program(self, id, programID, phases):
        """
        Ajoute un programme de feu routier au réseau
        :param id: Identifiant du programme de feu routier
        :param programID: Identifiant du programme de feu routier
        :param phases: Phases du programme de feu routier
        """
        self.tlprograms[id] = TrafficLightProgram(id, programID, phases)
        return self

    def build_traffic_light_programs(self, filename):
        """
        Construit les programmes de feu routier du réseau dans le fichier passé en paramètre.
        :param filename: Fichier où construire les programmes de feu routier
        """
        xml_tlprograms = ET.Element('additional')
        for tl in self.tlprograms:
            self.tlprograms[tl].build(xml_tlprograms)
        tree = ET.ElementTree(xml_tlprograms)
        tree.write(filename)


class Node:
    """
    Classe représentant un noeud du réseau routier
    """

    def __init__(self, id, x, y, type='', tl=''):
        """
        Constructeur de la classe Node
        :param id: Identifiant du noeud
        :param x: Abscisse du noeud
        :param y: Ordonnée du noeud
        :param type: Type du noeud
        :param tl: Identifiant du feu routier associé au noeud
        """
        self.id = id
        self.x = x
        self.y = y
        self.type = type
        self.tl = tl

    def build(self, nodes):
        """
        Construit le noeud dans l'objet XML ElementTree passé en paramètre
        :param nodes: Objet XML ElementTree où construire le noeud
        """
        node = ET.SubElement(nodes, 'node', {'id': self.id, 'x': str(self.x), 'y': str(self.y)})
        if self.type != '':
            node.set('type', self.type)
        if self.tl != '':
            node.set('tl', self.tl)


class Edge:
    """
    Classe représentant une arête du réseau routier
    """

    def __init__(self, id, from_node, to_node, type):
        """
        Constructeur de la classe Edge
        :param id: Identifiant de l'arête
        :param from_node: Noeud de départ de l'arête
        :param to_node: Noeud d'arrivée de l'arête
        :param type: Type de voie de l'arête
        """
        self.id = id
        self.from_node = from_node
        self.to_node = to_node
        self.type = type

    def build(self, edges):
        """
        Construit l'arête dans l'objet XML ElementTree passé en paramètre
        :param edges: Objet XML ElementTree où construire l'arête
        """
        ET.SubElement(edges, 'edge', {'id': self.id, 'from': self.from_node, 'to': self.to_node, 'type': self.type})


class Type:
    """
    Class représentant un type de voie du réseau routier
    """

    def __init__(self, id, values):
        """
        Constructeur de la classe Type
        :param id: Identifiant du type de voie
        :param values: Valeurs du type de voie
        """
        self.id = id
        self.values = values

    def build(self, types):
        """
        Construit le type de voie dans l'objet XML ElementTree passé en paramètre
        :param types: Objet XML ElementTree où construire le type de voie
        """
        type = ET.SubElement(types, 'type', {'id': self.id})
        for value in self.values:
            type.set(value, str(self.values[value]))


class Connection:
    """
    Classe représentant une connexion entre deux arêtes du réseau routier
    """

    def __init__(self, from_edge, to_edge):
        """
        Constructeur de la classe Connection
        :param from_edge: Arête de départ de la connexion
        :param to_edge: Arête d'arrivée de la connexion
        """
        self.from_edge = from_edge
        self.to_edge = to_edge

    def build(self, connections):
        """
        Construit la connexion dans l'objet XML ElementTree passé en paramètre
        :param connections: Objet XML ElementTree où construire la connexion
        """
        ET.SubElement(connections, 'connection', {'from': self.from_edge, 'to': self.to_edge})


class TrafficLightProgram:
    """
    Classe représentant un programme de feu routier
    """

    def __init__(self, id, programID, phases):
        """
        Constructeur de la classe TrafficLightProgram
        :param id: Identifiant du programme de feu routier
        :param programID: Identifiant du programme de feu routier
        :param phases: Phases du programme de feu routier
        """
        self.id = id
        self.programID = programID
        self.phases = phases

    def build(self, additionals):
        """
        Construit le programme de feu routier dans l'objet XML ElementTree passé en paramètre
        :param additionals: Objet XML ElementTree où construire le programme de feu routier
        """
        tl = ET.SubElement(additionals, 'tlLogic',
                           {'id': self.id, 'type': 'static', 'programID': self.programID, 'offset': '0'})
        for phase in self.phases:
            ET.SubElement(tl, 'phase', {'duration': str(phase['duration']), 'state': phase['state']})
