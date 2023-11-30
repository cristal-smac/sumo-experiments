import math
from abc import ABC, abstractmethod
import numpy as np

class Strategy(ABC):
    """
    Abstract class to create control strategies for all the traffic lights in a network.
    """

    def __init__(self, infrastructures, detectors):
        """
        Init of class.
        """
        self.relations = self._create_relations(infrastructures, detectors)

    @abstractmethod
    def run_all_agents(self):
        """
        Perform the choose action for all agents of the strategy.
        :return: Nothing
        """
        pass

    def _create_relations(self, infrastructures, detectors):
        """
        Create relations between the different elements of the traffic lights (nodes, edges, detectors)
        :param infrastructures: The infrastructures of the network
        :type infrastructures: InfrastructureBuilder
        :param detectors: The detectors of the network
        :type detectors: DetectorBuilder
        :return: A dict with all relations for each intersection
        :rtype: dict
        """
        relations = {}
        for key in infrastructures.nodes:
            if infrastructures.nodes[key].type == 'traffic_light':
                relations[key] = {'node_infos': infrastructures.nodes[key],
                                  'related_edges': [],
                                  'related_boolean_detectors': [],
                                  'related_numerical_detectors': []}
        for key in infrastructures.edges:
            if infrastructures.edges[key].to_node in relations:
                relations[infrastructures.edges[key].to_node]['related_edges'].append(key)
        for key in relations:
            relations[key]['related_edges'] = self._order_edges(relations[key]['related_edges'], infrastructures)
            boolean_detectors = []
            numerical_detectors = []
            for edge in relations[key]['related_edges']:
                b_det = []
                n_det = []
                for det_key in detectors.laneAreaDetectors:
                    if detectors.laneAreaDetectors[det_key].edge == edge:
                        if detectors.laneAreaDetectors[det_key].type == 'boolean':
                            b_det.append(detectors.laneAreaDetectors[det_key])
                        elif detectors.laneAreaDetectors[det_key].type == 'numerical':
                            n_det.append(detectors.laneAreaDetectors[det_key])
                boolean_detectors.append(sorted(b_det, key=lambda det: det.lane.split('_')[-1]))
                numerical_detectors.append(sorted(n_det, key=lambda det: det.lane.split('_')[-1]))
            relations[key]['related_boolean_detectors'] += boolean_detectors
            relations[key]['related_numerical_detectors'] += numerical_detectors
        return relations



    def _order_edges(self, edges, infrastructures):
        """
        Order edges related to a traffic light node to a sumo information link order.
        :param edges: The edges related to a traffic light node
        :type edges: list
        :param infrastructures: The infrastructures of the network
        :type infrastructures: InfrastructureBuilder
        :return: The ordered edges
        :rtype: list
        """
        new_edges = []
        for edge_id in edges:
            from_node = infrastructures.edges[edge_id].from_node
            from_node_coord = np.array((infrastructures.nodes[from_node].x, infrastructures.nodes[from_node].y))
            to_node = infrastructures.edges[edge_id].to_node
            to_node_coord = np.array((infrastructures.nodes[to_node].x, infrastructures.nodes[to_node].y))
            angle = math.atan2(from_node_coord[0] - to_node_coord[0], from_node_coord[1] - to_node_coord[1])
            if from_node_coord[0] >= to_node_coord[0]:
                new_edges.append((edge_id, np.degrees(angle)))
            else:
                new_edges.append((edge_id, np.degrees(angle) + 360))
        new_edges.sort(key=lambda x: x[1])
        new_edges = [edge[0] for edge in new_edges]
        return new_edges