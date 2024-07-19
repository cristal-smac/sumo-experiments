import os
import random
import xml.etree.ElementTree as ET
from sumo_experiments.components import InfrastructureBuilder, FlowBuilder, DetectorBuilder
from copy import deepcopy
import random


class BolognaNetwork:
    """
    Create the SUMO network and flows for the city of Lille.
    """

    THIS_FILE_PATH = os.path.abspath(os.path.dirname(__file__))

    def __init__(self):
        """
        Init of class
        """
        self.FULL_LINE_COMMAND = f"sumo -c {self.THIS_FILE_PATH}/bologna/acosta/run.sumocfg"
        self.NET_FILE = os.path.join(self.THIS_FILE_PATH, 'bologna/acosta/acosta_buslanes.net.xml')
        self.FLOW_FILE = os.path.join(self.THIS_FILE_PATH, 'bologna/acosta/acosta.rou.xml')
        self.NEW_FLOW_FILE = os.path.join(self.THIS_FILE_PATH, 'bologna/acosta/acosta_NEW.rou.xml')

    def generate_infrastructures(self):
        """
        Generate the infrastructures of the network from the network files
        :return: The infrastructures of the network
        :rtype: InfrastructureBuilder
        """
        infra = InfrastructureBuilder()
        tree = ET.parse(self.NET_FILE)
        nodes = [e for e in tree.iter('junction')]
        for node in nodes:
            infra.add_node(id=node.get('id'),
                           x=float(node.get('x')),
                           y=float(node.get('y')),
                           type=node.get('type'))
        edges = [e for e in tree.iter('edge')]
        for edge in edges:
            if not edge.get('id')[0] == ':':
                infra.add_edge(id=edge.get('id'),
                               from_node=edge.get('from'),
                               to_node=edge.get('to'),
                               edge_type='default')
        connections = [e for e in tree.iter('connection')]
        for conn in connections:
            if conn.get('from')[0] != ':' and conn.get('to')[0] != ':':
                infra.add_connection(from_edge=conn.get('from'),
                                     to_edge=conn.get('to'),
                                     from_lane=int(conn.get('fromLane')),
                                     to_lane=int(conn.get('toLane')))
        return infra

    def generate_flows(self, coeff):
        """
        Generate a flow file for the bologna network, based on FLOW_FILE.
        The coeff parameter is used to multiply the number of generated vehicles. Can be superior to 1.
        :param coeff: The multiplier of the number of generated vehicles.
        :type coeff: float
        :return: Nothing
        """
        if os.path.isfile(self.NEW_FLOW_FILE):
            os.remove(self.NEW_FLOW_FILE)
        with open(self.FLOW_FILE, 'r') as flow_file:
            with open(self.NEW_FLOW_FILE, 'a') as new_flow_file:
                entete = flow_file.readline()
                new_flow_file.write(entete)
                next_vehicle = flow_file.readline()
                while next_vehicle != "</routes>":
                    curr_coeff = deepcopy(coeff)
                    while curr_coeff >= 1:
                        splitted = next_vehicle.split('"')
                        splitted[11] += f'_x{int(curr_coeff * 10)}'
                        next_vehicle = '"'.join(splitted)
                        new_flow_file.write(next_vehicle)
                        curr_coeff -= 1
                    if random.random() < curr_coeff:
                        splitted = next_vehicle.split('"')
                        splitted[11] += f'_x{int(curr_coeff * 10)}'
                        next_vehicle = '"'.join(splitted)
                        new_flow_file.write(next_vehicle)
                    next_vehicle = flow_file.readline()
                new_flow_file.write(next_vehicle)



    def generate_detectors(self):
        """
        Generate the detectors of the network.
        :return: The detectors of the network
        :rtype: DetectorBuilder
        """
        det = DetectorBuilder()
        # Intersection 34
        det.add_lane_area_detector(id='34_1', edge='43[0]', lane=2, end_pos=20, type='boolean')
        det.add_lane_area_detector(id='34_2', edge='117', lane=0, pos=108, end_pos=128, type='boolean')
        det.add_lane_area_detector(id='34_3', edge='113', lane=0, pos=60, end_pos=80, type='boolean')
        det.add_lane_area_detector(id='34_4', edge='113', lane=1, pos=60, end_pos=80, type='boolean')
        det.add_lane_area_detector(id='34_5', edge='113', lane=2, pos=60, end_pos=80, type='boolean')
        det.add_lane_area_detector(id='34_6', edge='117', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='34_7', edge='113', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='34_8', edge='113', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='34_9', edge='113', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='34_10', edge='43[0]', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='34_11', edge='210', lane=2, pos=184, end_pos=364, type='numerical')
        det.add_lane_area_detector(id='34_12', edge='117', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='34_13', edge='113', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='34_14', edge='113', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='34_15', edge='113', lane=2, end_pos=-1, type='numerical')
        # Intersection 27
        det.add_lane_area_detector(id='27_1', edge='43[1]', lane=0, pos=60, end_pos=80, type='boolean')
        det.add_lane_area_detector(id='27_2', edge='43[1]', lane=1, pos=60, end_pos=80, type='boolean')
        det.add_lane_area_detector(id='27_3', edge='43[1]', lane=2, pos=60, end_pos=80, type='boolean')
        det.add_lane_area_detector(id='27_4', edge='31', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='27_5', edge='31', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='27_6', edge='31', lane=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='27_7', edge='34', lane=0, pos=38, end_pos=58, type='boolean')
        det.add_lane_area_detector(id='27_8', edge='34', lane=1, pos=38, end_pos=58, type='boolean')
        det.add_lane_area_detector(id='27_9', edge='34', lane=2, pos=38, end_pos=58, type='boolean')
        det.add_lane_area_detector(id='27_10', edge='46', lane=0, pos=46.8, end_pos=66.8, type='boolean')
        det.add_lane_area_detector(id='27_11', edge='46', lane=1, pos=46.8, end_pos=66.8, type='boolean')
        det.add_lane_area_detector(id='27_12', edge='46', lane=2, pos=46.8, end_pos=66.8, type='boolean')
        det.add_lane_area_detector(id='27_13', edge='43[1]', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='27_14', edge='43[1]', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='27_15', edge='43[1]', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='27_16', edge='202', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='27_17', edge='202', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='27_18', edge='202', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='27_19', edge='116', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='27_20', edge='43[1]', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_21', edge='43[1]', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_22', edge='43[1]', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_23', edge='31', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_24', edge='31', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_25', edge='31', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_26', edge='133', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_27', edge='165', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_28', edge='34', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_29', edge='34', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_30', edge='34', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_31', edge='202', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_32', edge='202', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_33', edge='202', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_34', edge='46', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_35', edge='46', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_36', edge='46', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='27_37', edge='116', lane=0, end_pos=-1, type='numerical')
        # Intersections 1-1b-1c-3
        det.add_lane_area_detector(id='1_1', edge='201', lane=0, pos=211.3, end_pos=231.3, type='boolean')
        det.add_lane_area_detector(id='1_2', edge='201', lane=1, pos=211.3, end_pos=231.3, type='boolean')
        det.add_lane_area_detector(id='1_3', edge='201', lane=2, pos=211.3, end_pos=231.3, type='boolean')
        det.add_lane_area_detector(id='1_4', edge='203[1]', lane=0, pos=209.7, end_pos=229.7, type='boolean')
        det.add_lane_area_detector(id='1_5', edge='203[1]', lane=1, pos=209.7, end_pos=229.7, type='boolean')
        det.add_lane_area_detector(id='1_6', edge='203[1]', lane=2, pos=209.7, end_pos=229.7, type='boolean')
        det.add_lane_area_detector(id='1_7', edge='1b', lane=0, pos=40.4, end_pos=60.4, type='boolean')
        det.add_lane_area_detector(id='1_8', edge='1b', lane=1, pos=40.4, end_pos=60.4, type='boolean')
        det.add_lane_area_detector(id='1_9', edge='2', lane=0, pos=19.6, end_pos=39.6, type='boolean')
        det.add_lane_area_detector(id='1_10', edge='2', lane=1, pos=19.6, end_pos=39.6, type='boolean')
        det.add_lane_area_detector(id='1_11', edge='2', lane=2, pos=19.6, end_pos=39.6, type='boolean')
        det.add_lane_area_detector(id='1_12', edge='201', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1_13', edge='201', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1_14', edge='201', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1_15', edge='203[1]', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1_16', edge='203[1]', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1_17', edge='203[1]', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1_18', edge='122', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1_19', edge='122', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1_20', edge='122', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1_21', edge='201', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_22', edge='201', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_23', edge='201', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_24', edge='203[1]', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_25', edge='203[1]', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_26', edge='203[1]', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_27', edge='1b', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_28', edge='1b', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_29', edge='2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_30', edge='2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_31', edge='2', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_32', edge='3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_33', edge='3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_34', edge='122', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_35', edge='122', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1_36', edge='122', lane=2, end_pos=-1, type='numerical')
        # Intersection 78-44-43-204c
        det.add_lane_area_detector(id='78_1', edge='203[0]', lane=0, pos=381.1, end_pos=401.1, type='boolean')
        det.add_lane_area_detector(id='78_2', edge='203[0]', lane=1, pos=381.1, end_pos=401.1, type='boolean')
        det.add_lane_area_detector(id='78_3', edge='203[0]', lane=2, pos=381.1, end_pos=401.1, type='boolean')
        det.add_lane_area_detector(id='78_4', edge='204a[0]', lane=0, pos=170.2, end_pos=190.2, type='boolean')
        det.add_lane_area_detector(id='78_5', edge='204a[0]', lane=1, pos=170.2, end_pos=190.2, type='boolean')
        det.add_lane_area_detector(id='78_6', edge='204a[0]', lane=2, pos=170.2, end_pos=190.2, type='boolean')
        det.add_lane_area_detector(id='78_7', edge='55b', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='78_8', edge='55b', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='78_9', edge='55b', lane=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='78_10', edge='121', lane=0, pos=673.6, end_pos=693.6, type='boolean')
        det.add_lane_area_detector(id='78_11', edge='204a[0]', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='78_12', edge='204a[0]', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='78_13', edge='204a[0]', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='78_14', edge='203[0]', lane=0, pos=200, end_pos=401.1, type='numerical')
        det.add_lane_area_detector(id='78_15', edge='203[0]', lane=1, pos=200, end_pos=401.1, type='numerical')
        det.add_lane_area_detector(id='78_16', edge='203[0]', lane=2, pos=200, end_pos=401.1, type='numerical')
        det.add_lane_area_detector(id='78_17', edge='204a[0]', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='78_18', edge='204a[0]', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='78_19', edge='204a[0]', lane=2, end_pos=-1, type='numerical')
        # Intersection 52
        det.add_lane_area_detector(id='52_1', edge='69', lane=0, pos=116, end_pos=136, type='boolean')
        det.add_lane_area_detector(id='52_2', edge='69', lane=1, pos=116, end_pos=136, type='boolean')
        det.add_lane_area_detector(id='52_3', edge='171', lane=0, pos=153.8, end_pos=173.8, type='boolean')
        det.add_lane_area_detector(id='52_4', edge='69', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='52_5', edge='69', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='52_6', edge='171', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='52_7', edge='69', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='52_8', edge='69', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='52_9', edge='171', lane=0, end_pos=-1, type='numerical')
        # Intersection 15
        det.add_lane_area_detector(id='15_1', edge='72[1]', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='15_2', edge='72[1]', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='15_3', edge='191', lane=0, pos=148, end_pos=168, type='boolean')
        det.add_lane_area_detector(id='15_4', edge='18', lane=0, pos=6.1, end_pos=26.1, type='boolean')
        det.add_lane_area_detector(id='15_5', edge='18', lane=1, pos=6.1, end_pos=26.1, type='boolean')
        det.add_lane_area_detector(id='15_6', edge='72[0]', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='15_7', edge='72[0]', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='15_8', edge='72[0]', lane=2, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='15_9', edge='74', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='15_10', edge='181', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='15_11', edge='181', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='15_12', edge='191', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='15_13', edge='72[0]', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='15_14', edge='72[0]', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='15_15', edge='72[0]', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='15_16', edge='72[1]', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='15_17', edge='72[1]', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='15_18', edge='74', lane=0, pos=67, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='15_19', edge='181', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='15_20', edge='181', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='15_21', edge='18', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='15_22', edge='18', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='15_23', edge='191', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='15_24', edge='72[0]', lane=0, pos=150, end_pos=170, type='boolean')
        det.add_lane_area_detector(id='15_25', edge='72[0]', lane=1, pos=150, end_pos=170, type='boolean')
        det.add_lane_area_detector(id='15_26', edge='72[0]', lane=2, pos=150, end_pos=170, type='boolean')
        # Intersection 9
        det.add_lane_area_detector(id='9_1', edge='12', lane=0, pos=38.6, end_pos=58.6, type='boolean')
        det.add_lane_area_detector(id='9_2', edge='12', lane=1, pos=38.6, end_pos=58.6, type='boolean')
        det.add_lane_area_detector(id='9_3', edge='12', lane=2, pos=38.6, end_pos=58.6, type='boolean')
        det.add_lane_area_detector(id='9_4', edge='71', lane=0, pos=41.2, end_pos=61.2, type='boolean')
        det.add_lane_area_detector(id='9_5', edge='71', lane=1, pos=41.2, end_pos=61.2, type='boolean')
        det.add_lane_area_detector(id='9_6', edge='71', lane=2, pos=41.2, end_pos=61.2, type='boolean')
        det.add_lane_area_detector(id='9_7', edge='11', lane=0, pos=13.9, end_pos=33.9, type='boolean')
        det.add_lane_area_detector(id='9_8', edge='11', lane=1, pos=13.9, end_pos=33.9, type='boolean')
        det.add_lane_area_detector(id='9_9', edge='11', lane=2, pos=13.9, end_pos=33.9, type='boolean')
        det.add_lane_area_detector(id='9_10', edge='85', lane=0, pos=313.1, end_pos=333.1, type='boolean')
        det.add_lane_area_detector(id='9_11', edge='85', lane=1, pos=313.1, end_pos=333.1, type='boolean')
        det.add_lane_area_detector(id='9_12', edge='85', lane=2, pos=313.1, end_pos=333.1, type='boolean')
        det.add_lane_area_detector(id='9_13', edge='83', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='9_14', edge='17', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='9_15', edge='17', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='9_16', edge='68', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='9_17', edge='68', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='9_18', edge='12', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='9_19', edge='12', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='9_20', edge='12', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='9_21', edge='83', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='9_22', edge='71', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='9_23', edge='71', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='9_24', edge='71', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='9_25', edge='17', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='9_26', edge='17', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='9_27', edge='11', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='9_28', edge='11', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='9_29', edge='11', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='9_30', edge='68', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='9_31', edge='68', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='9_32', edge='85', lane=0, pos=133.1, end_pos=333.1, type='numerical')
        det.add_lane_area_detector(id='9_33', edge='85', lane=1, pos=133.1, end_pos=333.1, type='numerical')
        det.add_lane_area_detector(id='9_34', edge='85', lane=2, pos=133.1, end_pos=333.1, type='numerical')
        # Intersection 12
        det.add_lane_area_detector(id='12_1', edge='104', lane=0, pos=17.8, end_pos=37.8, type='boolean')
        det.add_lane_area_detector(id='12_2', edge='104', lane=1, pos=17.8, end_pos=37.8, type='boolean')
        det.add_lane_area_detector(id='12_3', edge='15', lane=0, pos=71.4, end_pos=91.4, type='boolean')
        det.add_lane_area_detector(id='12_4', edge='103', lane=0, pos=117.85, end_pos=137.85, type='boolean')
        det.add_lane_area_detector(id='12_5', edge='103', lane=1, pos=117.85, end_pos=137.85, type='boolean')
        det.add_lane_area_detector(id='12_6', edge='103', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='12_7', edge='103', lane=1, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='12_8', edge='15', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='12_9', edge='13', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='12_10', edge='104', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='12_11', edge='104', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='12_12', edge='15', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='12_13', edge='103', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='12_14', edge='103', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='12_15', edge='13', lane=0, end_pos=-1, type='numerical')
        # Intersection 63
        det.add_lane_area_detector(id='63_1', edge='188', lane=0, pos=64.3, end_pos=84.3, type='boolean')
        det.add_lane_area_detector(id='63_2', edge='153', lane=0, pos=376, end_pos=396, type='boolean')
        det.add_lane_area_detector(id='63_3', edge='88', lane=0, pos=30.8, end_pos=50.8, type='boolean')
        det.add_lane_area_detector(id='63_4', edge='188', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='63_5', edge='153', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='63_6', edge='88', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='63_7', edge='188', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='63_8', edge='153', lane=196, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='63_9', edge='88', lane=0, end_pos=-1, type='numerical')
        # Intersection 82
        det.add_lane_area_detector(id='82_1', edge='189[1][1]', lane=0, pos=151.55, end_pos=171.55, type='boolean')
        det.add_lane_area_detector(id='82_2', edge='187', lane=0, pos=64.3, end_pos=84.3, type='boolean')
        det.add_lane_area_detector(id='82_3', edge='189[1][1]', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='82_4', edge='187', lane=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='82_5', edge='189[1][1]', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='82_6', edge='187', lane=0, end_pos=-1, type='saturation')
        return det

if __name__ == '__main__':
    bologna = BolognaNetwork()
    # det = bologna.generate_detectors()
    # det.build({'detectors': 'acosta_detectors.add.xml'})
    bologna.generate_flows(3)

