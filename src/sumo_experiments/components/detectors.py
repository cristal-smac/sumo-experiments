import xml.etree.ElementTree as ET

class DetectorBuilder:
    """
    The DetectorBuilder class can be used to create a set of detectors that will be added to a road network when
    creating an experiment. Detectors measure traffic data in real time, enabling the behavior of the network's physical
    infrastructure to be adapted to the flow of traffic.

    The class is composed of two types of functions:
    - The 'add_XXX' functions add different types of detectors to the object.
    - The 'build_XXX' functions are used to build SUMO configuration XML files based on the detectors added to the
    object beforehand. The build function generates the XML file.
    """

    def __init__(self):
        """
        Init of class.
        """
        self.laneAreaDetectors = {}

    def build(self, filenames):
        """
        Generate the XML configuration file with all detectors.
        :param filenames: The dictionnary with all filenames for configuration, including detectors file.
        :type filenames: dict
        """
        xml_additional = ET.Element('additional')
        self.build_lane_area_detectors(xml_additional)
        tree = ET.ElementTree(xml_additional)
        tree.write(filenames['detectors'])

    def add_lane_area_detector(self, id, edge, lane, type, pos=0, end_pos=-0.1, freq=100000):
        """
        Add a E2 detector to the object.
        :param id: ID of detector
        :type id: str
        :param edge: The edge where to place the detector
        :type edge: str
        :param lane: The lane where to place the detector
        :type lane: int
        :param type: The type of detector, 'boolean' or 'numerical'
        :type type: str
        :param pos: The beginning of the detection area on the lane
        :type pos: float
        :param end_pos: The end of the detection area on the lane
        :type end_pos: float
        :param freq: Detector frequency (Hertz)
        :type freq: int
        :param file: The file where the detector will be saved
        :type file: str
        """
        self.laneAreaDetectors[id] = LaneAreaDetector(id, edge, lane, type, pos, end_pos, freq, 'detectors.out')

    def build_lane_area_detectors(self, xml):
        """
        Build all lane area detectors in the XML.
        :param xml: The XML where to build lane area detectors.
        :type xml: xml.etree.ElementTree.Element
        """
        for detector in self.laneAreaDetectors:
            self.laneAreaDetectors[detector].build(xml)



class LaneAreaDetector:
    """
    Class defining an E2 detector to be integrated into a SUMO simulation
    """

    def __init__(self, id, edge, lane, type, pos, end_pos, freq, file):
        """
        Init of class.
        :param id: ID of detector
        :type id: str
        :param edge: The edge where to place the detector
        :type edge: str
        :param lane: The lane where to place the detector
        :type lane: int
        :param type: The type of detector, 'boolean' or 'numerical'
        :type type: str
        :param pos: The beginning of the detection area on the lane
        :type pos: float
        :param end_pos: The end of the detection area on the lane
        :type end_pos: float
        :param freq: Detector frequency (Hertz)
        :type freq: int
        :param file: The file where the detector will be saved
        :type file: str
        """
        self.id = str(id)
        self.edge = str(edge)
        self.lane = str(lane)
        self.type = type
        self.pos = str(pos)
        self.end_pos = str(end_pos)
        self.freq = str(freq)
        self.file = str(file)

    def build(self, xml):
        """
        Build the detector in the XML Element.
        :param xml: XML object where to build the detector
        :type xml: xml.etree.ElementTree.Element
        """
        ET.SubElement(xml,
                      'laneAreaDetector',
                      {'id': self.id,
                            'lane': f'{self.edge}_{self.lane}',
                            'pos': self.pos,
                            'endPos': self.end_pos,
                            'freq': self.freq,
                            'file': self.file}
                      )