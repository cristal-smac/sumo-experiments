import xml.etree.ElementTree as ET

class FlowBuilder:
    """
    The FlowBuilder class is used to create the vehicle flows of a SUMO network. The various
    components making up the flows are added one by one, before being built using the build function.

    The class is composed of two types of functions:
    - The 'add_XXX' functions add different types of detectors to the object.
    - The 'build_XXX' functions are used to build SUMO configuration XML files based on the detectors added to the
    object beforehand. The build function generates the XML file.
    """

    def __init__(self):
        self.routes = {}
        self.v_types = {}
        self.flows = {}

    def build(self, filenames):
        """
        Generate the XML configuration files with all flow components.
        :param filenames: The dictionnary with all filenames for configuration.
        :type filenames: dict
        """
        xml_routes = ET.Element('routes')
        self.build_v_types(xml_routes)
        self.build_routes(xml_routes)
        self.build_flows(xml_routes)
        tree = ET.ElementTree(xml_routes)
        tree.write(filenames['routes'])

    def add_route(self, id, type, from_edge, to_edge):
        """
        Add a route to the flows.
        :param id: ID of route
        :type id: str
        :param type: Type of vehicle using the route
        :type type: str
        :param from_edge: ID of the starting edge of route
        :type from_edge: str
        :param to_edge: ID of ending edge of route
        :type to_edge: str
        """
        self.routes[id] = Route(id, type, from_edge, to_edge)

    def build_routes(self, xml_flows):
        """
        Build all routes in an XML object.
        :param xml_flows: The XML object where to build nodes.
        :type xml_flows: xml.etree.ElementTree.Element
        """
        for route in self.routes:
            self.routes[route].build(xml_flows)

    def add_v_type(self, id, accel=0.8, decel=4.5, length=5.0, min_gap=2.5, max_speed=55.56, sigma=0.5, tau=1.0,
                   color='yellow'):
        """
        Add a vehicle type to define a flow.
        :param id: ID of vehicle type
        :type id: str
        :param accel: Acceleration level of vehicle (in meters/seconds^2)
        :type accel: float
        :param decel: Deceleration level of vehicle (in meters/seconds^2)
        :type decel: float
        :param length: Vehicle length (in meters)
        :type length: float
        :param min_gap: Minimum space between two vehicles (in meters)
        :type min_gap: float
        :param max_speed: Max speed of vehicle (in kilometers/hour)
        :type max_speed: float
        :param sigma: The driver imperfection (0 denotes perfect driving), between 0 and 1
        :type sigma: float
        :param tau: Minimum reaction time (in seconds) of vehicle
        :type tau: float
        :param color: Color of vehicle
        :type color: str
        """
        self.v_types[id] = VType(id, accel, decel, length, min_gap, max_speed, sigma, tau, color)

    def build_v_types(self, xml_flows):
        """
        Build all vehicle types in an XML object.
        :param xml_flows: The XML object where to build vTypes.
        :type xml_flows: xml.etree.ElementTree.Element
        """
        for v_type in self.v_types.values():
            v_type.build(xml_flows)

    def add_flow(self, id, end, v_type, frequency, from_edge='', to_edge='', route='', begin=0,
                 distribution="uniform"):
        """
        Add a flow of vehicules to the flows
        :param id: ID of flow
        :type id: str
        :param route: ID of the route on which the flow will move
        :type route: str
        :param end: Ending tick of the flow
        :type end: int
        :param v_type: ID of vType of the flow
        :type v_type: str
        :param frequency: Flow frequency (in vehicles/hour)
        :type frequency: int
        :param from_edge: ID of starting edge of the flow
        :type from_edge: str
        :param to_edge: ID of ending edge of the flow
        :type to_edge: str
        :param begin: Starting tick of the flow
        :type begin: int
        :param distribution: Type of distribution of the flow. Fill with "uniform" for a uniform distribution,
        or with "binomial" for a binomial distribution. The number of vehicle per hour will be respected anyway.
        :type distribution: str
        """
        self.flows[id] = Flow(id=id, end=end, frequency=frequency, v_type=v_type, from_edge=from_edge,
                              to_edge=to_edge, route=route, begin=begin, distribution=distribution)

    def build_flows(self, xml_flows):
        """
        Build all flows in an XML object.
        :param xml_flows: The XML object where to build flows.
        :type xml_flows: xml.etree.ElementTree.Element
        """
        for flow in self.flows.values():
            flow.build(xml_flows)


class Route:
    """
    Class representing a route of a SUMO network.
    """

    def __init__(self, id, type, from_edge, to_edge):
        """
        Init of class.
        :param id: ID of route
        :type id: str
        :param type: Type of vehicle using the route
        :type type: str
        :param from_edge: ID of the starting edge of route
        :type from_edge: str
        :param to_edge: ID of ending edge of route
        :type to_edge: str
        """
        self.id = id
        self.type = type
        self.from_edge = from_edge
        self.to_edge = to_edge

    def build(self, xml_flows):
        """
        Build the route in the xml object xml_flows.
        :param xml_flows: XML object where to build the node.
        :type xml_flows: xml.etree.ElementTree.Element
        """
        ET.SubElement(xml_flows, 'route', {'id': self.id, 'type': self.type, 'edges': self.from_edge + ' ' + self.to_edge})


class VType:
    """
    Classe représentant un type de véchicule
    """

    def __init__(self, id, accel=0.8, decel=4.5, length=5.0, min_gap=2.5, max_speed=55.56, sigma=0.5, tau=1.0,
                 color='yellow'):
        """
        Init of class.
        :param id: ID of vehicle type
        :type id: str
        :param accel: Acceleration level of vehicle (in meters/seconds^2)
        :type accel: float
        :param decel: Deceleration level of vehicle (in meters/seconds^2)
        :type decel: float
        :param length: Vehicle length (in meters)
        :type length: float
        :param min_gap: Minimum space between two vehicles (in meters)
        :type min_gap: float
        :param max_speed: Max speed of vehicle (in kilometers/hour)
        :type max_speed: float
        :param sigma: The driver imperfection (0 denotes perfect driving), between 0 and 1
        :type sigma: float
        :param tau: Minimum reaction time (in seconds) of vehicle
        :type tau: float
        :param color: Color of vehicle
        :type color: str
        """
        self.id = str(id)
        self.accel = str(accel)
        self.decel = str(decel)
        self.length = str(length)
        self.min_gap = str(min_gap)
        self.max_speed = str(max_speed)
        self.sigma = str(sigma)
        self.tau = str(tau)
        self.color = color

    def build(self, xml_flows):
        """
        Build the vType in the xml object xml_flows.
        :param xml_flows: XML object where to build the node.
        :type xml_flows: xml.etree.ElementTree.Element
        """
        ET.SubElement(xml_flows, 'vType', {'id': self.id, 'accel': self.accel, 'decel': self.decel, 'length': self.length,
                                        'minGap': self.min_gap, 'maxSpeed': self.max_speed, 'sigma': self.sigma,
                                        'tau': self.tau, 'color': self.color})


class Flow:
    """
    Class representing a vehicle flow
    """

    def __init__(self, id, end, frequency, v_type, route='', from_edge='', to_edge='', begin=0,
                 distribution="uniform"):
        """
        Init of class.
        :param id: ID of flow
        :type id: str
        :param route: ID of the route on which the flow will move
        :type route: str
        :param end: Ending tick of the flow
        :type end: int
        :param v_type: ID of vType of the flow
        :type v_type: str
        :param frequency: Flow frequency (in vehicles/hour)
        :type frequency: int
        :param from_edge: ID of starting edge of the flow (if route is not set)
        :type from_edge: str
        :param to_edge: ID of ending edge of the flow (if route is not set)
        :type to_edge: str
        :param begin: Starting tick of the flow
        :type begin: int
        :param distribution: Type of distribution of the flow. Fill with "uniform" for a uniform distribution,
        or with "binomial" for a binomial distribution. The number of vehicle per hour will be respected anyway.
        :type distribution: str
        """
        self.id = id
        self.route = route
        self.number = str(((end - begin) / 3600) * frequency)
        self.begin = str(begin)
        self.end = str(end)
        self.frequency = str(frequency)
        self.from_edge = from_edge
        self.to_edge = to_edge
        self.v_type = v_type
        self.distribution = distribution

    def build(self, xml_flows):
        """
        Build the vType in the xml object xml_flows.
        :param xml_flows: XML object where to build the node.
        :type xml_flows: xml.etree.ElementTree.Element
        """
        probability = str(float(self.frequency) / 3600)
        if self.route != '' and self.from_edge == '' and self.to_edge == '':
            if self.distribution == "uniform":
                ET.SubElement(xml_flows, 'flow', {'id': self.id, 'begin': self.begin, 'end': self.end, 'route': self.route,
                                              'vehsPerHour': self.frequency, 'type': self.v_type})
            elif self.distribution == "binomial":
                ET.SubElement(xml_flows, 'flow', {'id': self.id, 'begin': self.begin, 'end': self.end, 'route': self.route,
                                              'probability': probability, 'type': self.v_type})
        elif self.route == '' and self.from_edge != '' and self.to_edge != '':
            if self.distribution == "uniform":
                ET.SubElement(xml_flows, 'flow',
                              {'id': self.id, 'begin': self.begin, 'end': self.end, 'from': self.from_edge,
                               'to': self.to_edge, 'vehsPerHour': self.frequency, 'type': self.v_type})
            elif self.distribution == "binomial":
                ET.SubElement(xml_flows, 'flow',
                              {'id': self.id, 'begin': self.begin, 'end': self.end, 'from': self.from_edge,
                               'to': self.to_edge, 'probability': probability, 'type': self.v_type})

