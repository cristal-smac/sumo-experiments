import xml.etree.ElementTree as ET

class RoutesBuilder:
    """
    Classe représentant les routes du réseau routier
    """

    def __init__(self, routes={}, vTypes={}, flows={}):
        self.routes = routes
        self.vTypes = vTypes
        self.flows = flows

    def build(self, filenames):
        """
        Construit les routes du réseau dans le fichier passé en paramètre.
        :param filename: Fichier où construire les routes
        """
        xml_routes = ET.Element('routes')
        self.build_vTypes(xml_routes)
        self.build_routes(xml_routes)
        self.build_flows(xml_routes)
        tree = ET.ElementTree(xml_routes)
        tree.write(filenames['routes'])

    def add_route(self, id, type, from_edge, to_edge):
        """
        Ajoute une route aux routes
        :param id: Identifiant de la route
        :param type: Type de véhicule pour la route
        :param from_edge: Arête de départ de la route
        :param to_edge: Arête d'arrivée de la route
        """
        self.routes[id] = Route(id, type, from_edge, to_edge)
        return self

    def build_routes(self, routes):
        """
        Construit les routes dans l'objet XML ElementTree passé en paramètre
        :param routes: Objet XML ElementTree où construire les routes
        """
        for route in self.routes:
            self.routes[route].build(routes)

    def add_vType(self, id, accel=0.8, decel=4.5, length=5.0, min_gap=2.5, max_speed=55.56, sigma=0.5, tau=1.0,
                  color='yellow'):
        """
        Ajoute un type de véhicule
        :param id: Identifiant du type de véhicule
        :param accel: Accélération du type de véhicule
        :param decel: Décélération du type de véhicule
        :param length: Longueur du type de véhicule
        :param min_gap: Espace minimum entre deux véhicules du type de véhicule
        :param max_speed: Vitesse maximale du type de véhicule
        :param sigma: Variance du type de véhicule
        :param tau: Temps de réaction du type de véhicule
        :param color: Couleur du type de véhicule
        """
        self.vTypes[id] = VType(id, accel, decel, length, min_gap, max_speed, sigma, tau, color)
        return self

    def build_vTypes(self, routes):
        """
        Construit les types de véhicules dans l'objet XML ElementTree passé en paramètre
        :param routes: Objet XML ElementTree où construire les types de véhicules
        """
        for vType in self.vTypes.values():
            vType.build(routes)

    def add_flow(self, id, end, vType, vehsPerHour, from_edge='', to_edge='', route='', begin=0,
                 distribution="uniform"):
        """
        Ajoute un flux de véhicules
        :param id: Identifiant du flux de véhicules
        :param route: Route du flux de véhicules
        :param end: Temps de fin du flux de véhicules
        :param vehsPerHour: Nombre de véhicules à envoyer par heure
        :param vType: Type de véhicule du flux de véhicules
        :param begin: Temps de début du flux de véhicules
        :param distribution: "uniform" pour une distribution uniforme des véhicules, "binomial" pour une distribution selon une loi Binomiale
        """
        self.flows[id] = Flow(id=id, end=end, vehsPerHour=vehsPerHour, vType=vType, from_edge=from_edge,
                              to_edge=to_edge, route=route, begin=begin, distribution=distribution)
        return self

    def build_flows(self, routes):
        """
        Construit les flux de véhicules dans l'objet XML ElementTree passé en paramètre
        :param routes: Objet XML ElementTree où construire les flux de véhicules
        """
        for flow in self.flows.values():
            flow.build(routes)


class Route:
    """
    Classe représentant une route du réseau routier
    """

    def __init__(self, id, type, from_edge, to_edge):
        """
        Constructeur de la classe Route
        :param id: Identifiant de la route
        :param type: Type de véhicules pour cette route
        :param edges: Arêtes de la route
        """
        self.id = id
        self.type = type
        self.from_edge = from_edge
        self.to_edge = to_edge

    def build(self, routes):
        """
        Construit la route dans l'objet XML ElementTree passé en paramètre
        :param routes: Objet XML ElementTree où construire la route
        """
        ET.SubElement(routes, 'route', {'id': self.id, 'type': self.type, 'edges': self.from_edge + ' ' + self.to_edge})


class VType:
    """
    Classe représentant un type de véchicule
    """

    def __init__(self, id, accel=0.8, decel=4.5, length=5.0, min_gap=2.5, max_speed=55.56, sigma=0.5, tau=1.0,
                 color='yellow'):
        """
        Constructeur de la classe VType
        :param id: Identifiant du type de véhicule
        :param accel: Accélération du véhicule
        :param decel: Décélération du véhicule
        :param sigma: Vitesse de changement de vitesse du véhicule
        :param length: Longueur du véhicule
        :param min_gap: Espace minimum entre deux véhicules
        :param max_speed: Vitesse maximale du véhicule
        :param color: Couleur du véhicule
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

    def build(self, routes):
        """
        Construit le type de véhicule dans l'objet XML ElementTree passé en paramètre
        :param routes: Objet XML ElementTree où construire le type de véhicule
        """
        ET.SubElement(routes, 'vType', {'id': self.id, 'accel': self.accel, 'decel': self.decel, 'length': self.length,
                                        'minGap': self.min_gap, 'maxSpeed': self.max_speed, 'sigma': self.sigma,
                                        'tau': self.tau, 'color': self.color})


class Flow:
    """
    Classe représentant un flux de véhicules
    """

    def __init__(self, id, end, vehsPerHour, vType, route='', from_edge='', to_edge='', begin=0,
                 distribution="uniform"):
        """
        Constructeur de la classe Flow
        :param id: Identifiant du flux
        :param begin: Heure de début du flux
        :param end: Heure de fin du flux
        :param vehsPerHour: Nombre de véhicules par heure
        :param type: Type de véhicule du flux
        :param distribution: Le type de distribution pour la génération des véhicules ("uniform" ou "binomial")
        """
        self.id = id
        self.route = route
        self.number = str(((end - begin) / 3600) * vehsPerHour)
        self.begin = str(begin)
        self.end = str(end)
        self.vehsPerHour = str(vehsPerHour)
        self.from_edge = from_edge
        self.to_edge = to_edge
        self.vType = vType
        self.distribution = distribution

    def build(self, flows):
        """
        Construit le flux dans l'objet XML ElementTree passé en paramètre
        :param flows: Objet XML ElementTree où construire le flux
        """
        probability = str(float(self.vehsPerHour) / 3600)
        if self.route != '' and self.from_edge == '' and self.to_edge == '':
            if self.distribution == "uniform":
                ET.SubElement(flows, 'flow', {'id': self.id, 'begin': self.begin, 'end': self.end, 'route': self.route,
                                              'vehsPerHour': self.vehsPerHour, 'type': self.vType})
            elif self.distribution == "binomial":
                ET.SubElement(flows, 'flow', {'id': self.id, 'begin': self.begin, 'end': self.end, 'route': self.route,
                                              'probability': probability, 'type': self.vType})
        elif self.route == '' and self.from_edge != '' and self.to_edge != '':
            if self.distribution == "uniform":
                ET.SubElement(flows, 'flow',
                              {'id': self.id, 'begin': self.begin, 'end': self.end, 'from': self.from_edge,
                               'to': self.to_edge, 'vehsPerHour': self.vehsPerHour, 'type': self.vType})
            elif self.distribution == "binomial":
                ET.SubElement(flows, 'flow',
                              {'id': self.id, 'begin': self.begin, 'end': self.end, 'from': self.from_edge,
                               'to': self.to_edge, 'probability': probability, 'type': self.vType})

