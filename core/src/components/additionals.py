import xml.etree.ElementTree as ET

class Additional:
    """
    Classe impémentant les éléments de configuration additionnels d'une simulation SUMO.
    """

    def __init__(self, laneAreaDetectors={}):
        self.laneAreaDetectors = laneAreaDetectors

    def build(self, filenames):
        """
        Construit les éléments additionnels du réseau dans le fichier passé en paramètre.
        :param filename: Fichier où construire les éléments additonnels
        """
        xml_additional = ET.Element('additional')
        self.build_laneAreaDetectors(xml_additional)
        tree = ET.ElementTree(xml_additional)
        tree.write(filenames['additionals'])

    def add_laneAreaDetector(self, id, lane, pos=0, end_pos=-0.1, freq=100000, file="detectors.out"):
        """
        Ajoute un Detecteur E2 aux éléments additionnels
        :param id: Identifiant du détecteur
        :param lane: Voie sur laquelle placer le détecteur
        :param pos: Point de départ du détecteur sur la voie
        :param end_pos: Point de fin du détécteur sur la voie
        :param freq: Fréquence d'activation du détecteur
        :param file: Nom du fichier de retour des informations des détecteurs
        """
        self.laneAreaDetectors[id] = LaneAreaDetector(id, lane, pos, end_pos, freq, file)
        return self

    def build_laneAreaDetectors(self, xml):
        """
        Construit les détecteurs dans l'objet XML ElementTree passé en paramètre
        :param xml_file: Objet XML ElementTree où construire les détecteurs
        """
        for detector in self.laneAreaDetectors:
            self.laneAreaDetectors[detector].build(xml)



class LaneAreaDetector:
    """
    Classe définissant un détecteur E2 (laneAreaDetector) à intégrer dans une simulation SUMO
    """

    def __init__(self, id, lane, pos, end_pos, freq, file):
        """
        Constructeur de la classe LaneAreaDetector
        :param id: Identifiant du détecteur
        :param lane: Voie sur laquelle placer le détecteur
        :param pos: Point de départ du détecteur sur la voie
        :param end_pos: Point de fin du détécteur sur la voie
        :param freq: Fréquence d'activation du détecteur
        :param file: Nom du fichier de retour des informations des détecteurs
        """
        self.id = str(id)
        self.lane = str(lane)
        self.pos = str(pos)
        self.end_pos = str(end_pos)
        self.freq = str(freq)
        self.file = str(file)

    def build(self, xml):
        """
        Construit le détecteur dans l'objet XML ElementTree passé en paramètre
        :param xml: Objet XML ElementTree où construire le détecteur
        """
        ET.SubElement(xml, 'laneAreaDetector', {'id':self.id,
                                                'lane':self.lane,
                                                'pos':self.pos,
                                                'endPos':self.end_pos,
                                                'freq':self.freq,
                                                'file':self.file})