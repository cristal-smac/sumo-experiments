import os
import random
import sys
import xml.etree.ElementTree as ET
from sumo_experiments.components import FlowBuilder, DetectorBuilder
import networkx as nx
import libsumo as traci

from sumo_experiments.preset_networks import Network


class LilleNetwork(Network):
    """
    Create the SUMO network and flows for the city of Lille.
    """

    THIS_FILE_PATH = os.path.abspath(os.path.dirname(__file__))

    NB_ROUTES_EACH_FLOW = 100
    AD_HOC_COEFFICIENT_FOR_FLOWS = 0.03

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

    TL_IDS = ['joinedS_1', '6301940736', 'cluster1681715927_250883340_274897939_6301940714', '6316129114', 'GS_cluster_1615590751_3305892794', 'GS_cluster_1544209593_1615582086',
              'joinedS_13', 'GS_cluster_1773321434_198873202', 'cluster1589650044_cluster1589650045_1793003425', 'GS_cluster_506774252_8568267669_8568267670_8568267671',
              'GS_cluster_1589656419_1770465596', 'GS_cluster_1589659457_1912530468', 'GS_cluster_1770465595_4365093914', 'GS_cluster_149374929_1770465614_305929185',
              'GS_cluster_507288390_508056193', 'cluster_485133449_485133452', 'GS_cluster_485133387_485133391', 'cluster1638086122_cluster_251058778_32828632',
              'GS_cluster_1635760230_1635775294_3706468062', '6267747172', 'GS_cluster_1650597986_1650597989_1652594655', 'GS_cluster_1648564005_1648564008', 'cluster1647146953_1647146957',
              'joinedS_4', 'J11', 'cluster1436583672_1713998722_4401347167_4401347179_#2more', 'GS_cluster_426634816_6316128557_6316128558_801903207_801903212', 'GS_cluster_243072211_250894314',
              'GS_cluster_1800186885_1800186886_243072210_3404227323_494986739', 'GS_cluster_1302128733_423805246', 'GS_cluster_267375483_3077077589', 'J02',
              '652409498', 'cluster1302567574_149374913_305918532', 'GS_cluster_2298692584_7170315064', 'joinedS_5', '491543745', 'cluster1302567586_149374901_305918149',
              'GS_cluster_506774249_5371110830', 'GS_cluster_1845506036_305930099', 'GS_cluster_1635701799_1635701803', 'GS_1713983096', 'GS_1713983087', 'joinedS_12', 'GS_1299481284',
              '133278923', 'joinedS_3', 'GS_164480279', 'GS_1656149839', 'joinedS_9', '1783585139', 'joinedS_10', '1845505945', 'GS_198873099', 'GS_225057604', '235809784', 'GS_243072205',
              'GS_243072206', 'GS_250883344', 'GS_251048925', 'GS_251050905', '251053238', '251053453', 'GS_251053667', '251053668', '251054519', '251056180', 'GS_251997373', '252269915',
              '260579122', '260579123', 'GS_267375333', '288393948', 'GS_288393949', '295718903', 'GS_3076442881', 'GS_320950907', 'GS_352836899', '362339802', 'GS_393330677',
              'GS_439772972', '439806870', '439806886', '440740820', 'GS_468704354', 'GS_4688955408', 'GS_469141720', 'GS_469309921', '469309961', 'cluster1638013968_cluster_1638013992_1656149900_4052112289',
              '471611509', 'GS_471611512', 'GS_471611514', '471611519', 'GS_476281346', '485133343', 'GS_485133383', 'GS_485133394', '485133399', '489576842',
              'GS_492092314', '492204162', '492204171', '493138763', 'GS_494986735', '494986738', '496978290', '497394917', 'GS_498476334', 'GS_506774198',
              'joinedS_16', 'GS_508063549', '517340666', '517340669', '517340672', '530710713', 'GS_6257632307', 'GS_6286429547', 'GS_8983324940', 'GS_9160219128',
              'cluster_10861016298_1299481357_1299481366_164480282_849010069', 'GS_cluster_1302128722_3075924708_8674467625_8674467626', 'cluster_1499392862_393332459_5936292019',
              'cluster_1575153557_260469525', 'GS_cluster_1628579789_243072201', 'GS_cluster_1635333815_251056169_252192622_6233070869_6233070871_6233070875_6233070881_6304582153',
              'GS_cluster_1635660842_1635660844', 'GS_cluster_1674605528_478487605', 'joinedS_17', 'GS_cluster_1705440129_250883337_251056183', 'GS_cluster_1800186890_251997370',
              'cluster_198873054_3077077638', 'cluster_198873247_506774200', 'GS_cluster_198873257_8568267658', 'cluster_2327596112_439772980_469309959', 'joinedS_2',
              'cluster_250883332_251056162', 'cluster_250889004_250896553_3076475495_3076475499', 'GS_cluster_251997367_251997369_6525168932_6525168938_8666071616',
              'joinedS_18', 'cluster_294326287_531028789_5343505332', 'GS_cluster_408501435_6315125559', 'cluster_485133368_485133379', 'GS_cluster_488539561_8567356615',
              'cluster_530710726_683650128', 'GS_cluster_6286429546_6286429552', 'cluster_6286429557_6990557731', '269243965']




    def __init__(self, intensity=1, starting_time=0, ending_time=24, seed=42):
        """
        Init of class
        :param intensity: The intensity of the normal flow. A coefficient to multiply the number of vehicle for each flow.
        :type intensity: float
        :param starting_time: The hour of the day at which the flow starts. 0 is midnight, 12 is midday and 23 is 11PM. Must be between 0 and 23.
        :type starting_time: int
        :param ending_time: The hour of the day at which the flow ends. Must be between 1 and 24, and greater than starting time.
        :type ending_time: int
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
        self.generate_flows(intensity * self.AD_HOC_COEFFICIENT_FOR_FLOWS, starting_time, ending_time, seed)
        self.generate_detectors()
        self.generate_config_file()
        self.TLS_DETECTORS = {
            'joinedS_1': self.DETECTORS_JoinedS_1,
            '6301940736': self.DETECTORS_6301940736,
            'cluster1681715927_250883340_274897939_6301940714': self.DETECTORS_cluster1681715927_250883340_274897939_6301940714,
            '6316129114': self.DETECTORS_6316129114,
            'GS_cluster_1615590751_3305892794': self.DETECTORS_GS_cluster_1615590751_3305892794,
            'GS_cluster_1544209593_1615582086': self.DETECTORS_GS_cluster_1544209593_1615582086,
            'joinedS_13': self.DETECTORS_joinedS_13,
            'GS_cluster_1773321434_198873202': self.DETECTORS_GS_cluster_1773321434_198873202,
            'cluster1589650044_cluster1589650045_1793003425': self.DETECTORS_cluster1589650044_cluster1589650045_1793003425,
            'GS_cluster_506774252_8568267669_8568267670_8568267671': self.DETECTORS_GS_cluster_506774252_8568267669_8568267670_8568267671,
            'GS_cluster_506774249_5371110830': self.DETECTORS_GS_cluster_506774249_5371110830,
            'GS_cluster_1589656419_1770465596': self.DETECTORS_GS_cluster_1589656419_1770465596,
            'GS_cluster_1589659457_1912530468': self.DETECTORS_GS_cluster_1589659457_1912530468,
            'GS_cluster_1770465595_4365093914': self.DETECTORS_GS_cluster_1770465595_4365093914,
            'GS_cluster_149374929_1770465614_305929185': self.DETECTORS_GS_cluster_149374929_1770465614_305929185,
            'GS_cluster_1845506036_305930099': self.DETECTORS_GS_cluster_1845506036_305930099,
            'GS_cluster_507288390_508056193': self.DETECTORS_GS_cluster_507288390_508056193,
            'cluster_485133449_485133452': self.DETECTORS_cluster_485133449_485133452,
            'GS_cluster_485133387_485133391': self.DETECTORS_GS_cluster_485133387_485133391,
            'cluster1638086122_cluster_251058778_32828632': self.DETECTORS_cluster1638086122_cluster_251058778_32828632,
            'GS_cluster_1635760230_1635775294_3706468062': self.DETECTORS_GS_cluster_1635760230_1635775294_3706468062,
            'GS_cluster_1635701799_1635701803': self.DETECTORS_GS_cluster_1635701799_1635701803,
            '6267747172': self.DETECTORS_6267747172,
            'GS_cluster_1650597986_1650597989_1652594655': self.DETECTORS_GS_cluster_1650597986_1650597989_1652594655,
            'GS_cluster_1648564005_1648564008': self.DETECTORS_GS_cluster_1648564005_1648564008,
            'cluster1647146953_1647146957': self.DETECTORS_cluster1647146953_1647146957,
            'joinedS_4': self.DETECTORS_joinedS_4,
            'J11': self.DETECTORS_J11,
            'GS_1713983096': self.DETECTORS_GS_1713983096,
            'GS_1713983087': self.DETECTORS_GS_1713983087,
            'cluster1436583672_1713998722_4401347167_4401347179_#2more': self.DETECTORS_cluster1436583672_1713998722_4401347167_4401347179,
            'GS_cluster_426634816_6316128557_6316128558_801903207_801903212': self.DETECTORS_GS_cluster_426634816_6316128557_6316128558_801903207_801903212,
            'GS_cluster_243072211_250894314': self.DETECTORS_GS_cluster_243072211_250894314,
            'GS_cluster_1800186885_1800186886_243072210_3404227323_494986739': self.DETECTORS_GS_cluster_1800186885_1800186886_243072210_3404227323_494986739,
            'GS_cluster_1302128733_423805246': self.DETECTORS_GS_cluster_1302128733_423805246,
            'GS_cluster_267375483_3077077589': self.DETECTORS_GS_cluster_267375483_3077077589,
            'joinedS_12': self.DETECTORS_joinedS_12,
            '652409498': self.DETECTORS_652409498,
            'cluster1302567586_149374901_305918149': self.DETECTORS_cluster1302567586_149374901_305918149,
            'cluster1302567574_149374913_305918532': self.DETECTORS_cluster1302567574_149374913_305918532,
            'GS_cluster_2298692584_7170315064': self.DETECTORS_GS_cluster_2298692584_7170315064,
            'joinedS_5': self.DETECTORS_joinedS_5,
            '491543745': self.DETECTORS_491543745,
            'GS_1299481284': self.DETECTORS_GS_1299481284,
            '133278923': self.DETECTORS_133278923,
            'joinedS_3': self.DETECTORS_joinedS_3,
            'GS_164480279': self.DETECTORS_GS_164480279,
            'GS_1656149839': self.DETECTORS_GS_1656149839,
            'joinedS_9': self.DETECTORS_joinedS_9,
            '1783585139': self.DETECTORS_1783585139,
            'joinedS_10': self.DETECTORS_joinedS_10,
            '1845505945': self.DETECTORS_1845505945,
            'GS_198873099': self.DETECTORS_GS_198873099,
            'GS_225057604': self.DETECTORS_GS_225057604,
            '235809784': self.DETECTORS_235809784,
            'GS_243072205': self.DETECTORS_GS_243072205,
            'GS_243072206': self.DETECTORS_GS_243072206,
            'GS_250883344': self.DETECTORS_GS_250883344,
            'GS_251048925': self.DETECTORS_GS_251048925,
            'GS_251050905': self.DETECTORS_GS_251050905,
            '251053238': self.DETECTORS_251053238,
            '251053453': self.DETECTORS_251053453,
            'GS_251053667': self.DETECTORS_GS_251053667,
            '251053668': self.DETECTORS_251053668,
            '251054519': self.DETECTORS_251054519,
            '251056180': self.DETECTORS_251056180,
            'GS_251997373': self.DETECTORS_GS_251997373,
            '252269915': self.DETECTORS_252269915,
            '260579122': self.DETECTORS_260579122,
            '260579123': self.DETECTORS_260579123,
            'GS_267375333': self.DETECTORS_GS_267375333,
            '288393948': self.DETECTORS_288393948,
            'GS_288393949': self.DETECTORS_GS_288393949,
            '295718903': self.DETECTORS_295718903,
            'GS_3076442881': self.DETECTORS_GS_3076442881,
            'GS_320950907': self.DETECTORS_GS_320950907,
            'GS_352836899': self.DETECTORS_GS_352836899,
            '362339802': self.DETECTORS_362339802,
            'GS_393330677': self.DETECTORS_GS_393330677,
            'GS_439772972': self.DETECTORS_GS_439772972,
            '439806870': self.DETECTORS_439806870,
            '439806886': self.DETECTORS_439806886,
            '440740820': self.DETECTORS_440740820,
            'GS_468704354': self.DETECTORS_GS_468704354,
            'GS_4688955408': self.DETECTORS_GS_4688955408,
            'GS_469141720': self.DETECTORS_GS_469141720,
            'GS_469309921': self.DETECTORS_GS_469309921,
            '469309961': self.DETECTORS_469309961,
            '471611509': self.DETECTORS_471611509,
            'GS_471611512': self.DETECTORS_GS_471611512,
            'GS_471611514': self.DETECTORS_GS_471611514,
            '471611519': self.DETECTORS_471611519,
            'GS_476281346': self.DETECTORS_GS_476281346,
            '485133343': self.DETECTORS_485133343,
            'GS_485133383': self.DETECTORS_GS_485133383,
            'GS_485133394': self.DETECTORS_GS_485133394,
            '485133399': self.DETECTORS_485133399,
            '489576842': self.DETECTORS_489576842,
            'GS_492092314': self.DETECTORS_GS_492092314,
            '492204162': self.DETECTORS_492204162,
            '492204171': self.DETECTORS_492204171,
            '493138763': self.DETECTORS_493138763,
            'GS_494986735': self.DETECTORS_GS_494986735,
            '494986738': self.DETECTORS_494986738,
            '496978290': self.DETECTORS_496978290,
            '497394917': self.DETECTORS_497394917,
            'GS_498476334': self.DETECTORS_GS_498476334,
            'GS_506774198': self.DETECTORS_GS_506774198,
            'joinedS_16': self.DETECTORS_joinedS_16,
            'GS_508063549': self.DETECTORS_GS_508063549,
            '517340666': self.DETECTORS_517340666,
            '517340669': self.DETECTORS_517340669,
            '517340672': self.DETECTORS_517340672,
            '530710713': self.DETECTORS_530710713,
            'GS_6257632307': self.DETECTORS_GS_6257632307,
            'GS_6286429547': self.DETECTORS_GS_6286429547,
            'GS_8983324940': self.DETECTORS_GS_8983324940,
            'GS_9160219128': self.DETECTORS_GS_9160219128,
            'cluster1638013968_cluster_1638013992_1656149900_4052112289': self.DETECTORS_cluster1638013968_cluster_1638013992_1656149900_4052112289,
            'cluster_10861016298_1299481357_1299481366_164480282_849010069': self.DETECTORS_cluster_10861016298_1299481357_1299481366_164480282_849010069,
            'GS_cluster_1302128722_3075924708_8674467625_8674467626': self.DETECTORS_GS_cluster_1302128722_3075924708_8674467625_8674467626,
            'cluster_1499392862_393332459_5936292019': self.DETECTORS_cluster_1499392862_393332459_5936292019,
            'cluster_1575153557_260469525': self.DETECTORS_cluster_1575153557_260469525,
            'GS_cluster_1628579789_243072201': self.DETECTORS_GS_cluster_1628579789_243072201,
            'GS_cluster_1635333815_251056169_252192622_6233070869_6233070871_6233070875_6233070881_6304582153': self.DETECTORS_GS_cluster_1635333815_251056169_252192622_6233070869_6233070871_6233070875_6233070881_6304582153,
            'GS_cluster_1635660842_1635660844': self.DETECTORS_GS_cluster_1635660842_1635660844,
            'joinedS_17': self.DETECTORS_joinedS_17,
            'GS_cluster_1674605528_478487605': self.DETECTORS_GS_cluster_1674605528_478487605,
            'GS_cluster_1705440129_250883337_251056183': self.DETECTORS_GS_cluster_1705440129_250883337_251056183,
            'GS_cluster_1800186890_251997370': self.DETECTORS_GS_cluster_1800186890_251997370,
            'cluster_198873054_3077077638': self.DETECTORS_cluster_198873054_3077077638,
            'cluster_198873247_506774200': self.DETECTORS_cluster_198873247_506774200,
            'GS_cluster_198873257_8568267658': self.DETECTORS_GS_cluster_198873257_8568267658,
            'cluster_2327596112_439772980_469309959': self.DETECTORS_cluster_2327596112_439772980_469309959,
            'joinedS_2': self.DETECTORS_joinedS_2,
            'cluster_250883332_251056162': self.DETECTORS_cluster_250883332_251056162,
            'cluster_250889004_250896553_3076475495_3076475499': self.DETECTORS_cluster_250889004_250896553_3076475495_3076475499,
            'GS_cluster_251997367_251997369_6525168932_6525168938_8666071616': self.DETECTORS_GS_cluster_251997367_251997369_6525168932_6525168938_8666071616,
            'joinedS_18': self.DETECTORS_joinedS_18,
            'cluster_294326287_531028789_5343505332': self.DETECTORS_cluster_294326287_531028789_5343505332,
            'GS_cluster_408501435_6315125559': self.DETECTORS_GS_cluster_408501435_6315125559,
            'cluster_485133368_485133379': self.DETECTORS_cluster_485133368_485133379,
            'GS_cluster_488539561_8567356615': self.DETECTORS_GS_cluster_488539561_8567356615,
            'cluster_530710726_683650128': self.DETECTORS_cluster_530710726_683650128,
            'GS_cluster_6286429546_6286429552': self.DETECTORS_GS_cluster_6286429546_6286429552,
            'cluster_6286429557_6990557731': self.DETECTORS_cluster_6286429557_6990557731,
            '269243965': self.DETECTORS_269243965,
            'J02': self.DETECTORS_J02,
        }


    def run(self, traci_function, simulation_duration=None, gui=False, seed=None, no_warnings=True, nb_threads=1, time_to_teleport=150):
        """
        Run the network.
        :param traci_function: The function using TraCi package and that can control infrastructures.
        :type: function
        :param gui: True to run SUMO in graphical mode. False otherwise.
        :type gui: bool
        :param seed: The seed of the simulation. Same seeds = same simulations.
        :type seed: int
        :param no_warnings: If set to True, no warnings when executing SUMO.
        :type no_warnings: bool
        :param nb_threads: Number of thread to run SUMO
        :type nb_threads: int
        :param time_to_teleport: The time for a vehicle to teleport when the network is blocked
        :type time_to_teleport: int
        """
        try:
            if seed is not None:
                seed_text = f'--seed {seed} '
            else:
                seed_text = '--random '
            threads_text = f'--threads {nb_threads} '
            no_warnings_text = ''
            if no_warnings:
                no_warnings_text = '--no-warnings '
            if gui:
                traci.start((self.FULL_LINE_COMMAND_GUI + f' --time-to-teleport {time_to_teleport} ' + threads_text + seed_text + no_warnings_text).split())
            else:
                traci.start((self.FULL_LINE_COMMAND + f' --time-to-teleport {time_to_teleport} ' + threads_text + seed_text + no_warnings_text).split())
            res = traci_function(traci)
            traci.close()
        except Exception as err:
            print("Error during simulation :", sys.exc_info()[0])
            print("OS error: {0}".format(err))
            res = None
        self.clean_files()
        return res

    def generate_flows(self, intensity=1, starting_time=0, ending_time=24, seed=42):
        """
        Generate flows for the network.
        :param intensity: The intensity of the normal flow. A coefficient to multiply the number of vehicle for each flow.
        :type intensity: float
        :param starting_time: The hour of the day at which the flow starts. 0 is midnight, 12 is midday and 23 is 11PM. Must be between 0 and 23.
        :type starting_time: int
        :param ending_time: The hour of the day at which the flow ends. Must be between 1 and 24, and greater than starting time.
        :type ending_time: int
        :param seed: The seed of the random generator to create flows
        :type seed: int
        :return: The flows
        :rtype: FlowBuilder
        """
        random_generator = random.Random(seed)
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
                        exit = random_generator.choices(edges)[0]
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
        det.add_lane_area_detector(id='6316129114_b9', edge='295265888#0', lane=0, pos=141, end_pos=161, type='boolean')
        det.add_lane_area_detector(id='6316129114_b10', edge='295265888#0', lane=1, pos=141, end_pos=161, type='boolean')
        det.add_lane_area_detector(id='6316129114_b11', edge='295241795#0', lane=0, pos=128, end_pos=148, type='boolean')
        det.add_lane_area_detector(id='6316129114_b12', edge='295241795#0', lane=1, pos=128, end_pos=148, type='boolean')
        det.add_lane_area_detector(id='6316129114_b13', edge='267674184#7', lane=0, pos=30, end_pos=50, type='boolean')
        det.add_lane_area_detector(id='6316129114_b14', edge='267674184#7', lane=1, pos=30, end_pos=50, type='boolean')
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
        det.add_lane_area_detector(id='6316129114_n13', edge='295265888#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n14', edge='295265888#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n15', edge='295241795#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n16', edge='295241795#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n17', edge='267674184#7', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n18', edge='267674184#7', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n19', edge='267674184#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='6316129114_n20', edge='267674184#5', lane=1, end_pos=-1, type='numerical')
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
        det.add_lane_area_detector(id='cluster1638013968_b1', edge='366657788#0', lane=0, pos=140, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638013968_b2', edge='366657788#0', lane=1, pos=140, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638013968_b3', edge='40554478#0', lane=0, pos=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638013968_b4', edge='40554478#0', lane=1, pos=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638013968_b5', edge='23209684#0', lane=0, pos=134, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638013968_b6', edge='23209684#0', lane=1, pos=134, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster1638013968_s1', edge='366657788#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638013968_s2', edge='366657788#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638013968_s3', edge='40237005', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638013968_s4', edge='23209684#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638013968_s5', edge='23209684#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster1638013968_n1', edge='366657788#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638013968_n2', edge='366657788#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638013968_n3', edge='40554478#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638013968_n4', edge='40554478#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638013968_n5', edge='40237005', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638013968_n6', edge='23209684#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster1638013968_n7', edge='23209684#0', lane=1, end_pos=-1, type='numerical')
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
        # TLS joinedS_12
        det.add_lane_area_detector(id='joinedS_12_b1', edge='276335210#0', lane=0, pos=118, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b2', edge='276335210#0', lane=1, pos=118, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b3', edge='276335210#0', lane=2, pos=118, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b4', edge='694271872#4', lane=0, pos=45, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b5', edge='694271872#4', lane=1, pos=45, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b6', edge='204817782#0', lane=0, pos=124, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b7', edge='204817782#0', lane=1, pos=124, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b8', edge='204817782#0', lane=2, pos=124, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_12_b9', edge='671150562#0', lane=0, pos=15, end_pos=-1, type='boolean')
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
        det.add_lane_area_detector(id='joinedS_12_n7', edge='694271872#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n8', edge='694271872#4', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n9', edge='694271872#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n10', edge='694271872#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n11', edge='694271872#3', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n12', edge='694271872#0', lane=0, pos=78, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n13', edge='694271872#0', lane=1, pos=78, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n14', edge='204817782#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n15', edge='204817782#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n16', edge='204817782#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n17', edge='177480212', lane=0, pos=75, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n18', edge='177480212', lane=1, pos=75, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n19', edge='177480212', lane=2, pos=75, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n20', edge='671150562#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n21', edge='681552697', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n22', edge='126505997', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_12_n23', edge='32258211', lane=0, end_pos=-1, type='numerical')
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
        det.add_lane_area_detector(id='joinedS_5_b3', edge='1133747103#0', lane=0, pos=5, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b4', edge='1081841963#0', lane=0, pos=35, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b5', edge='1081841963#0', lane=1, pos=35, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b6', edge='927571006#0', lane=0, pos=37, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b7', edge='927571006#0', lane=1, pos=37, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b8', edge='927571006#0', lane=2, pos=37, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b9', edge='787899593#0', lane=0, pos=21, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_5_b10', edge='787899593#0', lane=1, pos=21, end_pos=-1, type='boolean')
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
        det.add_lane_area_detector(id='joinedS_5_n3', edge='1133747103#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n4', edge='-147396980#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n5', edge='-147396980#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n6', edge='1081841963#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n7', edge='1081841963#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n8', edge='40401871#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n9', edge='927571006#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n10', edge='927571006#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n11', edge='927571006#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n12', edge='39725932#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n13', edge='39725932#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n14', edge='39725932#1', lane=0, pos=34, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n15', edge='39725932#1', lane=1, pos=34, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n16', edge='787899593#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n17', edge='787899593#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n18', edge='306136474#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n19', edge='306136474#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n20', edge='306136474#2', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n21', edge='306136474#2', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n22', edge='306136474#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n23', edge='306136474#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n24', edge='306136474#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_5_n25', edge='306136474#0', lane=3, end_pos=-1, type='numerical')
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
        # GS_1299481284
        det.add_lane_area_detector(id='GS_1299481284_b1', edge='181636453#0', lane=0, pos=37, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_1299481284_b2', edge='181636453#0', lane=1, pos=37, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_1299481284_b3', edge='46656368#0', lane=0, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_1299481284_b4', edge='46656368#0', lane=1, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_1299481284_s1', edge='114800007', lane=0, pos=7, end_pos=27, type='saturation')
        det.add_lane_area_detector(id='GS_1299481284_s2', edge='114800007', lane=1, pos=7, end_pos=27, type='saturation')
        det.add_lane_area_detector(id='GS_1299481284_s3', edge='46656516#0', lane=0, pos=47, end_pos=67, type='saturation')
        det.add_lane_area_detector(id='GS_1299481284_s4', edge='46656516#0', lane=1, pos=47, end_pos=67, type='saturation')
        det.add_lane_area_detector(id='GS_1299481284_n1', edge='181636453#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1299481284_n2', edge='181636453#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1299481284_n3', edge='114800007', lane=0, pos=7, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1299481284_n4', edge='114800007', lane=1, pos=7, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1299481284_n5', edge='46656368#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1299481284_n6', edge='46656368#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1299481284_n7', edge='46656516#0-AddedOffRampEdge', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1299481284_n8', edge='46656516#0-AddedOffRampEdge', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1299481284_n9', edge='46656516#0', lane=0, pos=47, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1299481284_n10', edge='46656516#0', lane=1, pos=47, end_pos=-1, type='numerical')
        # 133278923
        det.add_lane_area_detector(id='133278923_b1', edge='246426373#5', lane=0, pos=59, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='133278923_b2', edge='246426373#5', lane=1, pos=59, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='133278923_b3', edge='145895351#3', lane=0, pos=43, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='133278923_b4', edge='145895351#3', lane=1, pos=43, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='133278923_s1', edge='246426373#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='133278923_s2', edge='246426373#2', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='133278923_s3', edge='145895351#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='133278923_s4', edge='145895351#3', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='133278923_n1', edge='246426373#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='133278923_n2', edge='246426373#5', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='133278923_n3', edge='246426373#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='133278923_n4', edge='246426373#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='133278923_n5', edge='145895351#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='133278923_n6', edge='145895351#3', lane=1, end_pos=-1, type='numerical')
        # joinedS_3
        det.add_lane_area_detector(id='joinedS_3_b1', edge='40203861#0', lane=0, pos=6, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_3_b2', edge='40203861#0', lane=1, pos=6, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_3_b3', edge='40203861#0', lane=2, pos=6, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_3_b4', edge='70991116', lane=0, pos=12, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_3_b5', edge='70991116', lane=1, pos=12, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_3_b6', edge='70991116', lane=2, pos=12, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_3_b7', edge='-40236821#3', lane=0, pos=49, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_3_b8', edge='40203859#1', lane=0, pos=21, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_3_b9', edge='40203859#1', lane=1, pos=21, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_3_s1', edge='40236834#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_3_s2', edge='40236834#4', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_3_s3', edge='-40236821#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_3_s4', edge='40203859#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_3_s5', edge='40203859#1', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_3_n1', edge='40203861#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_3_n2', edge='40203861#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_3_n3', edge='40203861#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_3_n4', edge='70991116', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_3_n5', edge='70991116', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_3_n6', edge='70991116', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_3_n7', edge='40236834#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_3_n8', edge='40236834#4', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_3_n9', edge='-40236821#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_3_n10', edge='40203859#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_3_n11', edge='40203859#1', lane=1, end_pos=-1, type='numerical')
        # GS_164480279
        det.add_lane_area_detector(id='GS_164480279_b1', edge='16207741#1', lane=0, pos=45, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_164480279_b2', edge='-484396465#3', lane=0, pos=137, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_164480279_b3', edge='-1167807980#3', lane=0, pos=74, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_164480279_s1', edge='16207741#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_164480279_s2', edge='-484396465#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_164480279_s3', edge='-1167807980#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_164480279_n1', edge='16207741#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_164480279_n2', edge='16207741#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_164480279_n3', edge='-484396465#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_164480279_n4', edge='-1167807980#3', lane=0, end_pos=-1, type='numerical')
        # GS_1656149839
        det.add_lane_area_detector(id='GS_1656149839_b1', edge='-150547012#1', lane=0, pos=55, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_1656149839_b2', edge='1050785991#0', lane=0, pos=33, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_1656149839_b3', edge='1050785991#0', lane=1, pos=33, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_1656149839_b4', edge='1021069943#2', lane=0, pos=64, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_1656149839_s1', edge='-150547012#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_1656149839_s2', edge='209877626#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_1656149839_s3', edge='209877626#1', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_1656149839_s4', edge='1021069943#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_1656149839_n1', edge='-150547012#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1656149839_n2', edge='-150547012#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1656149839_n3', edge='1050785991#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1656149839_n4', edge='1050785991#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1656149839_n5', edge='1050786416#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1656149839_n6', edge='209877626#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1656149839_n7', edge='209877626#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1656149839_n8', edge='1021069943#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_1656149839_n9', edge='1021069943#0', lane=0, end_pos=-1, type='numerical')
        # joinedS_9
        det.add_lane_area_detector(id='joinedS_9_b1', edge='674324361#2', lane=0, pos=61, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9_b2', edge='674324361#2', lane=1, pos=61, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9_b3', edge='165600269#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9_b4', edge='165600269#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9_b5', edge='103066467#1', lane=0, pos=10, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9_b6', edge='-165600269#5', lane=0, pos=66, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9_b7', edge='-165600269#2', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9_b8', edge='41502369#0', lane=0, pos=13, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9_b9', edge='41502369#0', lane=1, pos=13, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_9_s1', edge='674324361#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_9_s2', edge='674324361#2', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_9_s3', edge='103066467#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_9_s4', edge='-180790926#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_9_s5', edge='15117711#0', lane=0, pos=63, end_pos=83, type='saturation')
        det.add_lane_area_detector(id='joinedS_9_n1', edge='674324361#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9_n2', edge='674324361#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9_n3', edge='165600269#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9_n4', edge='165600269#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9_n5', edge='103066467#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9_n6', edge='-165600269#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9_n7', edge='-165600269#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9_n8', edge='-180790926#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9_n9', edge='-180790926#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9_n10', edge='41502369#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9_n11', edge='41502369#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_9_n12', edge='15117711#0', lane=0, pos=63, end_pos=-1, type='numerical')
        # 1783585139
        det.add_lane_area_detector(id='1783585139_b1', edge='709681981#0', lane=0, pos=172, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1783585139_b2', edge='41432791#0', lane=0, pos=172, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1783585139_b3', edge='41432776#0', lane=0, pos=76, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1783585139_b4', edge='41432776#0', lane=1, pos=76, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1783585139_s1', edge='709681981#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1783585139_s2', edge='41432791#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1783585139_s3', edge='41432776#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1783585139_s4', edge='41432776#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1783585139_n1', edge='709681981#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1783585139_n2', edge='41432791#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1783585139_n3', edge='41432776#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1783585139_n4', edge='41432776#0', lane=1, end_pos=-1, type='numerical')
        # joinedS_10
        det.add_lane_area_detector(id='joinedS_10_b1', edge='167175382#0', lane=0, pos=284, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_10_b2', edge='312521185#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_10_b3', edge='312521185#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_10_b4', edge='224900500#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_10_b5', edge='224900500#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_10_s1', edge='167175382#0', lane=0, pos=104, end_pos=124, type='saturation')
        det.add_lane_area_detector(id='joinedS_10_s2', edge='23282409', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_10_s3', edge='23282409', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_10_s4', edge='23282409', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_10_s5', edge='303291937', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_10_s6', edge='303291937', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_10_n1', edge='167175382#0', lane=0, pos=104, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_10_n2', edge='312521185#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_10_n3', edge='312521185#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_10_n4', edge='23282409', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_10_n5', edge='23282409', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_10_n6', edge='23282409', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_10_n7', edge='224900500#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_10_n8', edge='224900500#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_10_n9', edge='303291937', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_10_n10', edge='303291937', lane=1, end_pos=-1, type='numerical')
        # 1845505945
        det.add_lane_area_detector(id='1845505945_b1', edge='-173794839#0', lane=0, pos=35, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1845505945_b2', edge='857245401#7', lane=0, pos=112, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1845505945_b3', edge='-147572086#4', lane=0, pos=47, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='1845505945_s1', edge='-992570225#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1845505945_s2', edge='857245401#7', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1845505945_s3', edge='-147572086#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='1845505945_n1', edge='-173794839#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1845505945_n2', edge='-173794839#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1845505945_n3', edge='-992570225#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1845505945_n4', edge='857245401#7', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='1845505945_n5', edge='-147572086#4', lane=0, end_pos=-1, type='numerical')
        # GS_198873099
        det.add_lane_area_detector(id='GS_198873099_b1', edge='312521175#0', lane=0, pos=5, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_198873099_b2', edge='438130798#0', lane=0, pos=96, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_198873099_s1', edge='312521175#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_198873099_s2', edge='438130798#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_198873099_n1', edge='312521175#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_198873099_n2', edge='438130798#0', lane=0, end_pos=-1, type='numerical')
        # GS_225057604
        det.add_lane_area_detector(id='GS_225057604_b1', edge='-103066477#2', lane=0, pos=37, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_225057604_b2', edge='-103066477#2', lane=1, pos=37, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_225057604_b3', edge='992570225#0', lane=0, pos=18, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_225057604_b4', edge='992570225#0', lane=1, pos=18, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_225057604_b5', edge='-131751140#2', lane=0, pos=57, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_225057604_s1', edge='-548098364#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_225057604_s2', edge='173794839#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_225057604_s3', edge='-131751140#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_225057604_n1', edge='-103066477#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_225057604_n2', edge='-103066477#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_225057604_n3', edge='-548098364#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_225057604_n4', edge='992570225#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_225057604_n5', edge='992570225#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_225057604_n6', edge='173794839#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_225057604_n7', edge='-131751140#2', lane=0, end_pos=-1, type='numerical')
        # 235809784
        det.add_lane_area_detector(id='235809784_b1', edge='147677592#0', lane=0, pos=28, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='235809784_b2', edge='-21883171#1', lane=0, pos=48, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='235809784_b3', edge='180787926#3', lane=0, pos=151, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='235809784_s1', edge='147677592#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='235809784_s2', edge='-21883171#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='235809784_s3', edge='180787926#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='235809784_n1', edge='147677592#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='235809784_n2', edge='-21883171#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='235809784_n3', edge='180787926#3', lane=0, end_pos=-1, type='numerical')
        # GS_243072205
        det.add_lane_area_detector(id='GS_243072205_b1', edge='-1020857159#7', lane=0, pos=119, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_243072205_b2', edge='22662375', lane=0, pos=72, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_243072205_b3', edge='22662375', lane=1, pos=72, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_243072205_b4', edge='935166817#2', lane=0, pos=82, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_243072205_b5', edge='-169970249#1', lane=0, pos=156, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_243072205_s1', edge='-1020857159#7', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_243072205_s2', edge='84690886#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_243072205_s3', edge='935166817#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_243072205_s4', edge='-169970249#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_243072205_n1', edge='-1020857159#7', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_243072205_n2', edge='22662375', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_243072205_n3', edge='22662375', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_243072205_n4', edge='84690886#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_243072205_n5', edge='935166817#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_243072205_n6', edge='-169970249#1', lane=0, end_pos=-1, type='numerical')
        # GS_243072206
        det.add_lane_area_detector(id='GS_243072206_b1', edge='-23209407#6', lane=0, pos=84, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_243072206_b2', edge='135931683#0', lane=0, pos=156, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_243072206_b3', edge='1176629322#0', lane=0, pos=88, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_243072206_b4', edge='-206133857#1', lane=0, pos=74, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_243072206_s1', edge='-23209407#6', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_243072206_s2', edge='135931683#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_243072206_s3', edge='1176629322#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_243072206_s4', edge='-206133857#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_243072206_n1', edge='-23209407#6', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_243072206_n2', edge='135931683#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_243072206_n3', edge='1176629322#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_243072206_n4', edge='-206133857#1', lane=0, end_pos=-1, type='numerical')
        # GS_250883344
        det.add_lane_area_detector(id='GS_250883344_b1', edge='440531862', lane=0, pos=30, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_250883344_b2', edge='440531862', lane=1, pos=30, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_250883344_b3', edge='356256047#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_250883344_b4', edge='356256047#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_250883344_s1', edge='440531862', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_250883344_s2', edge='440531862', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_250883344_s3', edge='23197997', lane=0, pos=181, end_pos=201, type='saturation')
        det.add_lane_area_detector(id='GS_250883344_n1', edge='440531862', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_250883344_n2', edge='440531862', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_250883344_n3', edge='316680837', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_250883344_n4', edge='316680837', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_250883344_n5', edge='356256047#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_250883344_n6', edge='356256047#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_250883344_n7', edge='23197997', lane=0, pos=181, end_pos=-1, type='numerical')
        # GS_251048925
        det.add_lane_area_detector(id='GS_251048925_b1', edge='95631681#2', lane=0, pos=51, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_251048925_b2', edge='-95067483#4', lane=0, pos=63, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_251048925_b3', edge='-95631681#5', lane=0, pos=124, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_251048925_b4', edge='95067483#1', lane=0, pos=78, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_251048925_s1', edge='95631681#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_251048925_s2', edge='-95067483#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_251048925_s3', edge='-95631681#5', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_251048925_s4', edge='95067483#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_251048925_n1', edge='95631681#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_251048925_n2', edge='-95067483#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_251048925_n3', edge='-95631681#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_251048925_n4', edge='95067483#1', lane=0, end_pos=-1, type='numerical')
        # GS_251050905
        det.add_lane_area_detector(id='GS_251050905_b1', edge='880131127#0', lane=0, pos=25, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_251050905_b2', edge='120226485#0', lane=0, pos=128, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_251050905_b3', edge='26484576#0', lane=0, pos=99, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_251050905_s1', edge='880131127#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_251050905_s2', edge='120226485#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_251050905_s3', edge='26484576#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_251050905_n1', edge='880131127#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_251050905_n2', edge='120226485#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_251050905_n3', edge='26484576#0', lane=0, end_pos=-1, type='numerical')
        # 251053238
        det.add_lane_area_detector(id='251053238_b1', edge='670994200#0', lane=0, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='251053238_b2', edge='-220781185#7', lane=0, pos=89, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='251053238_b3', edge='24028294#0', lane=0, pos=95, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='251053238_s1', edge='670994200#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='251053238_s2', edge='-220781185#7', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='251053238_s3', edge='24028294#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='251053238_n1', edge='670994200#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='251053238_n2', edge='-220781185#7', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='251053238_n3', edge='24028294#0', lane=0, end_pos=-1, type='numerical')
        # 251053453
        det.add_lane_area_detector(id='251053453_b1', edge='484396456#5', lane=0, pos=87, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='251053453_b2', edge='-673365084#4', lane=0, pos=81, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='251053453_b3', edge='25674419#0', lane=0, pos=190, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='251053453_s1', edge='484396456#5', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='251053453_s2', edge='-673365084#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='251053453_s3', edge='25674419#0', lane=0, pos=10, end_pos=30, type='saturation')
        det.add_lane_area_detector(id='251053453_n1', edge='484396456#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='251053453_n2', edge='-673365084#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='251053453_n3', edge='25674419#0', lane=0, end_pos=-1, type='numerical')
        # GS_251053667
        det.add_lane_area_detector(id='GS_251053667_b1', edge='674959068#1', lane=0, pos=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_251053667_b2', edge='484396454#0', lane=0, pos=77, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_251053667_b3', edge='-484396453#3', lane=0, pos=172, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_251053667_s1', edge='674959068#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_251053667_s2', edge='484396454#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_251053667_s3', edge='-484396453#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_251053667_n1', edge='674959068#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_251053667_n2', edge='484396454#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_251053667_n3', edge='-484396453#3', lane=0, end_pos=-1, type='numerical')
        # 251053668
        det.add_lane_area_detector(id='251053668_b1', edge='-97596158#6', lane=0, pos=200, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='251053668_b2', edge='123593237#2', lane=0, pos=7, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='251053668_b3', edge='97596159#4', lane=0, pos=112, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='251053668_s1', edge='-97596158#6', lane=0, pos=20, end_pos=40, type='saturation')
        det.add_lane_area_detector(id='251053668_s2', edge='123593237#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='251053668_s3', edge='97596159#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='251053668_n1', edge='-97596158#6', lane=0, pos=20, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='251053668_n2', edge='123593237#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='251053668_n3', edge='97596159#4', lane=0, end_pos=-1, type='numerical')
        # 251054519
        det.add_lane_area_detector(id='251054519_b1', edge='84690889#1', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='251054519_b2', edge='84690889#1', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='251054519_b3', edge='40995422#1', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='251054519_s1', edge='84690889#1', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='251054519_s2', edge='84690889#1', lane=1, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='251054519_s3', edge='40995422#1', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='251054519_n1', edge='84690889#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='251054519_n2', edge='84690889#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='251054519_n3', edge='40995422#1', lane=0, end_pos=-1, type='numerical')
        # 251056180
        det.add_lane_area_detector(id='251056180_b1', edge='87607969#2', lane=0, pos=54, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='251056180_b2', edge='289351515#0', lane=0, pos=58, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='251056180_s1', edge='87607969#2', lane=0, pos =0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='251056180_s2', edge='685973537', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='251056180_s3', edge='685973537', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='251056180_n1', edge='87607969#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='251056180_n2', edge='289351515#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='251056180_n3', edge='685973537', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='251056180_n4', edge='685973537', lane=1, end_pos=-1, type='numerical')
        # GS_251997373
        det.add_lane_area_detector(id='GS_251997373_b1', edge='-312521170#5', lane=0, pos=174, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_251997373_b2', edge='23209407#0', lane=0, pos=83, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_251997373_b3', edge='641711251#0', lane=0, pos=69, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_251997373_b4', edge='641711251#0', lane=1, pos=69, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_251997373_s1', edge='-312521170#5', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_251997373_s2', edge='23209407#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_251997373_s3', edge='23282411#0', lane=0, pos=64, end_pos=84, type='saturation')
        det.add_lane_area_detector(id='GS_251997373_n1', edge='-312521170#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_251997373_n2', edge='23209407#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_251997373_n3', edge='641711251#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_251997373_n4', edge='641711251#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_251997373_n5', edge='23282411#0', lane=0, pos=64, end_pos=-1, type='numerical')
        # 252269915
        det.add_lane_area_detector(id='252269915_b1', edge='880143899', lane=0, pos=51, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='252269915_b2', edge='162138548#0', lane=0, pos=120, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='252269915_s1', edge='880143899', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='252269915_s2', edge='162138548#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='252269915_n1', edge='880143899', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='252269915_n2', edge='162138548#0', lane=0, end_pos=-1, type='numerical')
        # 260579122
        det.add_lane_area_detector(id='260579122_b1', edge='673365084#0', lane=0, pos=81, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='260579122_b2', edge='115202991#1', lane=0, pos=103, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='260579122_b3', edge='97596158#0', lane=0, pos=200, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='260579122_s1', edge='673365084#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='260579122_s2', edge='115202991#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='260579122_s3', edge='97596158#0', lane=0, pos=20, end_pos=40, type='saturation')
        det.add_lane_area_detector(id='260579122_n1', edge='673365084#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='260579122_n2', edge='115202991#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='260579122_n3', edge='97596158#0', lane=0, pos=20, end_pos=-1, type='numerical')
        # 260579123
        det.add_lane_area_detector(id='260579123_b1', edge='118086263#5', lane=0, pos=55, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='260579123_b2', edge='191002371', lane=0, pos=43, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='260579123_b3', edge='97596152#1', lane=0, pos=18, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='260579123_b4', edge='97596152#1', lane=1, pos=18, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='260579123_s1', edge='118086263#5', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='260579123_s2', edge='191002371', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='260579123_s3', edge='97596152#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='260579123_s4', edge='97596152#1', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='260579123_n1', edge='118086263#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='260579123_n2', edge='191002371', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='260579123_n3', edge='97596152#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='260579123_n4', edge='97596152#1', lane=1, end_pos=-1, type='numerical')
        # GS_267375333
        det.add_lane_area_detector(id='GS_267375333_b1', edge='152814630#3', lane=0, pos=80, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_267375333_b2', edge='87607965#4', lane=0, pos=78, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_267375333_b3', edge='-157848623#1', lane=0, pos=52, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_267375333_b4', edge='41847984#0', lane=0, pos=87, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_267375333_s1', edge='152814630#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_267375333_s2', edge='87607965#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_267375333_s3', edge='-157848623#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_267375333_s4', edge='41847984#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_267375333_n1', edge='152814630#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_267375333_n2', edge='87607965#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_267375333_n3', edge='-157848623#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_267375333_n4', edge='41847984#0', lane=0, end_pos=-1, type='numerical')
        # 288393948
        det.add_lane_area_detector(id='288393948_b1', edge='246426373#0', lane=0, pos=3, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='288393948_b2', edge='246426373#0', lane=1, pos=3, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='288393948_b3', edge='234880762#0', lane=0, pos=107, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='288393948_s1', edge='246426373#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='288393948_s2', edge='246426373#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='288393948_s3', edge='234880762#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='288393948_n1', edge='246426373#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='288393948_n2', edge='246426373#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='288393948_n3', edge='234880762#0', lane=0, end_pos=-1, type='numerical')
        # GS_288393949
        det.add_lane_area_detector(id='GS_288393949_b1', edge='833002651#5', lane=0, pos=111, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_288393949_b2', edge='1158931710#5', lane=0, pos=40, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_288393949_b3', edge='168275984#1', lane=0, pos=73, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_288393949_s1', edge='833002651#5', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_288393949_s2', edge='1158931710#5', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_288393949_s3', edge='168275984#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_288393949_n1', edge='833002651#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_288393949_n2', edge='1158931710#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_288393949_n3', edge='168275984#1', lane=0, end_pos=-1, type='numerical')
        # 295718903
        det.add_lane_area_detector(id='295718903_b1', edge='26980716#0', lane=0, pos=116, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='295718903_b2', edge='151991095#5', lane=0, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='295718903_b3', edge='-26980716#1', lane=0, pos=21, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='295718903_b4', edge='-151991095#7', lane=0, pos=58, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='295718903_s1', edge='26980716#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='295718903_s2', edge='151991095#5', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='295718903_s3', edge='-26980716#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='295718903_s4', edge='-151991095#7', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='295718903_n1', edge='26980716#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='295718903_n2', edge='151991095#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='295718903_n3', edge='-26980716#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='295718903_n4', edge='-151991095#7', lane=0, end_pos=-1, type='numerical')
        # GS_3076442881
        det.add_lane_area_detector(id='GS_3076442881_b1', edge='24204778#0', lane=0, pos=48, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_3076442881_b2', edge='671285245', lane=0, pos=46, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_3076442881_s1', edge='24204778#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_3076442881_s2', edge='671285245', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_3076442881_n1', edge='24204778#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_3076442881_n2', edge='671285245', lane=0, end_pos=-1, type='numerical')
        # GS_320950907
        det.add_lane_area_detector(id='GS_320950907_b1', edge='29187029#0', lane=0, pos=36, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_320950907_b2', edge='29187029#0', lane=1, pos=36, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_320950907_b3', edge='29185600#0', lane=0, pos=81, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_320950907_s1', edge='29187029#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_320950907_s2', edge='29187029#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_320950907_s3', edge='29185600#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_320950907_n1', edge='29187029#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_320950907_n2', edge='29187029#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_320950907_n3', edge='29185600#0', lane=0, end_pos=-1, type='numerical')
        # GS_352836899
        det.add_lane_area_detector(id='GS_352836899_b1', edge='23209072#0', lane=0, pos=17, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_352836899_b2', edge='94527181#1', lane=0, pos=117, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_352836899_b3', edge='-95631681#1', lane=0, pos=83, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_352836899_b4', edge='-94527140#1', lane=0, pos=134, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_352836899_s1', edge='23209072#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_352836899_s2', edge='94527181#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_352836899_s3', edge='-95631681#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_352836899_s4', edge='-94527140#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_352836899_n1', edge='23209072#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_352836899_n2', edge='94527181#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_352836899_n3', edge='-95631681#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_352836899_n4', edge='-94527140#1', lane=0, end_pos=-1, type='numerical')
        # 362339802
        det.add_lane_area_detector(id='362339802_b1', edge='-460236443#3', lane=0, pos=209, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='362339802_b2', edge='117119292#0', lane=0, pos=8, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='362339802_b3', edge='693976858#3', lane=0, pos=120, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='362339802_s1', edge='-460236443#3', lane=0, pos=29, end_pos=49, type='saturation')
        det.add_lane_area_detector(id='362339802_s2', edge='673321100', lane=0, pos=36, end_pos=56, type='saturation')
        det.add_lane_area_detector(id='362339802_s3', edge='24028303#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='362339802_s4', edge='693976858#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='362339802_n1', edge='-460236443#3', lane=0, pos=29, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='362339802_n2', edge='117119292#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='362339802_n3', edge='673321100', lane=0, pos=36, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='362339802_n4', edge='24028303#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='362339802_n5', edge='693976858#3', lane=0, end_pos=-1, type='numerical')
        # GS_393330677
        det.add_lane_area_detector(id='GS_393330677_b1', edge='686637168#0', lane=0, pos=44, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_393330677_b2', edge='686637168#0', lane=1, pos=44, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_393330677_b3', edge='686637168#0', lane=2, pos=44, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_393330677_b4', edge='686637168#0', lane=3, pos=44, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_393330677_b5', edge='23298863#0', lane=0, pos=31, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_393330677_b6', edge='23298863#0', lane=1, pos=31, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_393330677_b7', edge='23298863#0', lane=2, pos=31, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_393330677_s1', edge='23298864', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='GS_393330677_s2', edge='23298864', lane=1, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='GS_393330677_s3', edge='23298863#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_393330677_s4', edge='23298863#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_393330677_s5', edge='23298863#0', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_393330677_n1', edge='686637168#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_393330677_n2', edge='686637168#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_393330677_n3', edge='686637168#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_393330677_n4', edge='686637168#0', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_393330677_n5', edge='23298864', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_393330677_n6', edge='23298864', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_393330677_n7', edge='23298863#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_393330677_n8', edge='23298863#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_393330677_n9', edge='23298863#0', lane=2, end_pos=-1, type='numerical')
        # GS_439772972
        det.add_lane_area_detector(id='GS_439772972_b1', edge='392793710#4', lane=0, pos=133, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_439772972_b2', edge='766710290#10', lane=0, pos=25, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_439772972_b3', edge='-766710290#13', lane=0, pos=46, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_439772972_s1', edge='392793710#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_439772972_s2', edge='766710290#10', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_439772972_s3', edge='-766710290#13', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_439772972_n1', edge='392793710#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_439772972_n2', edge='766710290#10', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_439772972_n3', edge='-766710290#13', lane=0, end_pos=-1, type='numerical')
        # 439806870
        det.add_lane_area_detector(id='439806870_b1', edge='180787918#0', lane=0, pos=44, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='439806870_b2', edge='180787918#0', lane=1, pos=44, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='439806870_b3', edge='-37548828', lane=0, pos=213, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='439806870_s1', edge='180787918#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='439806870_s2', edge='180787918#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='439806870_s3', edge='-37548828', lane=0, pos=33, end_pos=53, type='saturation')
        det.add_lane_area_detector(id='439806870_n1', edge='180787918#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='439806870_n2', edge='180787918#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='439806870_n3', edge='-37548828', lane=0, pos=33, end_pos=-1, type='numerical')
        # 439806886
        det.add_lane_area_detector(id='439806886_b1', edge='-95565302#1', lane=0, pos=31, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='439806886_b2', edge='147677590#0', lane=0, pos=28, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='439806886_b3', edge='37548828', lane=0, pos=213, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='439806886_s1', edge='-95565302#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='439806886_s2', edge='147677590#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='439806886_s3', edge='37548828', lane=0, pos=33, end_pos=53, type='saturation')
        det.add_lane_area_detector(id='439806886_n1', edge='-95565302#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='439806886_n2', edge='147677590#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='439806886_n3', edge='37548828', lane=0, end_pos=-1, type='numerical')
        # 440740820
        det.add_lane_area_detector(id='440740820_b1', edge='95631681#4', lane=0, pos=124, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='440740820_b2', edge='151991095#8', lane=0, pos=11, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='440740820_b3', edge='-95631681#6', lane=0, pos=30, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='440740820_b4', edge='-151991095#10', lane=0, pos=68, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='440740820_s1', edge='95631681#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='440740820_s2', edge='151991095#6', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='440740820_s3', edge='-95631681#6', lane=0, pos=0, end_pos=13, type='saturation')
        det.add_lane_area_detector(id='440740820_s4', edge='-152025223#1', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='440740820_s5', edge='-151991095#10', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='440740820_n1', edge='95631681#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='440740820_n2', edge='151991095#6', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='440740820_n3', edge='151991095#8', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='440740820_n4', edge='-95631681#6', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='440740820_n5', edge='-152025223#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='440740820_n6', edge='-151991095#10', lane=0, end_pos=-1, type='numerical')
        # GS_468704354
        det.add_lane_area_detector(id='GS_468704354_b1', edge='766710290#15', lane=0, pos=16, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_468704354_b2', edge='155602430#7', lane=0, pos=145, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_468704354_b3', edge='-766710290#20', lane=0, pos=100, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_468704354_s1', edge='766710290#15', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_468704354_s2', edge='155602430#7', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_468704354_s3', edge='-766710290#20', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_468704354_n1', edge='766710290#15', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_468704354_n2', edge='155602430#7', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_468704354_n3', edge='-766710290#20', lane=0, end_pos=-1, type='numerical')
        # GS_4688955408
        det.add_lane_area_detector(id='GS_4688955408_b1', edge='39725933#0', lane=0, pos=66, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_4688955408_b2', edge='39725933#0', lane=1, pos=66, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_4688955408_b3', edge='39725933#0', lane=2, pos=66, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_4688955408_b4', edge='475086607#10', lane=0, pos=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_4688955408_b5', edge='475086607#10', lane=1, pos=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_4688955408_b6', edge='475086607#10', lane=2, pos=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_4688955408_s1', edge='39725933#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_4688955408_s2', edge='39725933#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_4688955408_s3', edge='39725933#0', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_4688955408_s4', edge='475086607#10', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_4688955408_s5', edge='475086607#10', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_4688955408_s6', edge='475086607#10', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_4688955408_n1', edge='39725933#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_4688955408_n2', edge='39725933#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_4688955408_n3', edge='39725933#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_4688955408_n4', edge='475086607#10', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_4688955408_n5', edge='475086607#10', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_4688955408_n6', edge='475086607#10', lane=2, end_pos=-1, type='numerical')
        # GS_469141720
        det.add_lane_area_detector(id='GS_469141720_b1', edge='155602430#4', lane=0, pos=73, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_469141720_b2', edge='-39166410#1', lane=0, pos=76, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_469141720_s1', edge='155602430#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_469141720_s2', edge='-39166410#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_469141720_n1', edge='155602430#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_469141720_n2', edge='-39166410#1', lane=0, end_pos=-1, type='numerical')
        # GS_469309921
        det.add_lane_area_detector(id='GS_469309921_b1', edge='766710290#27', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_469309921_b2', edge='39178017#3', lane=0, pos=108, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_469309921_b3', edge='-1158931710#1', lane=0, pos=102, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_469309921_s1', edge='766710290#25', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_469309921_s2', edge='39178017#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_469309921_s3', edge='-1158931710#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_469309921_n1', edge='766710290#27', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_469309921_n2', edge='766710290#25', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_469309921_n3', edge='39178017#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_469309921_n4', edge='-1158931710#1', lane=0, end_pos=-1, type='numerical')
        # 469309961
        det.add_lane_area_detector(id='469309961_b1', edge='833002650#4', lane=0, pos=61, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='469309961_b2', edge='147572086#2', lane=0, pos=47, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='469309961_b3', edge='147572085#2', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='469309961_s1', edge='833002650#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='469309961_s2', edge='147572086#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='469309961_s3', edge='147572085#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='469309961_n1', edge='833002650#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='469309961_n2', edge='147572086#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='469309961_n3', edge='147572085#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='469309961_n4', edge='147572085#0', lane=0, end_pos=-1, type='numerical')
        # 471611509
        det.add_lane_area_detector(id='471611509_b1', edge='1185722342#0', lane=0, pos=14, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='471611509_b2', edge='89606453#0', lane=0, pos=58, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='471611509_s1', edge='1185722342#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='471611509_s2', edge='89606453#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='471611509_n1', edge='1185722342#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='471611509_n2', edge='89606453#0', lane=0, end_pos=-1, type='numerical')
        # GS_471611512
        det.add_lane_area_detector(id='GS_471611512_b1', edge='392793712#0', lane=0, pos=15, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_471611512_b2', edge='39363178#0', lane=0, pos=119, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_471611512_s1', edge='392793712#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_471611512_s2', edge='39363178#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_471611512_n1', edge='392793712#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_471611512_n2', edge='39363178#0', lane=0, end_pos=-1, type='numerical')
        # GS_471611514
        det.add_lane_area_detector(id='GS_471611514_b1', edge='102621877#10', lane=0, pos=132, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_471611514_b2', edge='180790927#0', lane=0, pos=156, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_471611514_b3', edge='-39148839#1', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_471611514_s1', edge='102621877#10', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_471611514_s2', edge='180790927#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_471611514_s3', edge='-39148839#1', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='GS_471611514_n1', edge='102621877#10', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_471611514_n2', edge='180790927#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_471611514_n3', edge='-39148839#1', lane=0, end_pos=-1, type='numerical')
        # 471611519
        det.add_lane_area_detector(id='471611519_b1', edge='833002651#3', lane=0, pos=77, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='471611519_b2', edge='-88899637#3', lane=0, pos=24, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='471611519_b3', edge='88899637#1', lane=0, pos=81, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='471611519_s1', edge='833002651#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='471611519_s2', edge='-88899637#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='471611519_s3', edge='88899637#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='471611519_n1', edge='833002651#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='471611519_n2', edge='-88899637#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='471611519_n3', edge='88899637#1', lane=0, end_pos=-1, type='numerical')
        # GS_476281346
        det.add_lane_area_detector(id='GS_476281346_b1', edge='39878318#1', lane=0, pos=211, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_476281346_b2', edge='57241977#5', lane=0, pos=76, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_476281346_s1', edge='39878318#1', lane=0, pos=31, end_pos=51, type='saturation')
        det.add_lane_area_detector(id='GS_476281346_s2', edge='57241977#5', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_476281346_n1', edge='39878318#1', lane=0, pos=31, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_476281346_n2', edge='57241977#5', lane=0, end_pos=-1, type='numerical')
        # 485133343
        det.add_lane_area_detector(id='485133343_b1', edge='-150547012#9', lane=0, pos=56, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='485133343_b2', edge='150547012#6', lane=0, pos=4, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='485133343_b3', edge='-102621877#1', lane=0, pos=65, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='485133343_s1', edge='-150547012#9', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='485133343_s2', edge='150547012#6', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='485133343_s3', edge='-102621877#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='485133343_n1', edge='-150547012#9', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='485133343_n2', edge='150547012#6', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='485133343_n3', edge='-102621877#1', lane=0, end_pos=-1, type='numerical')
        # GS_485133383
        det.add_lane_area_detector(id='GS_485133383_b1', edge='668861332#1', lane=0, pos=72, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_485133383_b2', edge='102621877#8', lane=0, pos=42, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_485133383_b3', edge='-102621877#13', lane=0, pos=132, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_485133383_s1', edge='668861332#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_485133383_s2', edge='102621877#8', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_485133383_s3', edge='-102621877#13', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_485133383_n1', edge='668861332#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_485133383_n2', edge='102621877#8', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_485133383_n3', edge='-102621877#13', lane=0, end_pos=-1, type='numerical')
        # GS_485133394
        det.add_lane_area_detector(id='GS_485133394_b1', edge='180787914#0', lane=0, pos=62, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_485133394_b2', edge='150950348#0', lane=0, pos=102, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_485133394_b3', edge='-392793711#2', lane=0, pos=22, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_485133394_s1', edge='180787914#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_485133394_s2', edge='150950348#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_485133394_s3', edge='-392793711#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_485133394_n1', edge='180787914#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_485133394_n2', edge='150950348#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_485133394_n3', edge='-392793711#2', lane=0, end_pos=-1, type='numerical')
        # 485133399
        det.add_lane_area_detector(id='485133399_b1', edge='392793710#2', lane=0, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='485133399_b2', edge='1185722340', lane=0, pos=54, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='485133399_s1', edge='392793710#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='485133399_s2', edge='1185722340', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='485133399_n1', edge='392793710#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='485133399_n2', edge='1185722340', lane=0, end_pos=-1, type='numerical')
        # 489576842
        det.add_lane_area_detector(id='489576842_b1', edge='-302704693#4', lane=0, pos=7, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='489576842_b2', edge='880143900#1', lane=0, pos=12, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='489576842_b3', edge='302704693#3', lane=0, pos=98, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='489576842_b4', edge='-47705161#3', lane=0, pos=128, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='489576842_s1', edge='-302704693#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='489576842_s2', edge='880143900#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='489576842_s3', edge='302704693#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='489576842_s4', edge='-47705161#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='489576842_n1', edge='-302704693#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='489576842_n2', edge='880143900#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='489576842_n3', edge='302704693#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='489576842_n4', edge='1185722340', lane=0, end_pos=-1, type='numerical')
        # GS_492092314
        det.add_lane_area_detector(id='GS_492092314_b1', edge='-147396980#15', lane=0, pos=22, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_492092314_b2', edge='40550667#2', lane=0, pos=28, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_492092314_b3', edge='147396980#5', lane=0, pos=144, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_492092314_s1', edge='-147396980#15', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_492092314_s2', edge='40550667#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_492092314_s3', edge='147396980#5', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_492092314_n1', edge='-147396980#15', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_492092314_n2', edge='40550667#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_492092314_n3', edge='147396980#5', lane=0, end_pos=-1, type='numerical')
        # 492204162
        det.add_lane_area_detector(id='492204162_b1', edge='40554475#1', lane=0, pos=203, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='492204162_b2', edge='392793714#2', lane=0, pos=61, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='492204162_b3', edge='392793716#4', lane=0, pos=100, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='492204162_s1', edge='40554475#1', lane=0, pos=23, end_pos=43, type='saturation')
        det.add_lane_area_detector(id='492204162_s2', edge='392793714#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='492204162_s3', edge='392793716#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='492204162_n1', edge='40554475#1', lane=0, pos=23, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='492204162_n2', edge='392793714#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='492204162_n3', edge='392793716#4', lane=0, end_pos=-1, type='numerical')
        # 492204171
        det.add_lane_area_detector(id='492204171_b1', edge='39178010#4', lane=0, pos=126, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='492204171_b2', edge='392793716#0', lane=0, pos=60, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='492204171_s1', edge='39178010#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='492204171_s2', edge='392793716#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='492204171_n1', edge='39178010#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='492204171_n2', edge='392793716#0', lane=0, end_pos=-1, type='numerical')
        # 493138763
        det.add_lane_area_detector(id='493138763_b1', edge='40605749#0', lane=0, pos=216, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='493138763_b2', edge='40666176#0', lane=0, pos=85, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='493138763_b3', edge='40666176#0', lane=1, pos=85, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='493138763_s1', edge='40605749#0', lane=0, pos=36, end_pos=56, type='saturation')
        det.add_lane_area_detector(id='493138763_s2', edge='40666173#1', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='493138763_n1', edge='40605749#0', lane=0, pos=36, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='493138763_n2', edge='40666176#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='493138763_n3', edge='40666176#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='493138763_n4', edge='40666173#1', lane=0, end_pos=-1, type='numerical')
        # GS_494986735
        det.add_lane_area_detector(id='GS_494986735_b1', edge='84690879#0', lane=0, pos=53, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_494986735_b2', edge='84690879#0', lane=1, pos=53, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_494986735_b3', edge='102143189#0', lane=0, pos=64, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_494986735_s1', edge='84690879#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_494986735_s2', edge='84690879#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_494986735_s3', edge='102143189#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_494986735_n1', edge='84690879#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_494986735_n2', edge='84690879#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_494986735_n3', edge='102143189#0', lane=0, end_pos=-1, type='numerical')
        # 494986738
        det.add_lane_area_detector(id='494986738_b1', edge='-147572085#1', lane=0, pos=24, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='494986738_b2', edge='-665071996#3', lane=0, pos=63, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='494986738_b3', edge='1158931703#1', lane=0, pos=14, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='494986738_b4', edge='665071996#0', lane=0, pos=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='494986738_s1', edge='-147572085#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='494986738_s2', edge='-665071996#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='494986738_s3', edge='1158931703#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='494986738_s4', edge='665071996#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='494986738_n1', edge='-147572085#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='494986738_n2', edge='-665071996#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='494986738_n3', edge='1158931703#1', lane=0,  end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='494986738_n4', edge='665071996#0', lane=0, end_pos=-1, type='numerical')
        # 496978290
        det.add_lane_area_detector(id='496978290_b1', edge='-165742064#3', lane=0, pos=130, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='496978290_b2', edge='40848252#0', lane=0, pos=324, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='496978290_b3', edge='-165742420#2', lane=0, pos=142, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='496978290_s1', edge='-165742064#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='496978290_s2', edge='40848252#0', lane=0, pos=144, end_pos=164, type='saturation')
        det.add_lane_area_detector(id='496978290_s3', edge='-165742420#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='496978290_n1', edge='-165742064#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='496978290_n2', edge='40848252#0', lane=0, pos=144, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='496978290_n3', edge='-165742420#2', lane=0, end_pos=-1, type='numerical')
        # 497394917
        det.add_lane_area_detector(id='497394917_b1', edge='61482193#2', lane=0, pos=88, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='497394917_b2', edge='40848258#0', lane=0, pos=75, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='497394917_b3', edge='40876931#2', lane=0, pos=48, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='497394917_s1', edge='61482193#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='497394917_s2', edge='40848258#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='497394917_s3', edge='40876931#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='497394917_n1', edge='61482193#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='497394917_n2', edge='40848258#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='497394917_n3', edge='40876931#2', lane=0, end_pos=-1, type='numerical')
        # GS_498476334
        det.add_lane_area_detector(id='GS_498476334_b1', edge='-147396980#18', lane=0, pos=63, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_498476334_b2', edge='147396980#12', lane=0, pos=22, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_498476334_b3', edge='815430469#0', lane=0, pos=21, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_498476334_s1', edge='-147396980#18', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_498476334_s2', edge='147396980#12', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_498476334_s3', edge='815430469#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_498476334_n1', edge='-147396980#18', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_498476334_n2', edge='147396980#12', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_498476334_n3', edge='815430469#0', lane=0, end_pos=-1, type='numerical')
        # GS_506774198
        det.add_lane_area_detector(id='GS_506774198_b1', edge='131751140#3', lane=0, pos=16, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_506774198_b2', edge='89606444#0', lane=0, pos=52, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_506774198_b3', edge='-936924400#2', lane=0, pos=26, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_506774198_s1', edge='131751140#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_506774198_s2', edge='89606444#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_506774198_s3', edge='833002649#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_506774198_n1', edge='131751140#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_506774198_n2', edge='89606444#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_506774198_n3', edge='-936924400#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_506774198_n4', edge='833002649#1', lane=0, end_pos=-1, type='numerical')
        # joinedS_16
        det.add_lane_area_detector(id='joinedS_16_b1', edge='41432777#0', lane=0, pos=240, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_16_b2', edge='1020593791#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_16_b3', edge='1020593791#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_16_b4', edge='1020593791#0', lane=2, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_16_b5', edge='-832969436', lane=0, pos=151, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_16_b6', edge='-180787926#2', lane=0, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_16_b7', edge='922904465#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_16_b8', edge='922904465#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_16_b9', edge='111512089#2', lane=0, pos=98, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_16_b10', edge='145894912#0', lane=0, pos=165, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_16_b11', edge='180787926#0', lane=0, pos=19, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_16_b12', edge='50997880#0', lane=0, pos=3, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_16_s1', edge='41432777#0', lane=0, pos=60, end_pos=80, type='saturation')
        det.add_lane_area_detector(id='joinedS_16_s2', edge='-832969436', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_16_s3', edge='922904465#0', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='joinedS_16_s4', edge='922904465#0', lane=1, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='joinedS_16_s5', edge='111512089#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_16_s6', edge='145894912#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_16_n1', edge='41432777#0', lane=0, pos=60, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_16_n2', edge='1020593791#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_16_n3', edge='1020593791#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_16_n4', edge='1020593791#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_16_n5', edge='-832969436', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_16_n6', edge='-180787926#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_16_n7', edge='922904465#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_16_n8', edge='922904465#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_16_n9', edge='111512089#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_16_n10', edge='145894912#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_16_n11', edge='180787926#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_16_n12', edge='50997880#0', lane=0, end_pos=-1, type='numerical')
        # GS_508063549
        det.add_lane_area_detector(id='GS_508063549_b1', edge='-41503512#2', lane=0, pos=11, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_508063549_b2', edge='548098364#0', lane=0, pos=67, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_508063549_s1', edge='-41503512#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_508063549_s2', edge='103066477#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_508063549_n1', edge='-41503512#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_508063549_n2', edge='548098364#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_508063549_n3', edge='103066477#0', lane=0, end_pos=-1, type='numerical')
        # 517340666
        det.add_lane_area_detector(id='517340666_b1', edge='23303682#1', lane=0, pos=194, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='517340666_b2', edge='41847985#0', lane=0, pos=322, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='517340666_s1', edge='23303682#1', lane=0, pos=14, end_pos=34, type='saturation')
        det.add_lane_area_detector(id='517340666_s2', edge='41847985#0', lane=0, pos=142, end_pos=162, type='saturation')
        det.add_lane_area_detector(id='517340666_n1', edge='23303682#1', lane=0, pos=14, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='517340666_n2', edge='41847985#0', lane=0, pos=142, end_pos=-1, type='numerical')
        # 517340669
        det.add_lane_area_detector(id='517340669_b1', edge='41847988#3', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='517340669_b2', edge='23303682#0', lane=0, pos=58, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='517340669_s1', edge='41847988#3', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='517340669_s2', edge='23303682#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='517340669_n1', edge='41847988#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='517340669_n2', edge='23303682#0', lane=0, end_pos=-1, type='numerical')
        # 517340672
        det.add_lane_area_detector(id='517340672_b1', edge='296422468', lane=0, pos=77, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='517340672_b2', edge='296422468', lane=1, pos=77, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='517340672_b3', edge='41847986#0', lane=0, pos=255, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='517340672_s1', edge='296422468', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='517340672_s2', edge='296422468', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='517340672_s3', edge='41847986#0', lane=0, pos=75, end_pos=95, type='saturation')
        det.add_lane_area_detector(id='517340672_n1', edge='296422468', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='517340672_n2', edge='296422468', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='517340672_n3', edge='41847986#0', lane=0, pos=75, end_pos=-1, type='numerical')
        # 530710713
        det.add_lane_area_detector(id='530710713_b1', edge='87607973#2', lane=0, pos=36, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='530710713_b2', edge='23209551#6', lane=0, pos=36, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='530710713_s1', edge='87607973#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='530710713_s2', edge='23209551#6', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='530710713_n1', edge='87607973#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='530710713_n2', edge='23209551#6', lane=0, end_pos=-1, type='numerical')
        # GS_6257632307
        det.add_lane_area_detector(id='GS_6257632307_b1', edge='475086607#7', lane=0, pos=23, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_6257632307_b2', edge='475086607#7', lane=1, pos=23, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_6257632307_b3', edge='475086607#7', lane=2, pos=23, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_6257632307_b4', edge='978801828#1', lane=0, pos=6, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_6257632307_b5', edge='978801828#1', lane=1, pos=6, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_6257632307_b6', edge='978801828#1', lane=2, pos=6, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_6257632307_s1', edge='475086607#7', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_6257632307_s2', edge='475086607#7', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_6257632307_s3', edge='475086607#7', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_6257632307_s4', edge='978801828#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_6257632307_s5', edge='978801828#1', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_6257632307_s6', edge='978801828#1', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_6257632307_n1', edge='475086607#7', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_6257632307_n2', edge='475086607#7', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_6257632307_n3', edge='475086607#7', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_6257632307_n4', edge='978801828#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_6257632307_n5', edge='978801828#1', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_6257632307_n6', edge='978801828#1', lane=2, end_pos=-1, type='numerical')
        # GS_6286429547
        det.add_lane_area_detector(id='GS_6286429547_b1', edge='22324796', lane=0, pos=130, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_6286429547_b2', edge='22324796', lane=1, pos=130, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_6286429547_b3', edge='114869616', lane=0, pos=170, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_6286429547_b4', edge='114869616', lane=1, pos=170, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_6286429547_b5', edge='671285253#0', lane=0, pos=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_6286429547_b6', edge='671285253#0', lane=1, pos=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_6286429547_s1', edge='22324796', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_6286429547_s2', edge='22324796', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_6286429547_s3', edge='114869616', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_6286429547_s4', edge='114869616', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_6286429547_n1', edge='22324796', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_6286429547_n2', edge='22324796', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_6286429547_n3', edge='114869616', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_6286429547_n4', edge='114869616', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_6286429547_n5', edge='671285253#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_6286429547_n6', edge='671285253#0', lane=1, end_pos=-1, type='numerical')
        # GS_8983324940
        det.add_lane_area_detector(id='GS_8983324940_b1', edge='143367789#0', lane=0, pos=47, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_8983324940_b2', edge='-26484578#2', lane=0, pos=57, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_8983324940_s1', edge='143367789#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_8983324940_s2', edge='-26484578#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_8983324940_n1', edge='143367789#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_8983324940_n2', edge='-26484578#2', lane=0, end_pos=-1, type='numerical')
        # GS_9160219128
        det.add_lane_area_detector(id='GS_9160219128_b1', edge='180787921#0', lane=0, pos=116, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_9160219128_b2', edge='41432784#4', lane=0, pos=15, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_9160219128_b3', edge='-41432784#10', lane=0, pos=88, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_9160219128_s1', edge='180787921#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_9160219128_s2', edge='41432784#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_9160219128_s3', edge='-41432784#10', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_9160219128_n1', edge='180787921#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_9160219128_n2', edge='41432784#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_9160219128_n3', edge='-41432784#10', lane=0, end_pos=-1, type='numerical')
        # cluster_10861016298_1299481357_1299481366_164480282_849010069
        det.add_lane_area_detector(id='cluster_10861016298_b1', edge='1167807980#0', lane=0, pos=74, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_10861016298_b2', edge='673633320#3', lane=0, pos=67, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_10861016298_b3', edge='673633320#3', lane=1, pos=67, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_10861016298_b4', edge='118972291#0', lane=0, pos=33, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_10861016298_b5', edge='118972291#0', lane=1, pos=33, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_10861016298_b6', edge='118972291#0', lane=2, pos=33, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_10861016298_s1', edge='1167807980#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_10861016298_s2', edge='673633320#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_10861016298_s3', edge='673633320#3', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_10861016298_s4', edge='114800035#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_10861016298_s5', edge='114800035#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_10861016298_n1', edge='1167807980#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_10861016298_n2', edge='673633320#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_10861016298_n3', edge='673633320#3', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_10861016298_n4', edge='118972291#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_10861016298_n5', edge='118972291#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_10861016298_n6', edge='118972291#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_10861016298_n7', edge='114800035#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_10861016298_n8', edge='114800035#0', lane=1, end_pos=-1, type='numerical')
        # GS_cluster_1302128722_3075924708_8674467625_8674467626
        det.add_lane_area_detector(id='cluster_1302128722_b1', edge='967782641#0', lane=0, pos=16, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1302128722_b2', edge='967782641#0', lane=1, pos=16, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1302128722_b3', edge='936141520', lane=0, pos=65, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1302128722_b4', edge='114822970#0', lane=0, pos=179, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1302128722_s1', edge='967782641#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1302128722_s2', edge='967782641#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1302128722_s3', edge='-936141522', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1302128722_s4', edge='114822970#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1302128722_n1', edge='967782641#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1302128722_n2', edge='967782641#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1302128722_n3', edge='936141520', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1302128722_n4', edge='-936141522', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1302128722_n5', edge='114822970#0', lane=0, end_pos=-1, type='numerical')
        # cluster_1499392862_393332459_5936292019
        det.add_lane_area_detector(id='cluster_1499392862_b1', edge='202818248#4', lane=0, pos=3, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1499392862_b2', edge='202818248#4', lane=1, pos=3, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1499392862_b3', edge='679040266', lane=0, pos=248, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1499392862_b4', edge='679040266', lane=1, pos=248, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1499392862_s1', edge='1019211659#0-AddedOffRampEdge', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1499392862_s2', edge='1019211659#0-AddedOffRampEdge', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1499392862_s3', edge='1019211659#0-AddedOffRampEdge', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1499392862_s4', edge='679040266', lane=0, pos=68, end_pos=88, type='saturation')
        det.add_lane_area_detector(id='cluster_1499392862_s5', edge='679040266', lane=1, pos=68, end_pos=88, type='saturation')
        det.add_lane_area_detector(id='cluster_1499392862_n1', edge='202818248#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1499392862_n2', edge='202818248#4', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1499392862_n3', edge='1019211659#0-AddedOffRampEdge', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1499392862_n4', edge='1019211659#0-AddedOffRampEdge', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1499392862_n5', edge='1019211659#0-AddedOffRampEdge', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1499392862_n6', edge='679040266', lane=0, pos=68, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1499392862_n7', edge='679040266', lane=1, pos=68, end_pos=-1, type='numerical')
        # cluster_1575153557_260469525
        det.add_lane_area_detector(id='cluster_1575153557_b1', edge='132691252', lane=0, pos=66, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1575153557_b2', edge='167980920#1', lane=0, pos=103, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1575153557_b3', edge='40875620#5', lane=0, pos=25, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1575153557_b4', edge='1262937360#0', lane=0, pos=93, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_1575153557_s1', edge='132691252', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1575153557_s2', edge='167980920#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1575153557_s3', edge='40875620#5', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1575153557_s4', edge='1262937360#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_1575153557_n1', edge='132691252', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1575153557_n2', edge='167980920#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1575153557_n3', edge='40875620#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_1575153557_n4', edge='1262937360#0', lane=0, end_pos=-1, type='numerical')
        # GS_cluster_1628579789_243072201
        det.add_lane_area_detector(id='GS_cluster_1628579789_b1', edge='167980920#2', lane=0, pos=85, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1628579789_b2', edge='1020857159#10', lane=0, pos=50, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1628579789_b3', edge='640391933#6', lane=0, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1628579789_s1', edge='167980920#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1628579789_s2', edge='1020857159#10', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1628579789_s3', edge='640391933#6', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1628579789_n1', edge='167980920#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1628579789_n2', edge='1020857159#10', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1628579789_n3', edge='640391933#6', lane=0, end_pos=-1, type='numerical')
        # GS_cluster_1635333815_251056169_252192622_6233070869_6233070871_6233070875_6233070881_6304582153
        det.add_lane_area_detector(id='GS_cluster_1635333815_b1', edge='157848623#6', lane=0, pos=41, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1635333815_b2', edge='685973540#1', lane=0, pos=61, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1635333815_b3', edge='665859713#0', lane=0, pos=48, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1635333815_b4', edge='665859713#0', lane=1, pos=48, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1635333815_b5', edge='679034480#0', lane=0, pos=3, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1635333815_b6', edge='679034480#0', lane=1, pos=3, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1635333815_s1', edge='157848623#6', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1635333815_s2', edge='685973540#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1635333815_s3', edge='665859713#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1635333815_s4', edge='665859713#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1635333815_s5', edge='296422467#5', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1635333815_n1', edge='157848623#6', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1635333815_n2', edge='685973540#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1635333815_n3', edge='665859713#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1635333815_n4', edge='665859713#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1635333815_n5', edge='679034480#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1635333815_n6', edge='679034480#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1635333815_n7', edge='296422467#5', lane=0, end_pos=-1, type='numerical')
        # joinedS_17
        det.add_lane_area_detector(id='joinedS_17_b1', edge='151021288#0', lane=0, pos=25, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_17_b2', edge='151021288#0', lane=1, pos=25, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_17_b3', edge='625017978#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_17_b4', edge='625017978#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_17_b5', edge='40236830#0', lane=0, pos=74, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_17_b6', edge='40236830#0', lane=1, pos=74, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_17_b7', edge='40236830#0', lane=2, pos=74, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_17_b8', edge='151021290#0', lane=0, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_17_b9', edge='151021290#0', lane=1, pos=34, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_17_b10', edge='151021291#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_17_b11', edge='151021291#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_17_b12', edge='40236833#2', lane=0, pos=13, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_17_b13', edge='40236833#2', lane=1, pos=13, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_17_s1', edge='180818488#0', lane=0, pos=104, end_pos=124, type='saturation')
        det.add_lane_area_detector(id='joinedS_17_s2', edge='40236830#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_17_s3', edge='40236830#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_17_s4', edge='40236830#0', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_17_s5', edge='150547012#7', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_17_s6', edge='40236833#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_17_s7', edge='40236833#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_17_n1', edge='151021288#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n2', edge='151021288#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n3', edge='180818488#0', lane=0, pos=104, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n4', edge='625017978#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n5', edge='625017978#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n6', edge='40236830#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n7', edge='40236830#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n8', edge='40236830#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n9', edge='151021290#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n10', edge='151021290#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n11', edge='150547012#7', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n12', edge='151021291#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n13', edge='151021291#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n14', edge='40236833#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n15', edge='40236833#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n16', edge='40236833#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_17_n17', edge='40236833#0', lane=1, end_pos=-1, type='numerical')
        # GS_cluster_1674605528_478487605
        det.add_lane_area_detector(id='GS_cluster_1674605528_b1', edge='155013415#0', lane=0, pos=29, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1674605528_b2', edge='97564172#2', lane=0, pos=103, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1674605528_b3', edge='1074305289#0', lane=0, pos=130, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1674605528_s1', edge='-39878326#0', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1674605528_s2', edge='97564172#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1674605528_s3', edge='1074305289#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1674605528_n1', edge='155013415#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1674605528_n2', edge='-39878326#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1674605528_n3', edge='97564172#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1674605528_n4', edge='1074305289#0', lane=0, end_pos=-1, type='numerical')
        # GS_cluster_1705440129_250883337_251056183
        det.add_lane_area_detector(id='GS_cluster_1705440129_b1', edge='40435656#4', lane=0, pos=23, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1705440129_b2', edge='150636523#0', lane=0, pos=195, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1705440129_b3', edge='289351515#1', lane=0, pos=144, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1705440129_s1', edge='40435656#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1705440129_s2', edge='150636523#0', lane=0, pos=15, end_pos=35, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1705440129_s3', edge='289351515#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1705440129_n1', edge='40435656#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1705440129_n2', edge='40435656#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1705440129_n3', edge='150636523#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1705440129_n4', edge='289351515#1', lane=0, end_pos=-1, type='numerical')
        # GS_cluster_1800186890_251997370
        det.add_lane_area_detector(id='GS_cluster_1800186890_b1', edge='312521184#0', lane=0, pos=18, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1800186890_b2', edge='312521184#0', lane=1, pos=18, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1800186890_b3', edge='206133656#0', lane=0, pos=4, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1800186890_b4', edge='206133656#0', lane=1, pos=4, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1800186890_b5', edge='664910822', lane=0, pos=71, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_1800186890_s1', edge='312521184#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1800186890_s2', edge='312521184#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1800186890_s3', edge='206133857#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1800186890_s4', edge='664910822', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_1800186890_n1', edge='312521184#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1800186890_n2', edge='312521184#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1800186890_n3', edge='206133656#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1800186890_n4', edge='206133656#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1800186890_n5', edge='206133857#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_1800186890_n6', edge='664910822', lane=0, end_pos=-1, type='numerical')
        # cluster_198873054_3077077638
        det.add_lane_area_detector(id='cluster_198873054_b1', edge='43734375', lane=0, pos=144, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_198873054_b2', edge='717491830#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_198873054_b3', edge='435171001#0', lane=0, pos=302, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_198873054_b4', edge='90825927', lane=0, pos=66, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_198873054_s1', edge='43734375', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_198873054_s2', edge='435171001#0', lane=0, pos=124, end_pos=144, type='saturation')
        det.add_lane_area_detector(id='cluster_198873054_s3', edge='115132694', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_198873054_n1', edge='43734375', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_198873054_n2', edge='717491830#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_198873054_n3', edge='435171001#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_198873054_n4', edge='90825927', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_198873054_n5', edge='-303354750', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_198873054_n6', edge='-936141519', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_198873054_n7', edge='115132694', lane=0, end_pos=-1, type='numerical')
        # cluster_198873247_506774200
        det.add_lane_area_detector(id='cluster_198873247_b1', edge='709681531#0', lane=0, pos=67, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_198873247_b2', edge='-709681531#3', lane=0, pos=48, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_198873247_b3', edge='14037915', lane=0, pos=162, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_198873247_b4', edge='497944366#1', lane=0, pos=69, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_198873247_s1', edge='709681531#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_198873247_s2', edge='-709681531#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_198873247_s3', edge='14037915', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_198873247_s4', edge='497944366#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_198873247_n1', edge='709681531#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_198873247_n2', edge='-709681531#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_198873247_n3', edge='14037915', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_198873247_n4', edge='497944366#1', lane=0, end_pos=-1, type='numerical')
        # GS_cluster_198873257_8568267658
        det.add_lane_area_detector(id='GS_cluster_198873257_b1', edge='687315299#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_198873257_b2', edge='687315299#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_198873257_b3', edge='673717423#0', lane=0, pos=32, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_198873257_b4', edge='673717423#0', lane=1, pos=32, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_198873257_b5', edge='-131751145#3', lane=0, pos=116, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_198873257_b6', edge='673717426#1', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_198873257_s1', edge='687315299#0', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_198873257_s2', edge='687315299#0', lane=1, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_198873257_s3', edge='673717423#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_198873257_s4', edge='673717423#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_198873257_s5', edge='-131751145#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_198873257_s6', edge='-923010605#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_198873257_n1', edge='687315299#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_198873257_n2', edge='687315299#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_198873257_n3', edge='673717423#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_198873257_n4', edge='673717423#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_198873257_n5', edge='-131751145#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_198873257_n6', edge='673717426#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_198873257_n7', edge='-923010605#0', lane=0, end_pos=-1, type='numerical')
        # cluster_2327596112_439772980_469309959
        det.add_lane_area_detector(id='cluster_2327596112_b1', edge='-1158931703#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_2327596112_b2', edge='766710290#29', lane=0, pos=103, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_2327596112_b3', edge='95565302#2', lane=0, pos=54, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_2327596112_b4', edge='-1158931710#3', lane=0, pos=77, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_2327596112_s1', edge='-1158931703#0', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='cluster_2327596112_s2', edge='766710290#29', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_2327596112_s3', edge='95565302#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_2327596112_s4', edge='-1158931710#3', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_2327596112_n1', edge='-1158931703#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_2327596112_n2', edge='766710290#29', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_2327596112_n3', edge='95565302#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_2327596112_n4', edge='-1158931710#3', lane=0, end_pos=-1, type='numerical')
        # cluster_250883332_251056162
        det.add_lane_area_detector(id='cluster_250883332_b1', edge='120970425#2', lane=0, pos=176, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_250883332_b2', edge='120970425#2', lane=1, pos=176, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_250883332_b3', edge='40513089#2', lane=0, pos=40, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_250883332_b4', edge='296422467#0', lane=0, pos=245, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_250883332_s1', edge='120970425#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_250883332_s2', edge='120970425#2', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_250883332_s3', edge='40513089#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_250883332_s4', edge='296422467#0', lane=0, pos=65, end_pos=80, type='saturation')
        det.add_lane_area_detector(id='cluster_250883332_n1', edge='120970425#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_250883332_n2', edge='120970425#2', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_250883332_n3', edge='40513089#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_250883332_n4', edge='296422467#0', lane=0, pos=65, end_pos=-1, type='numerical')
        # GS_cluster_251997367_251997369_6525168932_6525168938_8666071616
        det.add_lane_area_detector(id='GS_cluster_251997367_b1', edge='880099405#3', lane=0, pos=255, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_251997367_b2', edge='180913157#0', lane=0, pos=18, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_251997367_b3', edge='180913157#0', lane=1, pos=18, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_251997367_b4', edge='717546622#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_251997367_b5', edge='717546622#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_251997367_b6', edge='935166811#0', lane=0, pos=71, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_251997367_b7', edge='935166811#0', lane=1, pos=71, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_251997367_s1', edge='880099405#3', lane=0, pos=75, end_pos=95, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_251997367_s2', edge='180913157#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_251997367_s3', edge='180913157#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_251997367_s4', edge='303247929#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_251997367_n1', edge='880099405#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_251997367_n2', edge='180913157#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_251997367_n3', edge='180913157#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_251997367_n4', edge='717546622#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_251997367_n5', edge='717546622#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_251997367_n6', edge='935166811#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_251997367_n7', edge='935166811#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_251997367_n8', edge='303247929#0', lane=0, end_pos=-1, type='numerical')
        # joinedS_18
        det.add_lane_area_detector(id='joinedS_18_b1', edge='481464449#2', lane=0, pos=4, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_18_b2', edge='680921676', lane=0, pos=67, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_18_b3', edge='680921676', lane=1, pos=67, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='joinedS_18_s1', edge='177433775', lane=0, pos=75, end_pos=95, type='saturation')
        det.add_lane_area_detector(id='joinedS_18_s2', edge='680921676', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_18_s3', edge='680921676', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='joinedS_18_n1', edge='481464449#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_18_n2', edge='481464449#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_18_n3', edge='177433775', lane=0, pos=75, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_18_n4', edge='680921676', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='joinedS_18_n5', edge='680921676', lane=1, end_pos=-1, type='numerical')
        # cluster_294326287_531028789_5343505332
        det.add_lane_area_detector(id='cluster_294326287_b1', edge='143367790#0', lane=0, pos=113, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_294326287_b2', edge='-143367789#2', lane=0, pos=48, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_294326287_b3', edge='561197306', lane=0, pos=56, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_294326287_b4', edge='-26826792#1', lane=0, pos=57, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_294326287_s1', edge='143367790#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_294326287_s2', edge='-143367789#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_294326287_s3', edge='561197306', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_294326287_s4', edge='-26826792#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_294326287_n1', edge='143367790#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_294326287_n2', edge='-143367789#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_294326287_n3', edge='561197306', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_294326287_n4', edge='-26826792#1', lane=0, end_pos=-1, type='numerical')
        # GS_cluster_408501435_6315125559
        det.add_lane_area_detector(id='GS_cluster_408501435_b1', edge='1022057099#0', lane=0, pos=33, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_408501435_b2', edge='1022057099#0', lane=1, pos=33, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_408501435_b3', edge='-41435572#2', lane=0, pos=27, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_408501435_b4', edge='165600530#0', lane=0, pos=62, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_408501435_b5', edge='165600530#0', lane=1, pos=62, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_408501435_b6', edge='393594509#0', lane=0, pos=50, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_408501435_s1', edge='40203863#0', lane=0, pos=30, end_pos=50, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_408501435_s2', edge='-41435572#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_408501435_s3', edge='165600530#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_408501435_s4', edge='165600530#0', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_408501435_s5', edge='393594509#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_408501435_n1', edge='1022057099#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_408501435_n2', edge='1022057099#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_408501435_n3', edge='40203863#0', lane=0, pos=30, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_408501435_n4', edge='-41435572#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_408501435_n5', edge='165600530#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_408501435_n6', edge='165600530#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_408501435_n7', edge='393594509#0', lane=0, end_pos=-1, type='numerical')
        # cluster_485133368_485133379
        det.add_lane_area_detector(id='cluster_485133368_b1', edge='40236821#4', lane=0, pos=29, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_485133368_b2', edge='86444416#1', lane=0, pos=15, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_485133368_b3', edge='180790918', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_485133368_b4', edge='41435574#0', lane=0, pos=4, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_485133368_s1', edge='40236821#4', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_485133368_s2', edge='86444416#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_485133368_s3', edge='180790918', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='cluster_485133368_s4', edge='41435574#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_485133368_n1', edge='40236821#4', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_485133368_n2', edge='86444416#1', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_485133368_n3', edge='180790918', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_485133368_n4', edge='41435574#0', lane=0, end_pos=-1, type='numerical')
        # GS_cluster_488539561_8567356615
        det.add_lane_area_detector(id='GS_cluster_488539561_b1', edge='-147396980#11', lane=0, pos=144, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_488539561_b2', edge='40401877#0', lane=0, pos=165, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_488539561_b3', edge='147396980#2', lane=0, pos=102, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_488539561_s1', edge='-147396980#11', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_488539561_s2', edge='40401877#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_488539561_s3', edge='147396980#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_488539561_n1', edge='-147396980#11', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_488539561_n2', edge='40401877#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_488539561_n3', edge='147396980#2', lane=0, end_pos=-1, type='numerical')
        # cluster_530710726_683650128
        det.add_lane_area_detector(id='cluster_530710726_b1', edge='524492805#0', lane=0, pos=50, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_530710726_b2', edge='94527184#0', lane=0, pos=66, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_530710726_b3', edge='87607993#5', lane=0, pos=16, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_530710726_b4', edge='-933868039#1', lane=0, pos=126, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_530710726_s1', edge='524492805#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_530710726_s2', edge='94527184#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_530710726_s3', edge='87607993#5', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_530710726_s4', edge='-933868039#1', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_530710726_n1', edge='524492805#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_530710726_n2', edge='94527184#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_530710726_n3', edge='87607993#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_530710726_n4', edge='-933868039#1', lane=0, end_pos=-1, type='numerical')
        # GS_cluster_6286429546_6286429552
        det.add_lane_area_detector(id='GS_cluster_6286429546_b1', edge='-71356645#2', lane=0, pos=67, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_6286429546_b2', edge='629444876#0', lane=0, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_6286429546_b3', edge='629444876#0', lane=1, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_6286429546_b4', edge='98669315#0', lane=0, pos=43, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='GS_cluster_6286429546_s1', edge='-71356645#2', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_6286429546_s2', edge='677485521', lane=0, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_6286429546_s3', edge='677485521', lane=1, end_pos=-1, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_6286429546_s4', edge='524345398#13', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='GS_cluster_6286429546_n1', edge='-71356645#2', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_6286429546_n2', edge='629444876#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_6286429546_n3', edge='629444876#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_6286429546_n4', edge='677485521', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_6286429546_n5', edge='677485521', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_6286429546_n6', edge='98669315#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='GS_cluster_6286429546_n7', edge='524345398#13', lane=0, end_pos=-1, type='numerical')
        # cluster_6286429557_6990557731
        det.add_lane_area_detector(id='cluster_6286429557_b1', edge='71356645#0', lane=0, pos=67, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_6286429557_b2', edge='934976694#0', lane=0, pos=22, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_6286429557_b3', edge='934976694#0', lane=1, pos=22, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='cluster_6286429557_s1', edge='71356645#0', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='cluster_6286429557_s2', edge='-86986701#7', lane=0, pos=260, end_pos=280, type='saturation')
        det.add_lane_area_detector(id='cluster_6286429557_n1', edge='71356645#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_6286429557_n2', edge='747087990', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_6286429557_n3', edge='86444412#3', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_6286429557_n4', edge='-86986701#7', lane=0, pos=260, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_6286429557_n5', edge='934976694#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='cluster_6286429557_n6', edge='934976694#0', lane=1, end_pos=-1, type='numerical')
        # TLS J02
        det.add_lane_area_detector(id='J02_b1', edge='1149903352#0', lane=0, pos=78, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_b2', edge='1149903352#0', lane=1, pos=78, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_b3', edge='897625775', lane=1, pos=54, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_b4', edge='897625775', lane=2, pos=54, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_b5', edge='897625775', lane=3, pos=54, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_b6', edge='897625775', lane=4, pos=54, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_b7', edge='934860520#0', lane=0, pos=52, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_b8', edge='934860520#0', lane=1, pos=52, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_b9', edge='934860520#0', lane=2, pos=52, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_b10', edge='276335209#5', lane=0, pos=94, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_b11', edge='276335209#5', lane=1, pos=94, end_pos=-1, type='boolean')
        det.add_lane_area_detector(id='J02_s1', edge='69682368', lane=0, pos=38, end_pos=58, type='saturation')
        det.add_lane_area_detector(id='J02_s2', edge='69682368', lane=1, pos=38, end_pos=58, type='saturation')
        det.add_lane_area_detector(id='J02_s3', edge='897625775', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J02_s4', edge='897625775', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J02_s5', edge='897625775', lane=3, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J02_s6', edge='897625775', lane=4, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J02_s7', edge='1149903351', lane=0, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J02_s8', edge='1149903351', lane=1, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J02_s9', edge='1149903351', lane=2, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J02_s10', edge='1149903351', lane=3, pos=0, end_pos=20, type='saturation')
        det.add_lane_area_detector(id='J02_s11', edge='276335209#2', lane=0, pos=155, end_pos=175, type='saturation')
        det.add_lane_area_detector(id='J02_s12', edge='276335209#2', lane=1, pos=155, end_pos=175, type='saturation')
        det.add_lane_area_detector(id='J02_n1', edge='1149903352#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n2', edge='1149903352#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n3', edge='69682368', lane=0, pos=38, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n4', edge='69682368', lane=1, pos=38, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n5', edge='897625775', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n6', edge='897625775', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n7', edge='897625775', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n8', edge='897625775', lane=4, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n9', edge='934860520#0', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n10', edge='934860520#0', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n11', edge='934860520#0', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n12', edge='1149903351', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n13', edge='1149903351', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n14', edge='1149903351', lane=2, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n15', edge='1149903351', lane=3, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n16', edge='276335209#5', lane=0, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n17', edge='276335209#5', lane=1, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n18', edge='276335209#2', lane=0, pos=155, end_pos=-1, type='numerical')
        det.add_lane_area_detector(id='J02_n19', edge='276335209#2', lane=1, pos=155, end_pos=-1, type='numerical')



        det.build({'detectors': os.path.join(self.THIS_FILE_PATH,'lille/lille_detectors.add.xml')})
        return det

    DETECTORS_JoinedS_1 = {
        0: {
            'boolean': ['joinedS_1_b3', 'joinedS_1_b4', 'joinedS_1_b5', 'joinedS_1_b6'],
            'saturation': ['joinedS_1_s3', 'joinedS_1_s4'],
            'numerical': ['joinedS_1_n3', 'joinedS_1_n4', 'joinedS_1_n5', 'joinedS_1_n6', 'joinedS_1_n9', 'joinedS_1_n10'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_1_b1', 'joinedS_1_b2'],
            'saturation': ['joinedS_1_s1', 'joinedS_1_s2'],
            'numerical': ['joinedS_1_n1', 'joinedS_1_n2', 'joinedS_1_n7', 'joinedS_1_n8'],
            'exit': []
        }
    }

    DETECTORS_6301940736 = {
        0: {
            'boolean': ['6301940736_b1', '6301940736_b2', '6301940736_b3', '6301940736_b4', '6301940736_b6', '6301940736_b7'],
            'saturation': ['6301940736_s1', '6301940736_s2'],
            'numerical': ['6301940736_n1', '6301940736_n2', '6301940736_n3', '6301940736_n4', '6301940736_n7', '6301940736_n8'],
            'exit': []
        },
        2: {
            'boolean': ['6301940736_b5', '6301940736_b8', '6301940736_b9', '6301940736_b10'],
            'saturation': ['6301940736_s3', '6301940736_s4'],
            'numerical': ['6301940736_n5', '6301940736_n6', '6301940736_n9', '6301940736_n10', '6301940736_n11'],
            'exit': []
        },
        4: {
            'boolean': ['6301940736_b3', '6301940736_b4', '6301940736_b6', '6301940736_b7', '6301940736_b9', '6301940736_b10'],
            'saturation': [],
            'numerical': ['6301940736_n3', '6301940736_n4', '6301940736_n7', '6301940736_n8', '6301940736_n10', '6301940736_n11'],
            'exit': []
        }
    }

    DETECTORS_cluster1681715927_250883340_274897939_6301940714 = {
        0: {
            'boolean': ['cluster6301940714_b1', 'cluster6301940714_b2', 'cluster6301940714_b3', 'cluster6301940714_b5', '6301940736_b7'],
            'saturation': ['cluster6301940714_s1', 'cluster6301940714_s2', 'cluster6301940714_s3', 'cluster6301940714_s5'],
            'numerical': ['cluster6301940714_n1', 'cluster6301940714_n2', 'cluster6301940714_n3', 'cluster6301940714_n4', 'cluster6301940714_n5', 'cluster6301940714_n6', 'cluster6301940714_n8', 'cluster6301940714_n9'],
            'exit': []
        },
        4: {
            'boolean': ['cluster6301940714_b4', 'cluster6301940714_b6'],
            'saturation': ['cluster6301940714_s4', 'cluster6301940714_s6'],
            'numerical': ['cluster6301940714_n7', 'cluster6301940714_n10'],
            'exit': []
        }
    }

    DETECTORS_6316129114 = {
        0: {
            'boolean': ['6316129114_b1', '6316129114_b2', '6316129114_b3', '6316129114_b4'],
            'saturation': ['6316129114_s1', '6316129114_s2'],
            'numerical': ['6316129114_n1', '6316129114_n2', '6316129114_n3', '6316129114_n4'],
            'exit': []
        },
        2: {
            'boolean': ['6316129114_b13', '6316129114_b14'],
            'saturation': ['6316129114_s9', '6316129114_s10'],
            'numerical': ['6316129114_n17', '6316129114_n18', '6316129114_n19', '6316129114_n20'],
            'exit': []
        },
        4: {
            'boolean': ['6316129114_b11', '6316129114_b12'],
            'saturation': ['6316129114_s7', '6316129114_s8'],
            'numerical': ['6316129114_n15', '6316129114_n16'],
            'exit': []
        },
        6: {
            'boolean': ['6316129114_b9', '6316129114_b10'],
            'saturation': ['6316129114_s5', '6316129114_s6'],
            'numerical': ['6316129114_n13', '6316129114_n14'],
            'exit': []
        },
        8: {
            'boolean': ['6316129114_b5', '6316129114_b6', '6316129114_b7', '6316129114_b8'],
            'saturation': ['6316129114_s3', '6316129114_s4'],
            'numerical': ['6316129114_n5', '6316129114_n6', '6316129114_n7', '6316129114_n8', '6316129114_n9', '6316129114_n10', '6316129114_n11', '6316129114_n12'],
            'exit': []
        }
    }

    DETECTORS_GS_cluster_1615590751_3305892794 = {
        0: {
            'boolean': ['cluster_1615590751_b1', 'cluster_1615590751_b3', 'cluster_1615590751_b4'],
            'saturation': ['cluster_1615590751_s1', 'cluster_1615590751_s3', 'cluster_1615590751_s4'],
            'numerical': ['cluster_1615590751_n1', 'cluster_1615590751_n5', 'cluster_1615590751_n6'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_1615590751_b2', 'cluster_1615590751_b5'],
            'saturation': ['cluster_1615590751_s2', 'cluster_1615590751_s5'],
            'numerical': ['cluster_1615590751_n2', 'cluster_1615590751_n3', 'cluster_1615590751_n4', 'cluster_1615590751_n7', 'cluster_1615590751_n8'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1544209593_1615582086 = {
        0: {
            'boolean': ['cluster_1544209593_b1', 'cluster_1544209593_b2', 'cluster_1544209593_b4', 'cluster_1544209593_b5'],
            'saturation': ['cluster_1544209593_s1', 'cluster_1544209593_s2', 'cluster_1544209593_s4', 'cluster_1544209593_s5'],
            'numerical': ['cluster_1544209593_n1', 'cluster_1544209593_n2', 'cluster_1544209593_n4', 'cluster_1544209593_n5'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_1544209593_b3', 'cluster_1544209593_b6'],
            'saturation': ['cluster_1544209593_s3', 'cluster_1544209593_s6'],
            'numerical': ['cluster_1544209593_n3', 'cluster_1544209593_n6'],
            'exit': []
        },
    }

    DETECTORS_joinedS_13 = {
        0: {
            'boolean': ['joinedS_13_b1', 'joinedS_13_b2', 'joinedS_13_b3', 'joinedS_13_b4', 'joinedS_13_b5', 'joinedS_13_b6', 'joinedS_13_b8', 'joinedS_13_b9', 'joinedS_13_b10', 'joinedS_13_b11'],
            'saturation': ['joinedS_13_s1', 'joinedS_13_s2', 'joinedS_13_s4', 'joinedS_13_s5'],
            'numerical': ['joinedS_13_n1', 'joinedS_13_n2', 'joinedS_13_n3', 'joinedS_13_n4', 'joinedS_13_n5', 'joinedS_13_n6', 'joinedS_13_n8', 'joinedS_13_n9', 'joinedS_13_n10', 'joinedS_13_n11', 'joinedS_13_n12', 'joinedS_13_n13', 'joinedS_13_n14', 'joinedS_13_n15'],
            'exit': []
        },
        4: {
            'boolean': ['joinedS_13_b7', 'joinedS_13_b12', 'joinedS_13_b13'],
            'saturation': ['joinedS_13_s3', 'joinedS_13_s6', 'joinedS_13_s7'],
            'numerical': ['joinedS_13_n7', 'joinedS_13_n16', 'joinedS_13_n17', 'joinedS_13_n18', 'joinedS_13_n19'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1773321434_198873202 = {
        0: {
            'boolean': ['cluster_1773321434_b1', 'cluster_1773321434_b2', 'cluster_1773321434_b4', 'cluster_1773321434_b5'],
            'saturation': ['cluster_1773321434_s1', 'cluster_1773321434_s2', 'cluster_1773321434_s3', 'cluster_1773321434_s4'],
            'numerical': ['cluster_1773321434_n1', 'cluster_1773321434_n2', 'cluster_1773321434_n3', 'cluster_1773321434_n4', 'cluster_1773321434_n5', 'cluster_1773321434_n6', 'cluster_1773321434_n7', 'cluster_1773321434_n8', 'cluster_1773321434_n10', 'cluster_1773321434_n11'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_1773321434_b3'],
            'saturation': ['cluster_1773321434_s3'],
            'numerical': ['cluster_1773321434_n9'],
            'exit': []
        },
    }

    DETECTORS_cluster1589650044_cluster1589650045_1793003425 = {
        0: {
            'boolean': ['cluster1589650044_b1', 'cluster1589650044_b2', 'cluster1589650044_b5', 'cluster1589650044_b6'],
            'saturation': ['cluster1589650044_s1', 'cluster1589650044_s2', 'cluster1589650044_s6', 'cluster1589650044_s7'],
            'numerical': ['cluster1589650044_n1', 'cluster1589650044_n2', 'cluster1589650044_n3', 'cluster1589650044_n4', 'cluster1589650044_n9', 'cluster1589650044_n10'],
            'exit': []
        },
        2: {
            'boolean': ['cluster1589650044_b3', 'cluster1589650044_b4'],
            'saturation': ['cluster1589650044_s3', 'cluster1589650044_s4', 'cluster1589650044_s5'],
            'numerical': ['cluster1589650044_n5', 'cluster1589650044_n6', 'cluster1589650044_n7', 'cluster1589650044_n8'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_506774252_8568267669_8568267670_8568267671 = {
        0: {
            'boolean': ['cluster_506774252_b1', 'cluster_506774252_b2', 'cluster_506774252_b3', 'cluster_506774252_b5', 'cluster_506774252_b6'],
            'saturation': ['cluster_506774252_s1', 'cluster_506774252_s2', 'cluster_506774252_s4', 'cluster_506774252_s5'],
            'numerical': ['cluster_506774252_n1', 'cluster_506774252_n2', 'cluster_506774252_n3', 'cluster_506774252_n4', 'cluster_506774252_n5', 'cluster_506774252_n10', 'cluster_506774252_n11'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_506774252_b4', 'cluster_506774252_b7'],
            'saturation': ['cluster_506774252_s3', 'cluster_506774252_s6'],
            'numerical': ['cluster_506774252_n6', 'cluster_506774252_n7', 'cluster_506774252_n8', 'cluster_506774252_n9', 'cluster_506774252_n12'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_506774249_5371110830 = {
        0: {
            'boolean': ['cluster_506774249_b1', 'cluster_506774249_b2', 'cluster_506774249_b4', 'cluster_506774249_b5'],
            'saturation': ['cluster_506774249_s1', 'cluster_506774249_s2', 'cluster_506774249_s4', 'cluster_506774249_s5'],
            'numerical': ['cluster_506774249_n1', 'cluster_506774249_n2', 'cluster_506774249_n5', 'cluster_506774249_n6'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_506774249_b3', 'cluster_506774249_b6'],
            'saturation': ['cluster_506774249_s3', 'cluster_506774249_s6'],
            'numerical': ['cluster_506774249_n3', 'cluster_506774249_n4', 'cluster_506774249_n7'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1589656419_1770465596 = {
        0: {
            'boolean': ['cluster_1589656419_b1', 'cluster_1589656419_b2', 'cluster_1589656419_b3', 'cluster_1589656419_b4'],
            'saturation': ['cluster_1589656419_s1', 'cluster_1589656419_s2', 'cluster_1589656419_s3', 'cluster_1589656419_s4'],
            'numerical': ['cluster_1589656419_n1', 'cluster_1589656419_n2', 'cluster_1589656419_n3', 'cluster_1589656419_n4'],
            'exit': []
        },
        4: {
            'boolean': ['cluster_1589656419_b5'],
            'saturation': ['cluster_1589656419_s5'],
            'numerical': ['cluster_1589656419_n5'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1589659457_1912530468 = {
        0: {
            'boolean': ['cluster_1589659457_b1', 'cluster_1589659457_b2', 'cluster_1589659457_b4', 'cluster_1589659457_b5'],
            'saturation': ['cluster_1589659457_s1', 'cluster_1589659457_s2', 'cluster_1589659457_s4', 'cluster_1589659457_s5'],
            'numerical': ['cluster_1589659457_n1', 'cluster_1589659457_n2', 'cluster_1589659457_n6', 'cluster_1589659457_n7'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_1589659457_b3', 'cluster_1589659457_b6'],
            'saturation': ['cluster_1589659457_s3', 'cluster_1589659457_s6'],
            'numerical': ['cluster_1589659457_n3', 'cluster_1589659457_n4', 'cluster_1589659457_n5', 'cluster_1589659457_n8', 'cluster_1589659457_n9', 'cluster_1589659457_n10', 'cluster_1589659457_n11'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1770465595_4365093914 = {
        0: {
            'boolean': ['cluster_1770465595_b1', 'cluster_1770465595_b2', 'cluster_1770465595_b4', 'cluster_1770465595_b5'],
            'saturation': ['cluster_1770465595_s1', 'cluster_1770465595_s2', 'cluster_1770465595_s4', 'cluster_1770465595_s5'],
            'numerical': ['cluster_1770465595_n1', 'cluster_1770465595_n2', 'cluster_1770465595_n6', 'cluster_1770465595_n7', 'cluster_1770465595_n8', 'cluster_1770465595_n9', 'cluster_1770465595_n10', 'cluster_1770465595_n11'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_1770465595_b3'],
            'saturation': ['cluster_1770465595_s3'],
            'numerical': ['cluster_1770465595_n3', 'cluster_1770465595_n4', 'cluster_1770465595_n5'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_149374929_1770465614_305929185 = {
        0: {
            'boolean': ['cluster_149374929_b4', 'cluster_149374929_b5', 'cluster_149374929_b6', 'cluster_149374929_b7'],
            'saturation': ['cluster_149374929_s3', 'cluster_149374929_s4', 'cluster_149374929_s5', 'cluster_149374929_s6'],
            'numerical': ['cluster_149374929_n8', 'cluster_149374929_n9', 'cluster_149374929_n10', 'cluster_149374929_n11', 'cluster_149374929_n12', 'cluster_149374929_n13'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_149374929_b1', 'cluster_149374929_b2', 'cluster_149374929_b3', 'cluster_149374929_b4', 'cluster_149374929_b5'],
            'saturation': ['cluster_149374929_s1', 'cluster_149374929_s2', 'cluster_149374929_s3', 'cluster_149374929_s4'],
            'numerical': ['cluster_149374929_n1', 'cluster_149374929_n2', 'cluster_149374929_n3', 'cluster_149374929_n4', 'cluster_149374929_n5', 'cluster_149374929_n6', 'cluster_149374929_n7', 'cluster_149374929_n8', 'cluster_149374929_n9'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1845506036_305930099 = {
        0: {
            'boolean': ['cluster_1845506036_b3', 'cluster_1845506036_b4', 'cluster_1845506036_b7', 'cluster_1845506036_b8'],
            'saturation': ['cluster_1845506036_s3', 'cluster_1845506036_s4', 'cluster_1845506036_s7', 'cluster_1845506036_s8'],
            'numerical': ['cluster_1845506036_n3', 'cluster_1845506036_n4', 'cluster_1845506036_n11', 'cluster_1845506036_n12', 'cluster_1845506036_n13', 'cluster_1845506036_n14'],
            'exit': []
        },
        4: {
            'boolean': ['cluster_1845506036_b1', 'cluster_1845506036_b2', 'cluster_1845506036_b5', 'cluster_1845506036_b6'],
            'saturation': ['cluster_1845506036_s1', 'cluster_1845506036_s2', 'cluster_1845506036_s5', 'cluster_1845506036_s6'],
            'numerical': ['cluster_1845506036_n1', 'cluster_1845506036_n2', 'cluster_1845506036_n5', 'cluster_1845506036_n6', 'cluster_1845506036_n7', 'cluster_1845506036_n8', 'cluster_1845506036_n9', 'cluster_1845506036_n10'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_507288390_508056193 = {
        0: {
            'boolean': ['cluster_507288390_b1', 'cluster_507288390_b2', 'cluster_507288390_b4', 'cluster_507288390_b5'],
            'saturation': ['cluster_507288390_s1', 'cluster_507288390_s2', 'cluster_507288390_s4', 'cluster_507288390_s5'],
            'numerical': ['cluster_507288390_n1', 'cluster_507288390_n2', 'cluster_507288390_n3', 'cluster_507288390_n4', 'cluster_507288390_n6', 'cluster_507288390_n7'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_507288390_b3'],
            'saturation': ['cluster_507288390_s3'],
            'numerical': ['cluster_507288390_n5'],
            'exit': []
        },
    }

    DETECTORS_cluster_485133449_485133452 = {
        0: {
            'boolean': ['cluster_485133449_b2', 'cluster_485133449_b3', 'cluster_485133449_b4', 'cluster_485133449_b5'],
            'saturation': ['cluster_485133449_s2', 'cluster_485133449_s3', 'cluster_485133449_s4', 'cluster_485133449_s5'],
            'numerical': ['cluster_485133449_n2', 'cluster_485133449_n3', 'cluster_485133449_n4', 'cluster_485133449_n5'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_485133449_b1'],
            'saturation': ['cluster_485133449_s1'],
            'numerical': ['cluster_485133449_n1'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_485133387_485133391 = {
        0: {
            'boolean': ['cluster_485133387_b1', 'cluster_485133387_b2', 'cluster_485133387_b4', 'cluster_485133387_b5'],
            'saturation': ['cluster_485133387_s1', 'cluster_485133387_s2', 'cluster_485133387_s4', 'cluster_485133387_s5'],
            'numerical': ['cluster_485133387_n1', 'cluster_485133387_n2', 'cluster_485133387_n5', 'cluster_485133387_n6'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_485133387_b3'],
            'saturation': ['cluster_485133387_s3'],
            'numerical': ['cluster_485133387_n3', 'cluster_485133387_n4'],
            'exit': []
        },
    }

    DETECTORS_cluster1638086122_cluster_251058778_32828632 = {
        0: {
            'boolean': ['cluster1638086122_b5', 'cluster1638086122_b6', 'cluster1638086122_b7', 'cluster1638086122_b8', 'cluster1638086122_b9', 'cluster1638086122_b10'],
            'saturation': ['cluster1638086122_s2', 'cluster1638086122_s3', 'cluster1638086122_s4', 'cluster1638086122_s5', 'cluster1638086122_s6', 'cluster1638086122_s7'],
            'numerical': ['cluster1638086122_n5', 'cluster1638086122_n6', 'cluster1638086122_n7', 'cluster1638086122_n8', 'cluster1638086122_n9', 'cluster1638086122_n10'],
            'exit': []
        },
        4: {
            'boolean': ['cluster1638086122_b1', 'cluster1638086122_b2', 'cluster1638086122_b3', 'cluster1638086122_b4'],
            'saturation': ['cluster1638086122_s1'],
            'numerical': ['cluster1638086122_n1', 'cluster1638086122_n2', 'cluster1638086122_n3', 'cluster1638086122_n4'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1635760230_1635775294_3706468062 = {
        0: {
            'boolean': ['cluster_1635760230_b1', 'cluster_1635760230_b2', 'cluster_1635760230_b5', 'cluster_1635760230_b6'],
            'saturation': ['cluster_1635760230_s1', 'cluster_1635760230_s2', 'cluster_1635760230_s5'],
            'numerical': ['cluster_1635760230_n1', 'cluster_1635760230_n2', 'cluster_1635760230_n3', 'cluster_1635760230_n4', 'cluster_1635760230_n5', 'cluster_1635760230_n6', 'cluster_1635760230_n11', 'cluster_1635760230_n12', 'cluster_1635760230_n13', 'cluster_1635760230_n14'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_1635760230_b3', 'cluster_1635760230_b4', 'cluster_1635760230_b7'],
            'saturation': ['cluster_1635760230_s3', 'cluster_1635760230_s4', 'cluster_1635760230_s6'],
            'numerical': ['cluster_1635760230_n7', 'cluster_1635760230_n8', 'cluster_1635760230_n9', 'cluster_1635760230_n10', 'cluster_1635760230_n15', 'cluster_1635760230_n16'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1635701799_1635701803 = {
        0: {
            'boolean': ['cluster_1635701799_b1', 'cluster_1635701799_b2', 'cluster_1635701799_b4', 'cluster_1635701799_b5'],
            'saturation': ['cluster_1635701799_s1', 'cluster_1635701799_s2', 'cluster_1635701799_s4', 'cluster_1635701799_s5'],
            'numerical': ['cluster_1635701799_n1', 'cluster_1635701799_n2', 'cluster_1635701799_n3', 'cluster_1635701799_n4', 'cluster_1635701799_n5', 'cluster_1635701799_n6', 'cluster_1635701799_n7', 'cluster_1635701799_n8', 'cluster_1635701799_n12', 'cluster_1635701799_n13', 'cluster_1635701799_n14', 'cluster_1635701799_n15', 'cluster_1635701799_n16', 'cluster_1635701799_n17', ],
            'exit': []
        },
        2: {
            'boolean': ['cluster_1635701799_b3', 'cluster_1635701799_b6'],
            'saturation': ['cluster_1635701799_s3', 'cluster_1635701799_s6'],
            'numerical': ['cluster_1635701799_n9', 'cluster_1635701799_n10', 'cluster_1635701799_n11', 'cluster_1635701799_n18', 'cluster_1635701799_n19'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1635660842_1635660844 = {
        0: {
            'boolean': ['cluster_1635660842_b1', 'cluster_1635660842_b2', 'cluster_1635660842_b4', 'cluster_1635660842_b5'],
            'saturation': ['cluster_1635660842_s1', 'cluster_1635660842_s2', 'cluster_1635660842_s4', 'cluster_1635660842_s5'],
            'numerical': ['cluster_1635660842_n1', 'cluster_1635660842_n2', 'cluster_1635660842_n3', 'cluster_1635660842_n4', 'cluster_1635660842_n5', 'cluster_1635660842_n6', 'cluster_1635660842_n10', 'cluster_1635660842_n11', 'cluster_1635660842_n12', 'cluster_1635660842_n13', 'cluster_1635660842_n14', 'cluster_1635660842_n15', 'cluster_1635660842_n16', 'cluster_1635660842_n17'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_1635660842_b3'],
            'saturation': ['cluster_1635660842_s3'],
            'numerical': ['cluster_1635660842_n7', 'cluster_1635660842_n8', 'cluster_1635660842_n9'],
            'exit': []
        }
    }

    DETECTORS_6267747172 = {
        0: {
            'boolean': ['6267747172_b1'],
            'saturation': ['6267747172_s1'],
            'numerical': ['6267747172_n1', '6267747172_n2'],
            'exit': []
        },
        2: {
            'boolean': ['6267747172_b10', '6267747172_b11'],
            'saturation': ['6267747172_s9', '6267747172_s10'],
            'numerical': ['6267747172_n19', '6267747172_n20'],
            'exit': []
        },
        4: {
            'boolean': ['6267747172_b9'],
            'saturation': ['6267747172_s8'],
            'numerical': ['6267747172_n17', '6267747172_n18'],
            'exit': []
        },
        6: {
            'boolean': ['6267747172_b8'],
            'saturation': ['6267747172_s7'],
            'numerical': ['6267747172_n15', '6267747172_n16'],
            'exit': []
        },
        8: {
            'boolean': ['6267747172_b6', '6267747172_b7'],
            'saturation': ['6267747172_s5', '6267747172_s6'],
            'numerical': ['6267747172_n11', '6267747172_n12', '6267747172_n13', '6267747172_n14'],
            'exit': []
        },
        10: {
            'boolean': ['6267747172_b5'],
            'saturation': ['6267747172_s4'],
            'numerical': ['6267747172_n8', '6267747172_n9', '6267747172_n10'],
            'exit': []
        },
        12: {
            'boolean': ['6267747172_b2', '6267747172_b3', '6267747172_b4'],
            'saturation': ['6267747172_s2', '6267747172_s3'],
            'numerical': ['6267747172_n3', '6267747172_n4', '6267747172_n5', '6267747172_n6', '6267747172_n7'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1650597986_1650597989_1652594655 = {
        0: {
            'boolean': ['cluster_1650597986_b1', 'cluster_1650597986_b2', 'cluster_1650597986_b4', 'cluster_1650597986_b5'],
            'saturation': ['cluster_1650597986_s1', 'cluster_1650597986_s2', 'cluster_1650597986_s4', 'cluster_1650597986_s5'],
            'numerical': ['cluster_1650597986_n1', 'cluster_1650597986_n2', 'cluster_1650597986_n5', 'cluster_1650597986_n6', 'cluster_1650597986_n7', 'cluster_1650597986_n8'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_1650597986_b3'],
            'saturation': ['cluster_1650597986_s3'],
            'numerical': ['cluster_1650597986_n3', 'cluster_1650597986_n4', 'cluster_1650597986_n5'],
            'exit': []
        }
    }

    DETECTORS_GS_cluster_1648564005_1648564008 = {
        0: {
            'boolean': ['cluster_1648564005_b1', 'cluster_1648564005_b2', 'cluster_1648564005_b5', 'cluster_1648564005_b6'],
            'saturation': ['cluster_1648564005_s1', 'cluster_1648564005_s2', 'cluster_1648564005_s4', 'cluster_1648564005_s5'],
            'numerical': ['cluster_1648564005_n1', 'cluster_1648564005_n2', 'cluster_1648564005_n3', 'cluster_1648564005_n4', 'cluster_1648564005_n7', 'cluster_1648564005_n8'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_1648564005_b3', 'cluster_1648564005_b4', 'cluster_1648564005_b7'],
            'saturation': ['cluster_1648564005_s3', 'cluster_1648564005_s6'],
            'numerical': ['cluster_1648564005_n5', 'cluster_1648564005_n6', 'cluster_1648564005_n9', 'cluster_1648564005_n10', 'cluster_1648564005_n11', 'cluster_1648564005_n12'],
            'exit': []
        }
    }

    DETECTORS_cluster1647146953_1647146957 = {
        0: {
            'boolean': ['cluster1647146953_b1', 'cluster1647146953_b2', 'cluster1647146953_b5', 'cluster1647146953_b6'],
            'saturation': ['cluster1647146953_s1', 'cluster1647146953_s2', 'cluster1647146953_s4', 'cluster1647146953_s5'],
            'numerical': ['cluster1647146953_n1', 'cluster1647146953_n2', 'cluster1647146953_n3', 'cluster1647146953_n4', 'cluster1647146953_n7', 'cluster1647146953_n8', 'cluster1647146953_n9', 'cluster1647146953_n10'],
            'exit': []
        },
        4: {
            'boolean': ['cluster1647146953_b3', 'cluster1647146953_b4', 'cluster1647146953_b7'],
            'saturation': ['cluster1647146953_s3', 'cluster1647146953_s6'],
            'numerical': ['cluster1647146953_n5', 'cluster1647146953_n6', 'cluster1647146953_n11', 'cluster1647146953_n12'],
            'exit': []
        }
    }

    DETECTORS_joinedS_4 = {
        0: {
            'boolean': ['joinedS_4_b1', 'joinedS_4_b2', 'joinedS_4_b4', 'joinedS_4_b5'],
            'saturation': ['joinedS_4_s1', 'joinedS_4_s2', 'joinedS_4_s4', 'joinedS_4_s5'],
            'numerical': ['joinedS_4_n1', 'joinedS_4_n2', 'joinedS_4_n5', 'joinedS_4_n6', 'joinedS_4_n7', 'joinedS_4_n8', 'joinedS_4_n9', 'joinedS_4_n10'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_4_b3', 'joinedS_4_b6'],
            'saturation': ['joinedS_4_s3', 'joinedS_4_s6'],
            'numerical': ['joinedS_4_n3', 'joinedS_4_n4', 'joinedS_4_n11', 'joinedS_4_n12', 'joinedS_4_n13'],
            'exit': []
        }
    }

    DETECTORS_J11 = {
        0: {
            'boolean': ['J11_b1'],
            'saturation': ['J11_s1'],
            'numerical': ['J11_n1'],
            'exit': []
        },
        2: {
            'boolean': ['J11_b2', 'J11_b3'],
            'saturation': ['J11_s2', 'J11_s3'],
            'numerical': ['J11_n2', 'J11_n3'],
            'exit': []
        }
    }

    DETECTORS_GS_1713983096 = {
        0: {
            'boolean': ['1713983096_b1', '1713983096_b2', '1713983096_b3'],
            'saturation': ['1713983096_s1', '1713983096_s2', '1713983096_s3'],
            'numerical': ['1713983096_n1', '1713983096_n2', '1713983096_n3'],
            'exit': []
        },
        4: {
            'boolean': ['1713983096_b4'],
            'saturation': ['1713983096_s4'],
            'numerical': ['1713983096_n4'],
            'exit': []
        }
    }

    DETECTORS_GS_1713983087 = {
        0: {
            'boolean': ['1713983087_b1', '1713983087_b2'],
            'saturation': ['1713983087_s1', '1713983087_s2'],
            'numerical': ['1713983087_n1', '1713983087_n2'],
            'exit': []
        },
        4: {
            'boolean': ['1713983087_b3', '1713983087_b4'],
            'saturation': ['1713983087_s3'],
            'numerical': ['1713983087_n3', '1713983087_n4', '1713983087_n5'],
            'exit': []
        }
    }

    DETECTORS_cluster1436583672_1713998722_4401347167_4401347179 = {
        0: {
            'boolean': ['cluster1436583672_b3', 'cluster1436583672_b4', 'cluster1436583672_b5', 'cluster1436583672_b8', 'cluster1436583672_b9', 'cluster1436583672_b10'],
            'saturation': ['cluster1436583672_s3', 'cluster1436583672_s4', 'cluster1436583672_s5', 'cluster1436583672_s7', 'cluster1436583672_s8'],
            'numerical': ['cluster1436583672_n5', 'cluster1436583672_n6', 'cluster1436583672_n7', 'cluster1436583672_n8', 'cluster1436583672_n9', 'cluster1436583672_n10', 'cluster1436583672_n15', 'cluster1436583672_n16', 'cluster1436583672_n17', 'cluster1436583672_n18', 'cluster1436583672_n19', 'cluster1436583672_n20', 'cluster1436583672_n21', 'cluster1436583672_n22'],
            'exit': []
        },
        4: {
            'boolean': ['cluster1436583672_b1', 'cluster1436583672_b2', 'cluster1436583672_b6', 'cluster1436583672_b7'],
            'saturation': ['cluster1436583672_s1', 'cluster1436583672_s2', 'cluster1436583672_s6'],
            'numerical': ['cluster1436583672_n1', 'cluster1436583672_n2', 'cluster1436583672_n3', 'cluster1436583672_n4', 'cluster1436583672_n11', 'cluster1436583672_n12', 'cluster1436583672_n13', 'cluster1436583672_n14'],
            'exit': []
        }
    }

    DETECTORS_GS_cluster_426634816_6316128557_6316128558_801903207_801903212 = {
        0: {
            'boolean': ['cluster_426634816_b4', 'cluster_426634816_b5', 'cluster_426634816_b6', 'cluster_426634816_b7'],
            'saturation': ['cluster_426634816_s2', 'cluster_426634816_s3', 'cluster_426634816_s4', 'cluster_426634816_s5'],
            'numerical': ['cluster_426634816_n8', 'cluster_426634816_n9', 'cluster_426634816_n10', 'cluster_426634816_n11', 'cluster_426634816_n12', 'cluster_426634816_n13', 'cluster_426634816_n14', 'cluster_426634816_n15', 'cluster_426634816_n16', 'cluster_426634816_n17', 'cluster_426634816_n18', 'cluster_426634816_n19'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_426634816_b1', 'cluster_426634816_b2', 'cluster_426634816_b3'],
            'saturation': ['cluster_426634816_s1'],
            'numerical': ['cluster_426634816_n1', 'cluster_426634816_n2', 'cluster_426634816_n3', 'cluster_426634816_n4', 'cluster_426634816_n5', 'cluster_426634816_n6', 'cluster_426634816_n7'],
            'exit': []
        }
    }

    DETECTORS_GS_cluster_243072211_250894314 = {
        0: {
            'boolean': ['cluster_243072211_b1', 'cluster_243072211_b2', 'cluster_243072211_b5', 'cluster_243072211_b6'],
            'saturation': ['cluster_243072211_s1', 'cluster_243072211_s2', 'cluster_243072211_s5', 'cluster_243072211_s6'],
            'numerical': ['cluster_243072211_n1', 'cluster_243072211_n2', 'cluster_243072211_n3', 'cluster_243072211_n4', 'cluster_243072211_n7', 'cluster_243072211_n8'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_243072211_b3', 'cluster_243072211_b4'],
            'saturation': ['cluster_243072211_s3', 'cluster_243072211_s4'],
            'numerical': ['cluster_243072211_n3', 'cluster_243072211_n4'],
            'exit': []
        }
    }

    DETECTORS_GS_cluster_1800186885_1800186886_243072210_3404227323_494986739 = {
        0: {
            'boolean': ['cluster_1800186885_b1', 'cluster_1800186885_b2', 'cluster_1800186885_b3', 'cluster_1800186885_b4', 'cluster_1800186885_b7', 'cluster_1800186885_b8', 'cluster_1800186885_b9', 'cluster_1800186885_b10'],
            'saturation': ['cluster_1800186885_s1', 'cluster_1800186885_s2', 'cluster_1800186885_s5', 'cluster_1800186885_s6'],
            'numerical': ['cluster_1800186885_n1', 'cluster_1800186885_n2', 'cluster_1800186885_n3', 'cluster_1800186885_n4', 'cluster_1800186885_n7', 'cluster_1800186885_n8', 'cluster_1800186885_n9', 'cluster_1800186885_n10'],
            'exit': []
        },
        4: {
            'boolean': ['cluster_1800186885_b5', 'cluster_1800186885_b6'],
            'saturation': ['cluster_1800186885_s3', 'cluster_1800186885_s4'],
            'numerical': ['cluster_1800186885_n3', 'cluster_1800186885_n4'],
            'exit': []
        }
    }

    DETECTORS_GS_cluster_1302128733_423805246 = {
        0: {
            'boolean': ['cluster_1302128733_b2', 'cluster_1302128733_b3', 'cluster_1302128733_b4', 'cluster_1302128733_b5'],
            'saturation': ['cluster_1302128733_s2', 'cluster_1302128733_s3', 'cluster_1302128733_s4', 'cluster_1302128733_s5'],
            'numerical': ['cluster_1302128733_n4', 'cluster_1302128733_n5', 'cluster_1302128733_n6', 'cluster_1302128733_n7'],
            'exit': []
        },
        4: {
            'boolean': ['cluster_1302128733_b1'],
            'saturation': ['cluster_1302128733_s1'],
            'numerical': ['cluster_1302128733_n1', 'cluster_1302128733_n2', 'cluster_1302128733_n3'],
            'exit': []
        }
    }

    DETECTORS_GS_cluster_267375483_3077077589 = {
        0: {
            'boolean': ['cluster_267375483_b1', 'cluster_267375483_b4', 'cluster_267375483_b5'],
            'saturation': ['cluster_267375483_s1', 'cluster_267375483_s4', 'cluster_267375483_s5'],
            'numerical': ['cluster_267375483_n1', 'cluster_267375483_n6', 'cluster_267375483_n7'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_267375483_b2', 'cluster_267375483_b3'],
            'saturation': ['cluster_267375483_s2', 'cluster_267375483_s3'],
            'numerical': ['cluster_267375483_n2', 'cluster_267375483_n3', 'cluster_267375483_n4', 'cluster_267375483_n5'],
            'exit': []
        }
    }

    DETECTORS_joinedS_12 = {
        0: {
            'boolean': ['joinedS_12_b4', 'joinedS_12_b5'],
            'saturation': ['joinedS_12_s4', 'joinedS_12_s5'],
            'numerical': ['joinedS_12_n7', 'joinedS_12_n8', 'joinedS_12_n9', 'joinedS_12_n10', 'joinedS_12_n11', 'joinedS_12_n12', 'joinedS_12_n13'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_12_b1', 'joinedS_12_b2', 'joinedS_12_b3'],
            'saturation': ['joinedS_12_s1', 'joinedS_12_s2', 'joinedS_12_s3'],
            'numerical': ['joinedS_12_n1', 'joinedS_12_n2', 'joinedS_12_n3', 'joinedS_12_n4', 'joinedS_12_n5', 'joinedS_12_n6'],
            'exit': []
        },
        4: {
            'boolean': ['joinedS_12_b9'],
            'saturation': ['joinedS_12_s9', 'joinedS_12_s10'],
            'numerical': ['joinedS_12_n20', 'joinedS_12_n21', 'joinedS_12_n22', 'joinedS_12_n23'],
            'exit': []
        },
        6: {
            'boolean': ['joinedS_12_b6', 'joinedS_12_b7', 'joinedS_12_b8'],
            'saturation': ['joinedS_12_s6', 'joinedS_12_s7', 'joinedS_12_s8'],
            'numerical': ['joinedS_12_n14', 'joinedS_12_n15', 'joinedS_12_n16', 'joinedS_12_n17', 'joinedS_12_n18', 'joinedS_12_n19'],
            'exit': []
        }
    }

    DETECTORS_652409498 = {
        0: {
            'boolean': ['652409498_b3', '652409498_b4'],
            'saturation': ['652409498_s3', '652409498_s4'],
            'numerical': ['652409498_n3', '652409498_n4'],
            'exit': []
        },
        2: {
            'boolean': ['652409498_b1', '652409498_b2'],
            'saturation': ['652409498_s1', '652409498_s2'],
            'numerical': ['652409498_n1', '652409498_n2'],
            'exit': []
        }
    }

    DETECTORS_269243965 = {
        0: {
            'boolean': ['269243965_b3', '269243965_b4'],
            'saturation': ['269243965_s3', '269243965_s4'],
            'numerical': ['269243965_n5', '269243965_n6'],
            'exit': []
        },
        2: {
            'boolean': ['269243965_b1', '269243965_b2'],
            'saturation': ['269243965_s1', '269243965_s2'],
            'numerical': ['269243965_n1', '269243965_n2', '269243965_n3', '269243965_n4'],
            'exit': []
        }
    }

    DETECTORS_joinedS_2 = {
        0: {
            'boolean': ['joinedS_2_b2', 'joinedS_2_b3', 'joinedS_2_b5', 'joinedS_2_b6', 'joinedS_2_b7'],
            'saturation': ['joinedS_2_s2', 'joinedS_2_s3', 'joinedS_2_s5', 'joinedS_2_s6', 'joinedS_2_s7'],
            'numerical': ['joinedS_2_n3', 'joinedS_2_n4', 'joinedS_2_n5', 'joinedS_2_n6', 'joinedS_2_n8', 'joinedS_2_n9', 'joinedS_2_n10', 'joinedS_2_n11'],
            'exit': []
        },
        4: {
            'boolean': ['joinedS_2_b1', 'joinedS_2_b4'],
            'saturation': ['joinedS_2_s1', 'joinedS_2_s4'],
            'numerical': ['joinedS_2_n1', 'joinedS_2_n2', 'joinedS_2_n7'],
            'exit': []
        }
    }

    DETECTORS_cluster1302567586_149374901_305918149 = {
        0: {
            'boolean': ['cluster1302567586_b2', 'cluster1302567586_b3', 'cluster1302567586_b4', 'cluster1302567586_b5'],
            'saturation': ['cluster1302567586_s2', 'cluster1302567586_s3', 'cluster1302567586_s4', 'cluster1302567586_s5'],
            'numerical': ['cluster1302567586_n3', 'cluster1302567586_n4', 'cluster1302567586_n5', 'cluster1302567586_n6'],
            'exit': []
        },
        2: {
            'boolean': ['cluster1302567586_b1'],
            'saturation': ['cluster1302567586_s1'],
            'numerical': ['cluster1302567586_n1', 'cluster1302567586_n2'],
            'exit': []
        },
        4: {
            'boolean': ['cluster1302567586_b2', 'cluster1302567586_b3', 'cluster1302567586_b6'],
            'saturation': ['cluster1302567586_s2', 'cluster1302567586_s3', 'cluster1302567586_s6'],
            'numerical': ['cluster1302567586_n3', 'cluster1302567586_n4', 'cluster1302567586_n7', 'cluster1302567586_n8'],
            'exit': []
        }
    }

    DETECTORS_cluster1302567574_149374913_305918532 = {
        0: {
            'boolean': ['cluster1302567574_b2', 'cluster1302567574_b3', 'cluster1302567574_b4', 'cluster1302567574_b5'],
            'saturation': ['cluster1302567574_s2', 'cluster1302567574_s3', 'cluster1302567574_s4', 'cluster1302567574_s6'],
            'numerical': ['cluster1302567574_n3', 'cluster1302567574_n4', 'cluster1302567574_n5', 'cluster1302567574_n6', 'cluster1302567574_n8', 'cluster1302567574_n9'],
            'exit': []
        },
        2: {
            'boolean': ['cluster1302567574_b1'],
            'saturation': ['cluster1302567574_s1'],
            'numerical': ['cluster1302567574_n1', 'cluster1302567574_n2'],
            'exit': []
        },
        4: {
            'boolean': ['cluster1302567574_b2', 'cluster1302567574_b3', 'cluster1302567574_b6'],
            'saturation': ['cluster1302567574_s2', 'cluster1302567574_s3', 'cluster1302567574_s6'],
            'numerical': ['cluster1302567574_n3', 'cluster1302567574_n4', 'cluster1302567574_n7'],
            'exit': []
        }
    }

    DETECTORS_GS_cluster_2298692584_7170315064 = {
        0: {
            'boolean': ['cluster_2298692584_b2', 'cluster_2298692584_b3', 'cluster_2298692584_b4', 'cluster_2298692584_b5'],
            'saturation': ['cluster_2298692584_s2', 'cluster_2298692584_s3', 'cluster_2298692584_s4', 'cluster_2298692584_s5'],
            'numerical': ['cluster_2298692584_n2', 'cluster_2298692584_n3', 'cluster_2298692584_n4', 'cluster_2298692584_n5', 'cluster_2298692584_n6', 'cluster_2298692584_n7'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_2298692584_b1'],
            'saturation': ['cluster_2298692584_s1'],
            'numerical': ['cluster_2298692584_n1'],
            'exit': []
        }
    }

    DETECTORS_joinedS_5 = {
        0: {
            'boolean': ['joinedS_5_b6', 'joinedS_5_b7', 'joinedS_5_b8'],
            'saturation': ['joinedS_5_s5', 'joinedS_5_s6'],
            'numerical': ['joinedS_5_n9', 'joinedS_5_n10', 'joinedS_5_n11', 'joinedS_5_n12', 'joinedS_5_n13', 'joinedS_5_n14', 'joinedS_5_n15'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_5_b9', 'joinedS_5_b10'],
            'saturation': ['joinedS_5_s7', 'joinedS_5_s8', 'joinedS_5_s9', 'joinedS_5_s10'],
            'numerical': ['joinedS_5_n16', 'joinedS_5_n17', 'joinedS_5_n18', 'joinedS_5_n19', 'joinedS_5_n20', 'joinedS_5_n21', 'joinedS_5_n22', 'joinedS_5_n23', 'joinedS_5_n24', 'joinedS_5_n25'],
            'exit': []
        },
        4: {
            'boolean': ['joinedS_5_b1', 'joinedS_5_b2'],
            'saturation': ['joinedS_5_s1', 'joinedS_5_s2'],
            'numerical': ['joinedS_5_n1', 'joinedS_5_n2'],
            'exit': []
        },
        6: {
            'boolean': ['joinedS_5_b3'],
            'saturation': ['joinedS_5_s3'],
            'numerical': ['joinedS_5_n3', 'joinedS_5_n4', 'joinedS_5_n5'],
            'exit': []
        },
        8: {
            'boolean': ['joinedS_5_b4', 'joinedS_5_b5'],
            'saturation': ['joinedS_5_s4'],
            'numerical': ['joinedS_5_n6', 'joinedS_5_n7', 'joinedS_5_n8'],
            'exit': []
        }
    }

    DETECTORS_491543745 = {
        0: {
            'boolean': ['491543745_b5', '491543745_b6', '491543745_b7', '491543745_b8', '491543745_b9', '491543745_b10', '491543745_b11', '491543745_b12', '491543745_b14', '491543745_b15', '491543745_b16', '491543745_b17', '491543745_b20', '491543745_b21', '491543745_b22', '491543745_b23'],
            'saturation': ['491543745_s4', '491543745_s5', '491543745_s6', '491543745_s7', '491543745_s9', '491543745_s10'],
            'numerical': ['491543745_n8', '491543745_n9', '491543745_n10', '491543745_n11', '491543745_n12', '491543745_n13', '491543745_n14', '491543745_n15', '491543745_n19', '491543745_n20', '491543745_n21', '491543745_n22', '491543745_n23', '491543745_n24', '491543745_n25', '491543745_n26', '491543745_n29', '491543745_n30', '491543745_n31', '491543745_n32'],
            'exit': []
        },
        2: {
            'boolean': ['491543745_b1', '491543745_b2', '491543745_b3', '491543745_b4', '491543745_b13', '491543745_b18', '491543745_b19', '491543745_b24', '491543745_b25'],
            'saturation': ['491543745_s1', '491543745_s2', '491543745_s3', '491543745_s8'],
            'numerical': ['491543745_n1', '491543745_n2', '491543745_n3', '491543745_n4', '491543745_n5', '491543745_n6', '491543745_n7', '491543745_n16', '491543745_n17', '491543745_n18', '491543745_n27', '491543745_n28', '491543745_n33', '491543745_n34'],
            'exit': []
        }
    }

    DETECTORS_GS_1299481284 = {
        0: {
            'boolean': ['GS_1299481284_b1', 'GS_1299481284_b2'],
            'saturation': ['GS_1299481284_s1', 'GS_1299481284_s2'],
            'numerical': ['GS_1299481284_n1', 'GS_1299481284_n2', 'GS_1299481284_n3', 'GS_1299481284_n4'],
            'exit': []
        },
        2: {
            'boolean': ['GS_1299481284_b3', 'GS_1299481284_b4'],
            'saturation': ['GS_1299481284_s3', 'GS_1299481284_s4'],
            'numerical': ['GS_1299481284_n5', 'GS_1299481284_n6', 'GS_1299481284_n7', 'GS_1299481284_n8', 'GS_1299481284_n9', 'GS_1299481284_n10'],
            'exit': []
        }
    }

    DETECTORS_133278923 = {
        0: {
            'boolean': ['133278923_b3', '133278923_b4'],
            'saturation': ['133278923_s3', '133278923_s4'],
            'numerical': ['133278923_n5', '133278923_n6'],
            'exit': []
        },
        2: {
            'boolean': ['133278923_b1', '133278923_b2'],
            'saturation': ['133278923_s1', '133278923_s2'],
            'numerical': ['133278923_n1', '133278923_n2', '133278923_n3', '133278923_n4'],
            'exit': []
        }
    }

    DETECTORS_joinedS_3 = {
        0: {
            'boolean': ['joinedS_3_b1', 'joinedS_3_b2', 'joinedS_3_b3', 'joinedS_3_b4', 'joinedS_3_b5', 'joinedS_3_b6'],
            'saturation': ['joinedS_3_s1', 'joinedS_3_s2'],
            'numerical': ['joinedS_3_n1', 'joinedS_3_n2', 'joinedS_3_n3', 'joinedS_3_n4', 'joinedS_3_n5', 'joinedS_3_n6', 'joinedS_3_n7', 'joinedS_3_n8'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_3_b7', 'joinedS_3_b8', 'joinedS_3_b9'],
            'saturation': ['joinedS_3_s3', 'joinedS_3_s4', 'joinedS_3_s5'],
            'numerical': ['joinedS_3_n9', 'joinedS_3_n10', 'joinedS_3_n11'],
            'exit': []
        }
    }

    DETECTORS_GS_164480279 = {
        0: {
            'boolean': ['GS_164480279_b2', 'joinedS_3_b3'],
            'saturation': ['GS_164480279_s2', 'GS_164480279_s3'],
            'numerical': ['GS_164480279_n3', 'GS_164480279_n4'],
            'exit': []
        },
        4: {
            'boolean': ['GS_164480279_b1'],
            'saturation': ['GS_164480279_s1'],
            'numerical': ['GS_164480279_n1', 'GS_164480279_n2'],
            'exit': []
        }
    }

    DETECTORS_GS_1656149839 = {
        0: {
            'boolean': ['GS_1656149839_b1', 'GS_1656149839_b2', 'GS_1656149839_b3'],
            'saturation': ['GS_1656149839_s1', 'GS_1656149839_s2', 'GS_1656149839_s3'],
            'numerical': ['GS_1656149839_n1', 'GS_1656149839_n2', 'GS_1656149839_n3', 'GS_1656149839_n4', 'GS_1656149839_n5', 'GS_1656149839_n6', 'GS_1656149839_n7'],
            'exit': []
        },
        4: {
            'boolean': ['GS_1656149839_b4'],
            'saturation': ['GS_1656149839_s4'],
            'numerical': ['GS_1656149839_n8', 'GS_1656149839_n9'],
            'exit': []
        }
    }

    DETECTORS_joinedS_9 = {
        0: {
            'boolean': ['joinedS_9_b1', 'joinedS_9_b2', 'joinedS_9_b3', 'joinedS_9_b4', 'joinedS_9_b6', 'joinedS_9_b7'],
            'saturation': ['joinedS_9_s1', 'joinedS_9_s2', 'joinedS_9_s4'],
            'numerical': ['joinedS_9_n1', 'joinedS_9_n2', 'joinedS_9_n3', 'joinedS_9_n4', 'joinedS_9_n6', 'joinedS_9_n7', 'joinedS_9_n8', 'joinedS_9_n9'],
            'exit': []
        },
        4: {
            'boolean': ['joinedS_9_b5', 'joinedS_9_b8', 'joinedS_9_b9'],
            'saturation': ['joinedS_9_s3', 'joinedS_9_s5'],
            'numerical': ['joinedS_9_n5', 'joinedS_9_n10', 'joinedS_9_n11', 'joinedS_9_n12'],
            'exit': []
        }
    }

    DETECTORS_1783585139 = {
        0: {
            'boolean': ['1783585139_b2'],
            'saturation': ['1783585139_s2'],
            'numerical': ['1783585139_n2'],
            'exit': []
        },
        2: {
            'boolean': ['1783585139_b3', '1783585139_b4'],
            'saturation': ['1783585139_s3', '1783585139_s4'],
            'numerical': ['1783585139_n3', '1783585139_n4'],
            'exit': []
        },
        4: {
            'boolean': ['1783585139_b1'],
            'saturation': ['1783585139_s1'],
            'numerical': ['1783585139_n1'],
            'exit': []
        }
    }

    DETECTORS_joinedS_10 = {
        0: {
            'boolean': ['joinedS_10_b1', 'joinedS_10_b2', 'joinedS_10_b3'],
            'saturation': ['joinedS_10_s1', 'joinedS_10_s2', 'joinedS_10_s3', 'joinedS_10_s4'],
            'numerical': ['joinedS_10_n1', 'joinedS_10_n2', 'joinedS_10_n3', 'joinedS_10_n4', 'joinedS_10_n5', 'joinedS_10_n6'],
            'exit': []
        },
        4: {
            'boolean': ['joinedS_10_b4', 'joinedS_10_b5'],
            'saturation': ['joinedS_10_s5', 'joinedS_10_s6'],
            'numerical': ['joinedS_10_n7', 'joinedS_10_n8', 'joinedS_10_n9', 'joinedS_10_n10'],
            'exit': []
        }
    }

    DETECTORS_1845505945 = {
        0: {
            'boolean': ['1845505945_b1', '1845505945_b3'],
            'saturation': ['1845505945_s1', '1845505945_s3'],
            'numerical': ['1845505945_n1', '1845505945_n2', '1845505945_n3', '1845505945_n5'],
            'exit': []
        },
        2: {
            'boolean': ['1845505945_b2'],
            'saturation': ['1845505945_s2'],
            'numerical': ['1845505945_n4'],
            'exit': []
        }
    }

    DETECTORS_GS_198873099 = {
        0: {
            'boolean': ['GS_198873099_b2'],
            'saturation': ['GS_198873099_s2'],
            'numerical': ['GS_198873099_n2'],
            'exit': []
        },
        2: {
            'boolean': ['GS_198873099_b1'],
            'saturation': ['GS_198873099_s1'],
            'numerical': ['GS_198873099_n1'],
            'exit': []
        }
    }

    DETECTORS_GS_225057604 = {
        0: {
            'boolean': ['GS_225057604_b1', 'GS_225057604_b2', 'GS_225057604_b3', 'GS_225057604_b4'],
            'saturation': ['GS_225057604_s1', 'GS_225057604_s2'],
            'numerical': ['GS_225057604_n1', 'GS_225057604_n2', 'GS_225057604_n3', 'GS_225057604_n4', 'GS_225057604_n5', 'GS_225057604_n6'],
            'exit': []
        },
        2: {
            'boolean': ['GS_225057604_b5'],
            'saturation': ['GS_225057604_s3'],
            'numerical': ['GS_225057604_n7'],
            'exit': []
        }
    }

    DETECTORS_235809784 = {
        0: {
            'boolean': ['235809784_b1', '235809784_b3'],
            'saturation': ['235809784_s1', '235809784_s3'],
            'numerical': ['235809784_n1', '235809784_n3'],
            'exit': []
        },
        4: {
            'boolean': ['235809784_b2'],
            'saturation': ['235809784_s2'],
            'numerical': ['235809784_n2'],
            'exit': []
        }
    }

    DETECTORS_GS_243072205 = {
        0: {
            'boolean': ['GS_243072205_b2', 'GS_243072205_b3', 'GS_243072205_b5'],
            'saturation': ['GS_243072205_s2', 'GS_243072205_s4'],
            'numerical': ['GS_243072205_n2', 'GS_243072205_n3', 'GS_243072205_n4', 'GS_243072205_n6'],
            'exit': []
        },
        4: {
            'boolean': ['GS_243072205_b1', 'GS_243072205_b4'],
            'saturation': ['GS_243072205_s1', 'GS_243072205_s3'],
            'numerical': ['GS_243072205_n1', 'GS_243072205_b5'],
            'exit': []
        },
    }

    DETECTORS_GS_243072206 = {
        0: {
            'boolean': ['GS_243072206_b2', 'GS_243072206_b4'],
            'saturation': ['GS_243072206_s2', 'GS_243072206_s4'],
            'numerical': ['GS_243072206_n2', 'GS_243072206_n4'],
            'exit': []
        },
        2: {
            'boolean': ['GS_243072206_b1', 'GS_243072206_b3'],
            'saturation': ['GS_243072206_s1', 'GS_243072206_s3'],
            'numerical': ['GS_243072206_n1', 'GS_243072206_n3'],
            'exit': []
        }
    }

    DETECTORS_GS_250883344 = {
        0: {
            'boolean': ['GS_250883344_b1', 'GS_250883344_b2'],
            'saturation': ['GS_250883344_s1', 'GS_250883344_s2'],
            'numerical': ['GS_250883344_n1', 'GS_250883344_n2'],
            'exit': []
        },
        2: {
            'boolean': ['GS_250883344_b3', 'GS_250883344_b4'],
            'saturation': ['GS_250883344_s3'],
            'numerical': ['GS_250883344_n3', 'GS_250883344_n4', 'GS_250883344_n5', 'GS_250883344_n6', 'GS_250883344_n7'],
            'exit': []
        }
    }

    DETECTORS_GS_251048925 = {
        0: {
            'boolean': ['GS_251048925_b1', 'GS_251048925_b3'],
            'saturation': ['GS_251048925_s1', 'GS_251048925_s3'],
            'numerical': ['GS_251048925_n1', 'GS_251048925_n3'],
            'exit': []
        },
        2: {
            'boolean': ['GS_251048925_b2', 'GS_251048925_b4'],
            'saturation': ['GS_251048925_s2', 'GS_251048925_s4'],
            'numerical': ['GS_251048925_n2', 'GS_251048925_n4'],
            'exit': []
        }
    }

    DETECTORS_GS_251050905 = {
        0: {
            'boolean': ['GS_251050905_b1'],
            'saturation': ['GS_251050905_s1'],
            'numerical': ['GS_251050905_n1'],
            'exit': []
        },
        2: {
            'boolean': ['GS_251050905_b3'],
            'saturation': ['GS_251050905_s3'],
            'numerical': ['GS_251050905_n3'],
            'exit': []
        },
        4: {
            'boolean': ['GS_251050905_b2'],
            'saturation': ['GS_251050905_s2'],
            'numerical': ['GS_251050905_n2'],
            'exit': []
        },
    }

    DETECTORS_251053238 = {
        0: {
            'boolean': ['251053238_b1', '251053238_b2'],
            'saturation': ['251053238_s1', '251053238_s2'],
            'numerical': ['251053238_n1', '251053238_n2'],
            'exit': []
        },
        4: {
            'boolean': ['251053238_b3'],
            'saturation': ['251053238_s3'],
            'numerical': ['251053238_n3'],
            'exit': []
        }
    }

    DETECTORS_251053453 = {
        0: {
            'boolean': ['251053453_b1', '251053453_b2'],
            'saturation': ['251053453_s1', '251053453_s2'],
            'numerical': ['251053453_n1', '251053453_n2'],
            'exit': []
        },
        4: {
            'boolean': ['251053453_b3'],
            'saturation': ['251053453_s3'],
            'numerical': ['251053453_n3'],
            'exit': []
        }
    }

    DETECTORS_GS_251053667 = {
        0: {
            'boolean': ['GS_251053667_b2', 'GS_251053667_b3'],
            'saturation': ['GS_251053667_s2', 'GS_251053667_s3'],
            'numerical': ['GS_251053667_n2', 'GS_251053667_n3'],
            'exit': []
        },
        4: {
            'boolean': ['GS_251053667_b1'],
            'saturation': ['GS_251053667_s1'],
            'numerical': ['GS_251053667_n1'],
            'exit': []
        }
    }

    DETECTORS_251053668 = {
        0: {
            'boolean': ['251053668_b1', '251053668_b2'],
            'saturation': ['251053668_s1', '251053668_s2'],
            'numerical': ['251053668_n1', '251053668_n2'],
            'exit': []
        },
        4: {
            'boolean': ['251053668_b3'],
            'saturation': ['251053668_s3'],
            'numerical': ['251053668_n3'],
            'exit': []
        }
    }

    DETECTORS_251054519 = {
        0: {
            'boolean': ['251054519_b3'],
            'saturation': ['251054519_s3'],
            'numerical': ['251054519_n3'],
            'exit': []
        },
        2: {
            'boolean': ['251054519_b1', '251054519_b2'],
            'saturation': ['251054519_s1', '251054519_s2'],
            'numerical': ['251054519_n1', '251054519_n2'],
            'exit': []
        }
    }

    DETECTORS_251056180 = {
        0: {
            'boolean': ['251056180_b2'],
            'saturation': ['251056180_s2', '251056180_s3'],
            'numerical': ['251056180_n2', '251056180_n3', '251056180_n4'],
            'exit': []
        },
        2: {
            'boolean': ['251056180_b1'],
            'saturation': ['251056180_s1'],
            'numerical': ['251056180_n1'],
            'exit': []
        }
    }

    DETECTORS_GS_251997373 = {
        0: {
            'boolean': ['GS_251997373_b1', 'GS_251997373_b2'],
            'saturation': ['GS_251997373_s1', 'GS_251997373_s2'],
            'numerical': ['GS_251997373_n1', 'GS_251997373_n2'],
            'exit': []
        },
        4: {
            'boolean': ['GS_251997373_b3', 'GS_251997373_b4'],
            'saturation': ['GS_251997373_s3'],
            'numerical': ['GS_251997373_n3', 'GS_251997373_n4', 'GS_251997373_n5'],
            'exit': []
        }
    }

    DETECTORS_252269915 = {
        0: {
            'boolean': ['252269915_b2'],
            'saturation': ['252269915_s2'],
            'numerical': ['252269915_n2'],
            'exit': []
        },
        2: {
            'boolean': ['252269915_b1'],
            'saturation': ['252269915_s1'],
            'numerical': ['252269915_n1'],
            'exit': []
        }
    }

    DETECTORS_260579122 = {
        0: {
            'boolean': ['260579122_b1', '260579122_b3'],
            'saturation': ['260579122_s1', '260579122_s3'],
            'numerical': ['260579122_n1', '260579122_n3'],
            'exit': []
        },
        4: {
            'boolean': ['260579122_b2'],
            'saturation': ['260579122_b2'],
            'numerical': ['260579122_b2'],
            'exit': []
        }
    }

    DETECTORS_260579123 = {
        0: {
            'boolean': ['260579123_b3', '260579123_b4'],
            'saturation': ['260579123_s3', '260579123_s4'],
            'numerical': ['260579123_n3', '260579123_n4'],
            'exit': []
        },
        2: {
            'boolean': ['260579123_b1'],
            'saturation': ['260579123_s1'],
            'numerical': ['260579123_n1'],
            'exit': []
        },
        4: {
            'boolean': ['260579123_b2'],
            'saturation': ['260579123_s2'],
            'numerical': ['260579123_n2'],
            'exit': []
        }
    }

    DETECTORS_GS_267375333 = {
        0: {
            'boolean': ['GS_267375333_b1', 'GS_267375333_b3'],
            'saturation': ['GS_267375333_s1', 'GS_267375333_s3'],
            'numerical': ['GS_267375333_n1', 'GS_267375333_n3'],
            'exit': []
        },
        4: {
            'boolean': ['GS_267375333_b2', 'GS_267375333_b4'],
            'saturation': ['GS_267375333_s2', 'GS_267375333_s4'],
            'numerical': ['GS_267375333_n2', 'GS_267375333_n4'],
            'exit': []
        }
    }

    DETECTORS_288393948 = {
        0: {
            'boolean': ['288393948_b1', '288393948_b2'],
            'saturation': ['288393948_s1', '288393948_s2'],
            'numerical': ['288393948_n1', '288393948_n2'],
            'exit': []
        },
        2: {
            'boolean': ['288393948_b3'],
            'saturation': ['288393948_s3'],
            'numerical': ['288393948_n3'],
            'exit': []
        }
    }

    DETECTORS_GS_288393949 = {
        0: {
            'boolean': ['GS_288393949_b2', 'GS_288393949_b3'],
            'saturation': ['GS_288393949_s2', 'GS_288393949_s3'],
            'numerical': ['GS_288393949_n2', 'GS_288393949_n3'],
            'exit': []
        },
        4: {
            'boolean': ['GS_288393949_b1'],
            'saturation': ['GS_288393949_s1'],
            'numerical': ['GS_288393949_n1'],
            'exit': []
        }
    }

    DETECTORS_295718903 = {
        0: {
            'boolean': ['295718903_b1', '295718903_b3'],
            'saturation': ['295718903_s1', '295718903_s3'],
            'numerical': ['295718903_n1', '295718903_n3'],
            'exit': []
        },
        2: {
            'boolean': ['295718903_b2', '295718903_b4'],
            'saturation': ['295718903_s2', '295718903_s4'],
            'numerical': ['295718903_n2', '295718903_n4'],
            'exit': []
        }
    }

    DETECTORS_GS_3076442881 = {
        0: {
            'boolean': ['GS_3076442881_b2'],
            'saturation': ['GS_3076442881_s2'],
            'numerical': ['GS_3076442881_n2'],
            'exit': []
        },
        2: {
            'boolean': ['GS_3076442881_b1'],
            'saturation': ['GS_3076442881_s1'],
            'numerical': ['GS_3076442881_n1'],
            'exit': []
        }
    }

    DETECTORS_GS_320950907 = {
        0: {
            'boolean': ['GS_320950907_b3'],
            'saturation': ['GS_320950907_s3'],
            'numerical': ['GS_320950907_n3'],
            'exit': []
        },
        2: {
            'boolean': ['GS_320950907_b1', 'GS_320950907_b2'],
            'saturation': ['GS_320950907_s1', 'GS_320950907_s2'],
            'numerical': ['GS_320950907_n1', 'GS_320950907_n2'],
            'exit': []
        }
    }

    DETECTORS_GS_352836899 = {
        0: {
            'boolean': ['GS_352836899_b2', 'GS_352836899_b4'],
            'saturation': ['GS_352836899_s2', 'GS_352836899_s4'],
            'numerical': ['GS_352836899_n2', 'GS_352836899_n4'],
            'exit': []
        },
        4: {
            'boolean': ['GS_352836899_b1', 'GS_352836899_b3'],
            'saturation': ['GS_352836899_s1', 'GS_352836899_s3'],
            'numerical': ['GS_352836899_n1', 'GS_352836899_n3'],
            'exit': []
        }
    }

    DETECTORS_362339802 = {
        0: {
            'boolean': ['362339802_b1', '362339802_b2'],
            'saturation': ['362339802_s1', '362339802_s2', '362339802_s3'],
            'numerical': ['362339802_n1', '362339802_n2', '362339802_n3', '362339802_n4'],
            'exit': []
        },
        4: {
            'boolean': ['362339802_b3'],
            'saturation': ['362339802_s4'],
            'numerical': ['362339802_n5'],
            'exit': []
        }
    }

    DETECTORS_GS_393330677 = {
        0: {
            'boolean': ['GS_393330677_b1', 'GS_393330677_b2', 'GS_393330677_b3', 'GS_393330677_b4'],
            'saturation': ['GS_393330677_s1', 'GS_393330677_s2'],
            'numerical': ['GS_393330677_n1', 'GS_393330677_n2', 'GS_393330677_n3', 'GS_393330677_n4', 'GS_393330677_n5', 'GS_393330677_n6'],
            'exit': []
        },
        2: {
            'boolean': ['GS_393330677_b5', 'GS_393330677_b6', 'GS_393330677_b7'],
            'saturation': ['GS_393330677_s3', 'GS_393330677_s4', 'GS_393330677_s5'],
            'numerical': ['GS_393330677_n7', 'GS_393330677_n8', 'GS_393330677_n9'],
            'exit': []
        }
    }

    DETECTORS_GS_439772972 = {
        0: {
            'boolean': ['GS_439772972_b2', 'GS_439772972_b3'],
            'saturation': ['GS_439772972_s2', 'GS_439772972_s3'],
            'numerical': ['GS_439772972_n2', 'GS_439772972_n3'],
            'exit': []
        },
        4: {
            'boolean': ['GS_439772972_b1'],
            'saturation': ['GS_439772972_s1'],
            'numerical': ['GS_439772972_n1'],
            'exit': []
        }
    }

    DETECTORS_439806870 = {
        0: {
            'boolean': ['439806870_b3'],
            'saturation': ['439806870_s3'],
            'numerical': ['439806870_n3'],
            'exit': []
        },
        2: {
            'boolean': ['439806870_b1', '439806870_b2'],
            'saturation': ['439806870_s1', '439806870_s2'],
            'numerical': ['439806870_n1', '439806870_n2'],
            'exit': []
        }
    }

    DETECTORS_439806886 = {
        0: {
            'boolean': ['439806886_b1', '439806886_b2'],
            'saturation': ['439806886_s1', '439806886_s2'],
            'numerical': ['439806886_n1', '439806886_n2'],
            'exit': []
        },
        4: {
            'boolean': ['439806886_b3'],
            'saturation': ['439806886_s3'],
            'numerical': ['439806886_n3'],
            'exit': []
        },
    }

    DETECTORS_440740820 = {
        0: {
            'boolean': ['440740820_b1', '440740820_b3'],
            'saturation': ['440740820_s1', '440740820_s3', '440740820_s4'],
            'numerical': ['440740820_n1', '440740820_n4', '440740820_n5'],
            'exit': []
        },
        2: {
            'boolean': ['440740820_b2', '440740820_b4'],
            'saturation': ['440740820_s2', '440740820_s5'],
            'numerical': ['440740820_n2', '440740820_n3', '440740820_n6'],
            'exit': []
        },
    }

    DETECTORS_GS_468704354 = {
        0: {
            'boolean': ['GS_468704354_b1', 'GS_468704354_b3'],
            'saturation': ['GS_468704354_s1', 'GS_468704354_s3'],
            'numerical': ['GS_468704354_n1', 'GS_468704354_n3'],
            'exit': []
        },
        4: {
            'boolean': ['GS_468704354_b2'],
            'saturation': ['GS_468704354_s2'],
            'numerical': ['GS_468704354_n2'],
            'exit': []
        },
    }

    DETECTORS_GS_4688955408 = {
        0: {
            'boolean': ['GS_4688955408_b1', 'GS_4688955408_b2', 'GS_4688955408_b3'],
            'saturation': ['GS_4688955408_s1', 'GS_4688955408_s2', 'GS_4688955408_s3'],
            'numerical': ['GS_4688955408_n1', 'GS_4688955408_n2', 'GS_4688955408_n3'],
            'exit': []
        },
        2: {
            'boolean': ['GS_4688955408_b4', 'GS_4688955408_b5', 'GS_4688955408_b6'],
            'saturation': ['GS_4688955408_s4', 'GS_4688955408_s5', 'GS_4688955408_s6'],
            'numerical': ['GS_4688955408_n4', 'GS_4688955408_n5', 'GS_4688955408_n6'],
            'exit': []
        },
    }

    DETECTORS_GS_469141720 = {
        0: {
            'boolean': ['GS_469141720_b2'],
            'saturation': ['GS_469141720_s2'],
            'numerical': ['GS_469141720_n2'],
            'exit': []
        },
        2: {
            'boolean': ['GS_469141720_b1'],
            'saturation': ['GS_469141720_s1'],
            'numerical': ['GS_469141720_n1'],
            'exit': []
        },
    }

    DETECTORS_GS_469309921 = {
        0: {
            'boolean': ['GS_469309921_b1', 'GS_469309921_b3'],
            'saturation': ['GS_469309921_s1', 'GS_469309921_s3'],
            'numerical': ['GS_469309921_n1', 'GS_469309921_n2', 'GS_469309921_n4'],
            'exit': []
        },
        4: {
            'boolean': ['GS_469309921_b2'],
            'saturation': ['GS_469309921_s2'],
            'numerical': ['GS_469309921_n2'],
            'exit': []
        },
    }

    DETECTORS_469309961 = {
        0: {
            'boolean': ['469309961_b2', '469309961_b3'],
            'saturation': ['469309961_s2', '469309961_s3'],
            'numerical': ['469309961_n2', '469309961_n3', '469309961_n4'],
            'exit': []
        },
        4: {
            'boolean': ['469309961_b1'],
            'saturation': ['469309961_s1'],
            'numerical': ['469309961_n1'],
            'exit': []
        },
    }

    DETECTORS_471611509 = {
        0: {
            'boolean': ['471611509_b2'],
            'saturation': ['471611509_s2'],
            'numerical': ['471611509_n2'],
            'exit': []
        },
        2: {
            'boolean': ['471611509_b1'],
            'saturation': ['471611509_s1'],
            'numerical': ['471611509_n1'],
            'exit': []
        },
    }

    DETECTORS_GS_471611512 = {
        0: {
            'boolean': ['GS_471611512_b2'],
            'saturation': ['GS_471611512_s2'],
            'numerical': ['GS_471611512_n2'],
            'exit': []
        },
        2: {
            'boolean': ['GS_471611512_b1'],
            'saturation': ['GS_471611512_s1'],
            'numerical': ['GS_471611512_n1'],
            'exit': []
        },
    }

    DETECTORS_GS_471611514 = {
        0: {
            'boolean': ['GS_471611514_b1', 'GS_471611514_b3'],
            'saturation': ['GS_471611514_s1', 'GS_471611514_s3'],
            'numerical': ['GS_471611514_n1', 'GS_471611514_n3'],
            'exit': []
        },
        4: {
            'boolean': ['GS_471611514_b2'],
            'saturation': ['GS_471611514_s2'],
            'numerical': ['GS_471611514_n2'],
            'exit': []
        },
    }

    DETECTORS_471611519 = {
        0: {
            'boolean': ['471611519_b2', '471611519_b3'],
            'saturation': ['471611519_s2', '471611519_s3'],
            'numerical': ['471611519_n2', '471611519_n3'],
            'exit': []
        },
        4: {
            'boolean': ['471611519_b1'],
            'saturation': ['471611519_s1'],
            'numerical': ['471611519_n1'],
            'exit': []
        },
    }

    DETECTORS_GS_476281346 = {
        0: {
            'boolean': ['GS_476281346_b2'],
            'saturation': ['GS_476281346_s2'],
            'numerical': ['GS_476281346_n2'],
            'exit': []
        },
        2: {
            'boolean': ['GS_476281346_b1'],
            'saturation': ['GS_476281346_s1'],
            'numerical': ['GS_476281346_n1'],
            'exit': []
        },
    }

    DETECTORS_485133343 = {
        0: {
            'boolean': ['485133343_b1', '485133343_b2'],
            'saturation': ['485133343_s1', '485133343_s2'],
            'numerical': ['485133343_n1', '485133343_n2'],
            'exit': []
        },
        4: {
            'boolean': ['485133343_b3'],
            'saturation': ['485133343_s3'],
            'numerical': ['485133343_n3'],
            'exit': []
        },
    }

    DETECTORS_GS_485133383 = {
        0: {
            'boolean': ['GS_485133383_b2', 'GS_485133383_b3'],
            'saturation': ['GS_485133383_s2', 'GS_485133383_s3'],
            'numerical': ['GS_485133383_n2', 'GS_485133383_n3'],
            'exit': []
        },
        4: {
            'boolean': ['GS_485133383_b1'],
            'saturation': ['GS_485133383_s1'],
            'numerical': ['GS_485133383_n1'],
            'exit': []
        },
    }

    DETECTORS_GS_485133394 = {
        0: {
            'boolean': ['GS_485133394_b2'],
            'saturation': ['GS_485133394_s2'],
            'numerical': ['GS_485133394_n2'],
            'exit': []
        },
        2: {
            'boolean': ['GS_485133394_b3'],
            'saturation': ['GS_485133394_s3'],
            'numerical': ['GS_485133394_n3'],
            'exit': []
        },
        4: {
            'boolean': ['GS_485133394_b1'],
            'saturation': ['GS_485133394_s1'],
            'numerical': ['GS_485133394_n1'],
            'exit': []
        },
    }

    DETECTORS_485133399 = {
        0: {
            'boolean': ['485133399_b2'],
            'saturation': ['485133399_s2'],
            'numerical': ['485133399_n2'],
            'exit': []
        },
        2: {
            'boolean': ['485133399_b1'],
            'saturation': ['485133399_s1'],
            'numerical': ['485133399_n1'],
            'exit': []
        },
    }

    DETECTORS_489576842 = {
        0: {
            'boolean': ['489576842_b1', '489576842_b3'],
            'saturation': ['489576842_s1', '489576842_s3'],
            'numerical': ['489576842_n1', '489576842_n3'],
            'exit': []
        },
        4: {
            'boolean': ['489576842_b2', '489576842_b4'],
            'saturation': ['489576842_s2', '489576842_s4'],
            'numerical': ['489576842_n2', '489576842_n4'],
            'exit': []
        },
    }

    DETECTORS_GS_492092314 = {
        0: {
            'boolean': ['GS_492092314_b1', 'GS_492092314_b3'],
            'saturation': ['GS_492092314_s1', 'GS_492092314_s3'],
            'numerical': ['GS_492092314_n1', 'GS_492092314_n3'],
            'exit': []
        },
        4: {
            'boolean': ['GS_492092314_b2'],
            'saturation': ['GS_492092314_s2'],
            'numerical': ['GS_492092314_n2'],
            'exit': []
        },
    }

    DETECTORS_492204162 = {
        0: {
            'boolean': ['492204162_b3'],
            'saturation': ['492204162_s3'],
            'numerical': ['492204162_n3'],
            'exit': []
        },
        2: {
            'boolean': ['492204162_b2'],
            'saturation': ['492204162_s2'],
            'numerical': ['492204162_n2'],
            'exit': []
        },
        4: {
            'boolean': ['492204162_b1'],
            'saturation': ['492204162_s1'],
            'numerical': ['492204162_n1'],
            'exit': []
        },
    }

    DETECTORS_492204171 = {
        0: {
            'boolean': ['492204171_b2'],
            'saturation': ['492204171_s2'],
            'numerical': ['492204171_n2'],
            'exit': []
        },
        2: {
            'boolean': ['492204171_b1'],
            'saturation': ['492204171_s1'],
            'numerical': ['492204171_n1'],
            'exit': []
        },
    }

    DETECTORS_493138763 = {
        0: {
            'boolean': ['493138763_b1'],
            'saturation': ['493138763_s1'],
            'numerical': ['493138763_n1'],
            'exit': []
        },
        2: {
            'boolean': ['493138763_b2', '493138763_b3'],
            'saturation': ['493138763_s2'],
            'numerical': ['493138763_n2', '493138763_n3', '493138763_n4'],
            'exit': []
        },
    }

    DETECTORS_GS_494986735 = {
        0: {
            'boolean': ['GS_494986735_b3'],
            'saturation': ['GS_494986735_s3'],
            'numerical': ['GS_494986735_n3'],
            'exit': []
        },
        2: {
            'boolean': ['GS_494986735_b1', 'GS_494986735_b2'],
            'saturation': ['GS_494986735_s1', 'GS_494986735_s2'],
            'numerical': ['GS_494986735_n1', 'GS_494986735_n2'],
            'exit': []
        },
    }

    DETECTORS_494986738 = {
        0: {
            'boolean': ['494986738_b1', '494986738_b3'],
            'saturation': ['494986738_s1', '494986738_s3'],
            'numerical': ['494986738_n1', '494986738_n3'],
            'exit': []
        },
        2: {
            'boolean': ['494986738_b2', '494986738_b4'],
            'saturation': ['494986738_s2', '494986738_s4'],
            'numerical': ['494986738_n2', '494986738_n4'],
            'exit': []
        },
    }

    DETECTORS_496978290 = {
        0: {
            'boolean': ['496978290_b1', '496978290_b3'],
            'saturation': ['496978290_s1', '496978290_s3'],
            'numerical': ['496978290_n1', '496978290_n3'],
            'exit': []
        },
        4: {
            'boolean': ['496978290_b2'],
            'saturation': ['496978290_s2'],
            'numerical': ['496978290_n2'],
            'exit': []
        },
    }

    DETECTORS_497394917 = {
        0: {
            'boolean': ['497394917_b1', '497394917_b3'],
            'saturation': ['497394917_s1', '497394917_s3'],
            'numerical': ['497394917_n1', '497394917_n3'],
            'exit': []
        },
        2: {
            'boolean': ['497394917_b2'],
            'saturation': ['497394917_s2'],
            'numerical': ['497394917_n2'],
            'exit': []
        },
    }

    DETECTORS_GS_498476334 = {
        0: {
            'boolean': ['GS_498476334_b1', 'GS_498476334_b2'],
            'saturation': ['GS_498476334_s1', 'GS_498476334_s2'],
            'numerical': ['GS_498476334_n1', 'GS_498476334_n2'],
            'exit': []
        },
        4: {
            'boolean': ['GS_498476334_b3'],
            'saturation': ['GS_498476334_s3'],
            'numerical': ['GS_498476334_n3'],
            'exit': []
        },
    }

    DETECTORS_GS_506774198 = {
        0: {
            'boolean': ['GS_506774198_b1', 'GS_506774198_b3'],
            'saturation': ['GS_506774198_s1', 'GS_506774198_s3'],
            'numerical': ['GS_506774198_n1', 'GS_506774198_n3'],
            'exit': []
        },
        4: {
            'boolean': ['GS_506774198_b2'],
            'saturation': ['GS_506774198_s2'],
            'numerical': ['GS_506774198_n2'],
            'exit': []
        },
    }

    DETECTORS_joinedS_16 = {
        0: {
            'boolean': ['joinedS_16_b1', 'joinedS_16_b2', 'joinedS_16_b3', 'joinedS_16_b4', 'joinedS_16_b7', 'joinedS_16_b8', 'joinedS_16_b9'],
            'saturation': ['joinedS_16_s1', 'joinedS_16_s3', 'joinedS_16_s4', 'joinedS_16_s5'],
            'numerical': ['joinedS_16_n1', 'joinedS_16_n2', 'joinedS_16_n3', 'joinedS_16_n4', 'joinedS_16_n7', 'joinedS_16_n8', 'joinedS_16_n9'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_16_b5', 'joinedS_16_b6', 'joinedS_16_b10', 'joinedS_16_b11', 'joinedS_16_b12'],
            'saturation': ['joinedS_16_s2', 'joinedS_16_s6'],
            'numerical': ['joinedS_16_n5', 'joinedS_16_n6', 'joinedS_16_n10', 'joinedS_16_n11', 'joinedS_16_n12'],
            'exit': []
        },
    }

    DETECTORS_GS_508063549 = {
        0: {
            'boolean': ['GS_508063549_b1'],
            'saturation': ['GS_508063549_s1'],
            'numerical': ['GS_508063549_n1'],
            'exit': []
        },
        2: {
            'boolean': ['GS_508063549_b2'],
            'saturation': ['GS_508063549_s2'],
            'numerical': ['GS_508063549_n2', 'GS_508063549_n3'],
            'exit': []
        },
    }

    DETECTORS_517340666 = {
        0: {
            'boolean': ['517340666_b2'],
            'saturation': ['517340666_s2'],
            'numerical': ['517340666_n2'],
            'exit': []
        },
        2: {
            'boolean': ['517340666_b1'],
            'saturation': ['517340666_s1'],
            'numerical': ['517340666_n1'],
            'exit': []
        },
    }

    DETECTORS_517340669 = {
        0: {
            'boolean': ['517340669_b2'],
            'saturation': ['517340669_s2'],
            'numerical': ['517340669_n2'],
            'exit': []
        },
        2: {
            'boolean': ['517340669_b1'],
            'saturation': ['517340669_s1'],
            'numerical': ['517340669_n1'],
            'exit': []
        },
    }

    DETECTORS_517340672 = {
        0: {
            'boolean': ['517340672_b1', '517340672_b2'],
            'saturation': ['517340672_s1', '517340672_s2'],
            'numerical': ['517340672_n1', '517340672_n2'],
            'exit': []
        },
        2: {
            'boolean': ['517340672_b3'],
            'saturation': ['517340672_s3'],
            'numerical': ['517340672_n3'],
            'exit': []
        },
    }

    DETECTORS_530710713 = {
        0: {
            'boolean': ['530710713_b1'],
            'saturation': ['530710713_s1'],
            'numerical': ['530710713_n1'],
            'exit': []
        },
        2: {
            'boolean': ['530710713_b2'],
            'saturation': ['530710713_s2'],
            'numerical': ['530710713_n2'],
            'exit': []
        },
    }

    DETECTORS_GS_6257632307 = {
        0: {
            'boolean': ['GS_6257632307_b4', 'GS_6257632307_b5', 'GS_6257632307_b6'],
            'saturation': ['GS_6257632307_s4', 'GS_6257632307_s5', 'GS_6257632307_s6'],
            'numerical': ['GS_6257632307_n4', 'GS_6257632307_n5', 'GS_6257632307_n6'],
            'exit': []
        },
        2: {
            'boolean': ['GS_6257632307_b1', 'GS_6257632307_b2', 'GS_6257632307_b3'],
            'saturation': ['GS_6257632307_s1', 'GS_6257632307_s2', 'GS_6257632307_s3'],
            'numerical': ['GS_6257632307_n1', 'GS_6257632307_n2', 'GS_6257632307_n3'],
            'exit': []
        },
    }

    DETECTORS_GS_6286429547 = {
        0: {
            'boolean': ['GS_6286429547_b1', 'GS_6286429547_b2'],
            'saturation': ['GS_6286429547_s1', 'GS_6286429547_s2'],
            'numerical': ['GS_6286429547_n1', 'GS_6286429547_n2'],
            'exit': []
        },
        2: {
            'boolean': ['GS_6286429547_b3', 'GS_6286429547_b4', 'GS_6286429547_b5', 'GS_6286429547_b6'],
            'saturation': ['GS_6286429547_s3', 'GS_6286429547_s4'],
            'numerical': ['GS_6286429547_n3', 'GS_6286429547_n4', 'GS_6286429547_n5', 'GS_6286429547_n6'],
            'exit': []
        },
    }

    DETECTORS_GS_8983324940 = {
        0: {
            'boolean': ['GS_8983324940_b2'],
            'saturation': ['GS_8983324940_s2'],
            'numerical': ['GS_8983324940_n2'],
            'exit': []
        },
        2: {
            'boolean': ['GS_8983324940_b1'],
            'saturation': ['GS_8983324940_s1'],
            'numerical': ['GS_8983324940_n1'],
            'exit': []
        },
    }

    DETECTORS_GS_9160219128 = {
        0: {
            'boolean': ['GS_9160219128_b2', 'GS_9160219128_b3'],
            'saturation': ['GS_9160219128_s2', 'GS_9160219128_s3'],
            'numerical': ['GS_9160219128_n2', 'GS_9160219128_n3'],
            'exit': []
        },
        4: {
            'boolean': ['GS_9160219128_b1'],
            'saturation': ['GS_9160219128_s1'],
            'numerical': ['GS_9160219128_n1'],
            'exit': []
        },
    }

    DETECTORS_cluster1638013968_cluster_1638013992_1656149900_4052112289 = {
        0: {
            'boolean': ['cluster1638013968_b1', 'cluster1638013968_b2', 'cluster1638013968_b5', 'cluster1638013968_b6'],
            'saturation': ['cluster1638013968_s1', 'cluster1638013968_s2', 'cluster1638013968_s4', 'cluster1638013968_s5'],
            'numerical': ['cluster1638013968_n1', 'cluster1638013968_n2', 'cluster1638013968_n6', 'cluster1638013968_n7'],
            'exit': []
        },
        4: {
            'boolean': ['cluster1638013968_b3', 'cluster1638013968_b4'],
            'saturation': ['cluster1638013968_s3'],
            'numerical': ['cluster1638013968_n3', 'cluster1638013968_n4', 'cluster1638013968_n5'],
            'exit': []
        },
    }

    DETECTORS_cluster_10861016298_1299481357_1299481366_164480282_849010069 = {
        0: {
            'boolean': ['cluster_10861016298_b1', 'cluster_10861016298_b4', 'cluster_10861016298_b5', 'cluster_10861016298_b6'],
            'saturation': ['cluster_10861016298_s1', 'cluster_10861016298_s4', 'cluster_10861016298_s5'],
            'numerical': ['cluster_10861016298_n1', 'cluster_10861016298_n4', 'cluster_10861016298_n5', 'cluster_10861016298_n6', 'cluster_10861016298_n7', 'cluster_10861016298_n8'],
            'exit': []
        },
        4: {
            'boolean': ['cluster_10861016298_b2', 'cluster_10861016298_b3'],
            'saturation': ['cluster_10861016298_s2', 'cluster_10861016298_s3'],
            'numerical': ['cluster_10861016298_n2', 'cluster_10861016298_n3'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1302128722_3075924708_8674467625_8674467626 = {
        0: {
            'boolean': ['cluster_1302128722_b1', 'cluster_1302128722_b2', 'cluster_1302128722_b4'],
            'saturation': ['cluster_1302128722_s1', 'cluster_1302128722_s2', 'cluster_1302128722_s4'],
            'numerical': ['cluster_1302128722_n1', 'cluster_1302128722_n4', 'cluster_1302128722_n5'],
            'exit': []
        },
        4: {
            'boolean': ['cluster_1302128722_b2'],
            'saturation': ['cluster_1302128722_s2'],
            'numerical': ['cluster_1302128722_n2', 'cluster_1302128722_n3'],
            'exit': []
        },
    }

    DETECTORS_cluster_1499392862_393332459_5936292019 = {
        0: {
            'boolean': ['cluster_1499392862_b1', 'cluster_1499392862_b2', 'cluster_1499392862_b3'],
            'saturation': ['cluster_1499392862_s1', 'cluster_1499392862_s2', 'cluster_1499392862_s3', 'cluster_1499392862_s4'],
            'numerical': ['cluster_1499392862_n1', 'cluster_1499392862_n2', 'cluster_1499392862_n3', 'cluster_1499392862_n4', 'cluster_1499392862_n5', 'cluster_1499392862_n6'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_1499392862_b4'],
            'saturation': ['cluster_1499392862_s5'],
            'numerical': ['cluster_1499392862_n7'],
            'exit': []
        },
    }

    DETECTORS_cluster_1575153557_260469525 = {
        0: {
            'boolean': ['cluster_1575153557_b2', 'cluster_1575153557_b4'],
            'saturation': ['cluster_1575153557_s2', 'cluster_1575153557_s4'],
            'numerical': ['cluster_1575153557_n2', 'cluster_1575153557_n4'],
            'exit': []
        },
        4: {
            'boolean': ['cluster_1575153557_b1', 'cluster_1575153557_b3'],
            'saturation': ['cluster_1575153557_s1', 'cluster_1575153557_s3'],
            'numerical': ['cluster_1575153557_n1', 'cluster_1575153557_n3'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1628579789_243072201 = {
        0: {
            'boolean': ['GS_cluster_1628579789_b1', 'GS_cluster_1628579789_b3'],
            'saturation': ['GS_cluster_1628579789_s1', 'GS_cluster_1628579789_s3'],
            'numerical': ['GS_cluster_1628579789_n1', 'GS_cluster_1628579789_n3'],
            'exit': []
        },
        2: {
            'boolean': ['GS_cluster_1628579789_b2'],
            'saturation': ['GS_cluster_1628579789_s2'],
            'numerical': ['GS_cluster_1628579789_n2'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1635333815_251056169_252192622_6233070869_6233070871_6233070875_6233070881_6304582153 = {
        0: {
            'boolean': ['GS_cluster_1635333815_b2', 'GS_cluster_1635333815_b5', 'GS_cluster_1635333815_b6'],
            'saturation': ['GS_cluster_1635333815_s2', 'GS_cluster_1635333815_s5'],
            'numerical': ['GS_cluster_1635333815_n2', 'GS_cluster_1635333815_n5', 'GS_cluster_1635333815_n6', 'GS_cluster_1635333815_n7'],
            'exit': []
        },
        2: {
            'boolean': ['GS_cluster_1635333815_b1', 'GS_cluster_1635333815_b3', 'GS_cluster_1635333815_b4'],
            'saturation': ['GS_cluster_1635333815_s1', 'GS_cluster_1635333815_s3', 'GS_cluster_1635333815_s4'],
            'numerical': ['GS_cluster_1635333815_n1', 'GS_cluster_1635333815_n3', 'GS_cluster_1635333815_n4'],
            'exit': []
        },
    }

    DETECTORS_joinedS_17 = {
        0: {
            'boolean': ['joinedS_17_b5', 'joinedS_17_b6', 'joinedS_17_b7', 'joinedS_17_b12', 'joinedS_17_b13'],
            'saturation': ['joinedS_17_s2', 'joinedS_17_s3', 'joinedS_17_s4', 'joinedS_17_s6', 'joinedS_17_s7'],
            'numerical': ['joinedS_17_n6', 'joinedS_17_n7', 'joinedS_17_n8', 'joinedS_17_n14', 'joinedS_17_n15', 'joinedS_17_n16', 'joinedS_17_n17'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_17_b1', 'joinedS_17_b2', 'joinedS_17_b3', 'joinedS_17_b4', 'joinedS_17_b8', 'joinedS_17_b9', 'joinedS_17_b10', 'joinedS_17_b11'],
            'saturation': ['joinedS_17_s1', 'joinedS_17_s5'],
            'numerical': ['joinedS_17_n1', 'joinedS_17_n2', 'joinedS_17_n3', 'joinedS_17_n4', 'joinedS_17_n8', 'joinedS_17_n9', 'joinedS_17_n10', 'joinedS_17_n11'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1674605528_478487605 = {
        0: {
            'boolean': ['GS_cluster_1674605528_b1', 'GS_cluster_1674605528_b2'],
            'saturation': ['GS_cluster_1674605528_s1', 'GS_cluster_1674605528_s2'],
            'numerical': ['GS_cluster_1674605528_n1', 'GS_cluster_1674605528_n2', 'GS_cluster_1674605528_n3'],
            'exit': []
        },
        2: {
            'boolean': ['GS_cluster_1674605528_b3'],
            'saturation': ['GS_cluster_1674605528_s3'],
            'numerical': ['GS_cluster_1674605528_n4'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1705440129_250883337_251056183 = {
        0: {
            'boolean': ['GS_cluster_1705440129_b2', 'GS_cluster_1705440129_b3'],
            'saturation': ['GS_cluster_1705440129_s2', 'GS_cluster_1705440129_s3'],
            'numerical': ['GS_cluster_1705440129_n3', 'GS_cluster_1705440129_n4'],
            'exit': []
        },
        4: {
            'boolean': ['GS_cluster_1705440129_b1'],
            'saturation': ['GS_cluster_1705440129_s1'],
            'numerical': ['GS_cluster_1705440129_n1', 'GS_cluster_1705440129_n2'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_1800186890_251997370 = {
        0: {
            'boolean': ['GS_cluster_1800186890_b3', 'GS_cluster_1800186890_b4', 'GS_cluster_1800186890_b5'],
            'saturation': ['GS_cluster_1800186890_s3', 'GS_cluster_1800186890_s4'],
            'numerical': ['GS_cluster_1800186890_b3', 'GS_cluster_1800186890_n4', 'GS_cluster_1800186890_n5', 'GS_cluster_1800186890_n6'],
            'exit': []
        },
        4: {
            'boolean': ['GS_cluster_1800186890_b1', 'GS_cluster_1800186890_b2'],
            'saturation': ['GS_cluster_1800186890_s1', 'GS_cluster_1800186890_s2'],
            'numerical': ['GS_cluster_1800186890_n1', 'GS_cluster_1800186890_n2'],
            'exit': []
        },
    }

    DETECTORS_cluster_198873054_3077077638 = {
        0: {
            'boolean': ['cluster_198873054_b2', 'cluster_198873054_b3', 'cluster_198873054_b4'],
            'saturation': ['cluster_198873054_s2', 'cluster_198873054_s3'],
            'numerical': ['cluster_198873054_n2', 'cluster_198873054_n3', 'cluster_198873054_n4', 'cluster_198873054_n5', 'cluster_198873054_n6', 'cluster_198873054_n7'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_198873054_b1'],
            'saturation': ['cluster_198873054_s1'],
            'numerical': ['cluster_198873054_n1'],
            'exit': []
        },
    }

    DETECTORS_cluster_198873247_506774200 = {
        0: {
            'boolean': ['cluster_198873247_b1', 'cluster_198873247_b2'],
            'saturation': ['cluster_198873247_s1', 'cluster_198873247_s2'],
            'numerical': ['cluster_198873247_n1', 'cluster_198873247_n2'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_198873247_b4'],
            'saturation': ['cluster_198873247_s4'],
            'numerical': ['cluster_198873247_n4'],
            'exit': []
        },
        4: {
            'boolean': ['cluster_198873247_b3'],
            'saturation': ['cluster_198873247_s3'],
            'numerical': ['cluster_198873247_n3'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_198873257_8568267658 = {
        0: {
            'boolean': ['GS_cluster_198873257_b3', 'GS_cluster_198873257_b4', 'GS_cluster_198873257_b6'],
            'saturation': ['GS_cluster_198873257_s3', 'GS_cluster_198873257_s4', 'GS_cluster_198873257_s6'],
            'numerical': ['GS_cluster_198873257_n3', 'GS_cluster_198873257_n4', 'GS_cluster_198873257_n6', 'GS_cluster_198873257_n7'],
            'exit': []
        },
        2: {
            'boolean': ['GS_cluster_198873257_b1', 'GS_cluster_198873257_b2', 'GS_cluster_198873257_b5'],
            'saturation': ['GS_cluster_198873257_s1', 'GS_cluster_198873257_s2', 'GS_cluster_198873257_s5'],
            'numerical': ['GS_cluster_198873257_n1', 'GS_cluster_198873257_n2', 'GS_cluster_198873257_n5'],
            'exit': []
        },
    }

    DETECTORS_cluster_2327596112_439772980_469309959 = {
        0: {
            'boolean': ['cluster_2327596112_b1', 'cluster_2327596112_b3'],
            'saturation': ['cluster_2327596112_s1', 'cluster_2327596112_s3'],
            'numerical': ['cluster_2327596112_n1', 'cluster_2327596112_n3'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_2327596112_b2', 'cluster_2327596112_b4'],
            'saturation': ['cluster_2327596112_s2', 'cluster_2327596112_s4'],
            'numerical': ['cluster_2327596112_n2', 'cluster_2327596112_n4'],
            'exit': []
        },
    }

    DETECTORS_cluster_250883332_251056162 = {
        0: {
            'boolean': ['cluster_250883332_b1', 'cluster_250883332_b2', 'cluster_250883332_b4'],
            'saturation': ['cluster_250883332_s1', 'cluster_250883332_s2', 'cluster_250883332_s4'],
            'numerical': ['cluster_250883332_n1', 'cluster_250883332_n2', 'cluster_250883332_n4'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_250883332_b3'],
            'saturation': ['cluster_250883332_s3'],
            'numerical': ['cluster_250883332_n3'],
            'exit': []
        },
    }

    DETECTORS_cluster_250889004_250896553_3076475495_3076475499 = {
        0: {
            'boolean': ['cluster_250889004_b1'],
            'saturation': ['cluster_250889004_s1'],
            'numerical': ['cluster_250889004_n1'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_250889004_b2', 'cluster_250889004_b3'],
            'saturation': ['cluster_250889004_s2', 'cluster_250889004_s3'],
            'numerical': ['cluster_250889004_n2', 'cluster_250889004_n3'],
            'exit': []
        },
    }


    DETECTORS_GS_cluster_251997367_251997369_6525168932_6525168938_8666071616 = {
        0: {
            'boolean': ['GS_cluster_251997367_b4', 'GS_cluster_251997367_b5', 'GS_cluster_251997367_b6', 'GS_cluster_251997367_b7'],
            'saturation': ['GS_cluster_251997367_s4'],
            'numerical': ['GS_cluster_251997367_n4', 'GS_cluster_251997367_n5', 'GS_cluster_251997367_n6', 'GS_cluster_251997367_n7', 'GS_cluster_251997367_n8'],
            'exit': []
        },
        2: {
            'boolean': ['GS_cluster_251997367_b1', 'GS_cluster_251997367_b2', 'GS_cluster_251997367_b3'],
            'saturation': ['GS_cluster_251997367_s1', 'GS_cluster_251997367_s2', 'GS_cluster_251997367_s3'],
            'numerical': ['GS_cluster_251997367_n1', 'GS_cluster_251997367_n2', 'GS_cluster_251997367_n3'],
            'exit': []
        },
    }

    DETECTORS_joinedS_18 = {
        0: {
            'boolean': ['joinedS_18_b1'],
            'saturation': ['joinedS_18_s1'],
            'numerical': ['joinedS_18_n1', 'joinedS_18_n2', 'joinedS_18_n3'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_18_b2', 'joinedS_18_b3'],
            'saturation': ['joinedS_18_s2', 'joinedS_18_s3'],
            'numerical': ['joinedS_18_n4', 'joinedS_18_n5'],
            'exit': []
        },
    }

    DETECTORS_cluster_294326287_531028789_5343505332 = {
        0: {
            'boolean': ['cluster_294326287_b1', 'cluster_294326287_b2'],
            'saturation': ['cluster_294326287_s1', 'cluster_294326287_s2'],
            'numerical': ['cluster_294326287_n1', 'cluster_294326287_n2'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_294326287_b3'],
            'saturation': ['cluster_294326287_s3'],
            'numerical': ['cluster_294326287_n3'],
            'exit': []
        },
        4: {
            'boolean': ['cluster_294326287_b4'],
            'saturation': ['cluster_294326287_s4'],
            'numerical': ['cluster_294326287_n4'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_408501435_6315125559 = {
        0: {
            'boolean': ['GS_cluster_408501435_b2', 'GS_cluster_408501435_b3', 'GS_cluster_408501435_b5', 'GS_cluster_408501435_b6'],
            'saturation': ['GS_cluster_408501435_s2', 'GS_cluster_408501435_s4', 'GS_cluster_408501435_s5'],
            'numerical': ['GS_cluster_408501435_n2', 'GS_cluster_408501435_n3', 'GS_cluster_408501435_n4', 'GS_cluster_408501435_n6', 'GS_cluster_408501435_n7'],
            'exit': []
        },
        2: {
            'boolean': ['GS_cluster_408501435_b1', 'GS_cluster_408501435_b4'],
            'saturation': ['GS_cluster_408501435_s1', 'GS_cluster_408501435_s3'],
            'numerical': ['GS_cluster_408501435_n1', 'GS_cluster_408501435_n5'],
            'exit': []
        },
    }

    DETECTORS_cluster_485133368_485133379 = {
        0: {
            'boolean': ['cluster_485133368_b1', 'cluster_485133368_b4'],
            'saturation': ['cluster_485133368_s1', 'cluster_485133368_s4'],
            'numerical': ['cluster_485133368_n1', 'cluster_485133368_n4'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_485133368_b2', 'cluster_485133368_b3'],
            'saturation': ['cluster_485133368_s2', 'cluster_485133368_s3'],
            'numerical': ['cluster_485133368_n2', 'cluster_485133368_n3'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_488539561_8567356615 = {
        0: {
            'boolean': ['GS_cluster_488539561_b1', 'GS_cluster_488539561_b3'],
            'saturation': ['GS_cluster_488539561_s1', 'GS_cluster_488539561_s3'],
            'numerical': ['GS_cluster_488539561_n1', 'GS_cluster_488539561_n3'],
            'exit': []
        },
        2: {
            'boolean': ['GS_cluster_488539561_b2'],
            'saturation': ['GS_cluster_488539561_s2'],
            'numerical': ['GS_cluster_488539561_n2'],
            'exit': []
        },
    }

    DETECTORS_cluster_530710726_683650128 = {
        0: {
            'boolean': ['cluster_530710726_b1', 'cluster_530710726_b4'],
            'saturation': ['cluster_530710726_s1', 'cluster_530710726_s4'],
            'numerical': ['cluster_530710726_n1', 'cluster_530710726_n4'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_530710726_b2'],
            'saturation': ['cluster_530710726_s2'],
            'numerical': ['cluster_530710726_n2'],
            'exit': []
        },
        4: {
            'boolean': ['cluster_530710726_b3'],
            'saturation': ['cluster_530710726_s3'],
            'numerical': ['cluster_530710726_n3'],
            'exit': []
        },
    }

    DETECTORS_GS_cluster_6286429546_6286429552 = {
        0: {
            'boolean': ['GS_cluster_6286429546_b2', 'GS_cluster_6286429546_b3'],
            'saturation': ['GS_cluster_6286429546_s2', 'GS_cluster_6286429546_s3'],
            'numerical': ['GS_cluster_6286429546_n2', 'GS_cluster_6286429546_n3', 'GS_cluster_6286429546_n4', 'GS_cluster_6286429546_n5'],
            'exit': []
        },
        2: {
            'boolean': ['GS_cluster_6286429546_b1', 'GS_cluster_6286429546_b4'],
            'saturation': ['GS_cluster_6286429546_b1', 'GS_cluster_6286429546_b4'],
            'numerical': ['GS_cluster_6286429546_n1', 'GS_cluster_6286429546_n6', 'GS_cluster_6286429546_n7'],
            'exit': []
        },
    }

    DETECTORS_cluster_6286429557_6990557731 = {
        0: {
            'boolean': ['cluster_6286429557_b2', 'cluster_6286429557_b3'],
            'saturation': ['cluster_6286429557_s2'],
            'numerical': ['cluster_6286429557_n2', 'cluster_6286429557_n3', 'cluster_6286429557_n4', 'cluster_6286429557_n5', 'cluster_6286429557_n6'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_6286429557_b1'],
            'saturation': ['cluster_6286429557_s1'],
            'numerical': ['cluster_6286429557_n1'],
            'exit': []
        },
    }

    DETECTORS_J02 = {
        0: {
            'boolean': ['J02_b1', 'J02_b2'],
            'saturation': ['J02_s1', 'J02_s2'],
            'numerical': ['J02_n1', 'J02_n2', 'J02_n3', 'J02_n4'],
            'exit': []
        },
        2: {
            'boolean': ['J02_b3', 'J02_b4', 'J02_b5', 'J02_b6'],
            'saturation': ['J02_s3', 'J02_s4', 'J02_s5', 'J02_s6'],
            'numerical': ['J02_n5', 'J02_n6', 'J02_n7''J02_n8'],
            'exit': []
        },
        4: {
            'boolean': ['J02_b7', 'J02_b8', 'J02_b9'],
            'saturation': ['J02_s7', 'J02_s8', 'J02_s9', 'J02_s10'],
            'numerical': ['J02_n9', 'J02_n10', 'J02_n11', 'J02_n12', 'J02_n13', 'J02_n14', 'J02_n15'],
            'exit': []
        },
        6: {
            'boolean': ['J02_b10', 'J02_b11'],
            'saturation': ['J02_s11', 'J02_s12'],
            'numerical': ['J02_n16', 'J02_n17', 'J02_n18', 'J02_n19'],
            'exit': []
        }
    }

if __name__ == '__main__':
    lille = LilleNetwork()
    print(lille.generate_flows_intra_city(100))


