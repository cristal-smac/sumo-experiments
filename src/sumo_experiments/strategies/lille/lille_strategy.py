from abc import ABC, abstractmethod

class LilleStrategy(ABC):
    """
    Abstract class for Lille strategies
    """

    def __init__(self):
        """
        Init of class
        """
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
            'cluster1299481290_252000834_4063893143': self.DETECTORS_cluster1299481290_252000834_4063893143,
            'J02': self.DETECTORS_J02,
            'J0333': self.DETECTORS_J0333,
            'GS_3075917655': self.DETECTORS_GS_3075917655,
            'joinedS_12': self.DETECTORS_joinedS_12,
            '652409498': self.DETECTORS_652409498,
            'cluster1302567586_149374901_305918149': self.DETECTORS_cluster1302567586_149374901_305918149,
            'cluster1302567574_149374913_305918532': self.DETECTORS_cluster1302567574_149374913_305918532,
            'GS_cluster_2298692584_7170315064': self.DETECTORS_GS_cluster_2298692584_7170315064,
            'joinedS_5': self.DETECTORS_joinedS_5,
            '491543745': self.DETECTORS_491543745,

        }

    TL_IDS = ['joinedS_1', '6301940736', 'cluster1681715927_250883340_274897939_6301940714', '6316129114', 'GS_cluster_1615590751_3305892794', 'GS_cluster_1544209593_1615582086',
              'joinedS_13', 'GS_cluster_1773321434_198873202', 'cluster1589650044_cluster1589650045_1793003425', 'GS_cluster_506774252_8568267669_8568267670_8568267671',
              'GS_cluster_1589656419_1770465596', 'GS_cluster_1589659457_1912530468', 'GS_cluster_1770465595_4365093914', 'GS_cluster_149374929_1770465614_305929185',
              'GS_cluster_507288390_508056193', 'cluster_485133449_485133452', 'GS_cluster_485133387_485133391', 'cluster1638086122_cluster_251058778_32828632',
              'GS_cluster_1635760230_1635775294_3706468062', '6267747172', 'GS_cluster_1650597986_1650597989_1652594655', 'GS_cluster_1648564005_1648564008', 'cluster1647146953_1647146957',
              'joinedS_4', 'J11', 'cluster1436583672_1713998722_4401347167_4401347179_#2more', 'GS_cluster_426634816_6316128557_6316128558_801903207_801903212', 'GS_cluster_243072211_250894314',
              'GS_cluster_1800186885_1800186886_243072210_3404227323_494986739', 'GS_cluster_1302128733_423805246', 'GS_cluster_267375483_3077077589', 'cluster1299481290_252000834_4063893143',
              'J02', 'J0333', 'GS_3075917655', '652409498', 'cluster1302567574_149374913_305918532', 'GS_cluster_2298692584_7170315064', 'joinedS_5', '491543745', 'cluster1302567586_149374901_305918149',
              'GS_cluster_506774249_5371110830', 'GS_cluster_1845506036_305930099', 'GS_cluster_1635701799_1635701803', 'GS_1713983096', 'GS_1713983087', 'joinedS_12']

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
            'boolean': ['6316129114_b1', '6316129114_b2', '6316129114_b3', '6316129114_b4', '6316129114_b9', '6316129114_b10', '6316129114_b11', '6316129114_b17', '6316129114_b18', '6316129114_b19', '6316129114_b20', '6316129114_b21'],
            'saturation': ['6316129114_s1', '6316129114_s2', '6316129114_s7', '6316129114_s8'],
            'numerical': ['6316129114_n1', '6316129114_n2', '6316129114_n3', '6316129114_n4', '6316129114_n13', '6316129114_n14', '6316129114_n15', '6316129114_n21', '6316129114_n22', '6316129114_n23', '6316129114_n24', '6316129114_n25'],
            'exit': []
        },
        2: {
            'boolean': ['6316129114_b5', '6316129114_b6', '6316129114_b7', '6316129114_b8', '6316129114_b9', '6316129114_b10', '6316129114_b11', '6316129114_b14', '6316129114_b15', '6316129114_b16', '6316129114_b19', '6316129114_b20', '6316129114_b21', '6316129114_b24', '6316129114_b25', '6316129114_b26'],
            'saturation': ['6316129114_s3', '6316129114_s4'],
            'numerical': ['6316129114_n5', '6316129114_n6', '6316129114_n7', '6316129114_n8', '6316129114_n9', '6316129114_n10', '6316129114_n10', '6316129114_n11', '6316129114_n12', '6316129114_n13', '6316129114_n14', '6316129114_n15', '6316129114_n18', '6316129114_n19', '6316129114_n20', '6316129114_n23', '6316129114_n24', '6316129114_n25', '6316129114_n30', '6316129114_n31', '6316129114_n32'],
            'exit': []
        },
        4: {
            'boolean': ['6316129114_b12', '6316129114_b13', '6316129114_b14', '6316129114_b15', '6316129114_b16', '6316129114_b22', '6316129114_b23', '6316129114_b24', '6316129114_b25', '6316129114_b26'],
            'saturation': ['6316129114_s5', '6316129114_s6', '6316129114_s9', '6316129114_s10'],
            'numerical': ['6316129114_n16', '6316129114_n17', '6316129114_n18', '6316129114_n19', '6316129114_n20', '6316129114_n26', '6316129114_n27', '6316129114_n28', '6316129114_n29', '6316129114_n30', '6316129114_n31', '6316129114_n32'],
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
            'numerical': ['cluster_1635701799_n1', 'cluster_1635701799_n2', 'cluster_1635701799_n3', 'cluster_1635701799_n4', 'cluster_1635701799_n5', 'cluster_1635701799_n6', 'cluster_1635701799_n7', 'cluster_1635701799_n8', 'cluster_1635701799_n12', 'cluster_1635701799_n13', 'cluster_1635701799_n14', 'cluster_1635701799_n15', 'cluster_1635701799_n16', 'cluster_1635701799_n17',],
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
            'boolean': ['6267747172_b1', '6267747172_b5', '6267747172_b8'],
            'saturation': ['6267747172_s1', '6267747172_s4', '6267747172_s7'],
            'numerical': ['6267747172_n1', '6267747172_n2', '6267747172_n8', '6267747172_n9', '6267747172_n10'],
            'exit': []
        },
        4: {
            'boolean': ['6267747172_b6', '6267747172_b7', '6267747172_b10', '6267747172_b11'],
            'saturation': ['6267747172_s5', '6267747172_s6', '6267747172_s9', '6267747172_s10'],
            'numerical': ['6267747172_n11', '6267747172_n12', '6267747172_n13', '6267747172_n14', '6267747172_n19', '6267747172_n20'],
            'exit': []
        },
        8: {
            'boolean': ['6267747172_b2', '6267747172_b3', '6267747172_b4', '6267747172_b9'],
            'saturation': ['6267747172_s2', '6267747172_s3', '6267747172_s8'],
            'numerical': ['6267747172_n3', '6267747172_n4', '6267747172_n5', '6267747172_n6', '6267747172_n7', '6267747172_n15', '6267747172_n16', '6267747172_n17', '6267747172_n18'],
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

    DETECTORS_cluster1299481290_252000834_4063893143 = {
        0: {
            'boolean': ['cluster_250889004_b3', 'cluster_250889004_b4'],
            'saturation': ['cluster_250889004_s3', 'cluster_250889004_b4'],
            'numerical': ['cluster_250889004_n3', 'cluster_250889004_n4', 'cluster_250889004_n5', 'cluster_250889004_n6'],
            'exit': []
        },
        2: {
            'boolean': ['cluster_250889004_b1', 'cluster_250889004_b2'],
            'saturation': ['cluster_250889004_s1', 'cluster_250889004_s2'],
            'numerical': ['cluster_250889004_n1', 'cluster_250889004_n2'],
            'exit': []
        }
    }

    DETECTORS_J02 = {
        0: {
            'boolean': ['J02_b1', 'J02_b2', 'J02_b3'],
            'saturation': ['J02_s1', 'J02_s2', 'J02_s3', 'J02_s4'],
            'numerical': ['J02_n1', 'J02_n2', 'J02_n3', 'J02_n4', 'J02_n5', 'J02_n6', 'J02_n7'],
            'exit': []
        },
        2: {
            'boolean': ['J02_b4', 'J02_b5'],
            'saturation': ['J02_s5', 'J02_s6'],
            'numerical': ['J02_n8', 'J02_n9'],
            'exit': []
        }
    }

    DETECTORS_J0333 = {
        0: {
            'boolean': ['J0333_b1', 'J0333_b2'],
            'saturation': ['J0333_s1', 'J0333_s2'],
            'numerical': ['J0333_n1', 'J0333_n2'],
            'exit': []
        },
        2: {
            'boolean': ['J0333_b3', 'J0333_b4', 'J0333_b5', 'J0333_b6'],
            'saturation': ['J0333_s3', 'J0333_s4', 'J0333_s5', 'J0333_s6'],
            'numerical': ['J0333_n3', 'J0333_n4', 'J0333_n5', 'J0333_n6'],
            'exit': []
        }
    }

    DETECTORS_GS_3075917655 = {
        0: {
            'boolean': ['3075917655_b1', '3075917655_b2'],
            'saturation': ['3075917655_s1', '3075917655_s2'],
            'numerical': ['3075917655_n1', '3075917655_n2'],
            'exit': []
        },
        2: {
            'boolean': ['3075917655_b3', '3075917655_b4'],
            'saturation': ['3075917655_s3', '3075917655_s4'],
            'numerical': ['3075917655_n3', '3075917655_n4', '3075917655_n5', '3075917655_n6'],
            'exit': []
        }
    }

    DETECTORS_joinedS_12 = {
        0: {
            'boolean': ['joinedS_12_b7', 'joinedS_12_b8', 'joinedS_12_b9', 'joinedS_12_b14', 'joinedS_12_b15', 'joinedS_12_b16'],
            'saturation': ['joinedS_12_s4', 'joinedS_12_s5'],
            'numerical': ['joinedS_12_n10', 'joinedS_12_n11', 'joinedS_12_n12', 'joinedS_12_n13', 'joinedS_12_n14', 'joinedS_12_n15', 'joinedS_12_n16', 'joinedS_12_n17', 'joinedS_12_n27', 'joinedS_12_n28', 'joinedS_12_n29', 'joinedS_12_n30'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_12_b1', 'joinedS_12_b2', 'joinedS_12_b3', 'joinedS_12_b4', 'joinedS_12_b5', 'joinedS_12_b6', 'joinedS_12_b10', 'joinedS_12_b11', 'joinedS_12_b12'],
            'saturation': ['joinedS_12_s1', 'joinedS_12_s2', 'joinedS_12_s3', 'joinedS_12_s6', 'joinedS_12_s7', 'joinedS_12_s8'],
            'numerical': ['joinedS_12_n1', 'joinedS_12_n2', 'joinedS_12_n3', 'joinedS_12_n4', 'joinedS_12_n5', 'joinedS_12_n6', 'joinedS_12_n7', 'joinedS_12_n8', 'joinedS_12_n9', 'joinedS_12_n18', 'joinedS_12_n19', 'joinedS_12_n20', 'joinedS_12_n21', 'joinedS_12_n22', 'joinedS_12_n23'],
            'exit': []
        },
        4: {
            'boolean': ['joinedS_12_b7', 'joinedS_12_b8', 'joinedS_12_b9', 'joinedS_12_b13', 'joinedS_12_b14', 'joinedS_12_b15', 'joinedS_12_b16'],
            'saturation': ['joinedS_12_s4', 'joinedS_12_s5', 'joinedS_12_s9', 'joinedS_12_s10'],
            'numerical': ['joinedS_12_n10', 'joinedS_12_n11', 'joinedS_12_n12', 'joinedS_12_n13', 'joinedS_12_n14', 'joinedS_12_n15', 'joinedS_12_n16', 'joinedS_12_n17', 'joinedS_12_n24', 'joinedS_12_n25', 'joinedS_12_n26', 'joinedS_12_n27', 'joinedS_12_n28', 'joinedS_12_n29', 'joinedS_12_n30'],
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
            'boolean': ['joinedS_5_b6', 'joinedS_5_b7', 'joinedS_5_b8', 'joinedS_5_b9', 'joinedS_5_b10', 'joinedS_5_b11', 'joinedS_5_b17', 'joinedS_5_b18', 'joinedS_5_b19', 'joinedS_5_b20'],
            'saturation': ['joinedS_5_s4', 'joinedS_5_s7', 'joinedS_5_s8', 'joinedS_5_s9', 'joinedS_5_s10'],
            'numerical': ['joinedS_5_n8', 'joinedS_5_n9', 'joinedS_5_n10', 'joinedS_5_n11', 'joinedS_5_n12', 'joinedS_5_n13', 'joinedS_5_n14', 'joinedS_5_n24', 'joinedS_5_n25', 'joinedS_5_n26', 'joinedS_5_n27', 'joinedS_5_n28', 'joinedS_5_n29', 'joinedS_5_n30', 'joinedS_5_n31', 'joinedS_5_n32', 'joinedS_5_n33', 'joinedS_5_n34', 'joinedS_5_n35'],
            'exit': []
        },
        2: {
            'boolean': ['joinedS_5_b5', 'joinedS_5_b8', 'joinedS_5_b9', 'joinedS_5_b10', 'joinedS_5_b11', 'joinedS_5_b17', 'joinedS_5_b18', 'joinedS_5_b19', 'joinedS_5_b20'],
            'saturation': ['joinedS_5_s3', 'joinedS_5_s7', 'joinedS_5_s8', 'joinedS_5_s9', 'joinedS_5_s10'],
            'numerical': ['joinedS_5_n5', 'joinedS_5_n6', 'joinedS_5_n7', 'joinedS_5_n11', 'joinedS_5_n12', 'joinedS_5_n13', 'joinedS_5_n14', 'joinedS_5_n24', 'joinedS_5_n25', 'joinedS_5_n26', 'joinedS_5_n27', 'joinedS_5_n28', 'joinedS_5_n29', 'joinedS_5_n30', 'joinedS_5_n31', 'joinedS_5_n32', 'joinedS_5_n33', 'joinedS_5_n34', 'joinedS_5_n35'],
            'exit': []
        },
        4: {
            'boolean': ['joinedS_5_b3', 'joinedS_5_b4', 'joinedS_5_b8', 'joinedS_5_b9', 'joinedS_5_b10', 'joinedS_5_b11', 'joinedS_5_b15', 'joinedS_5_b16', 'joinedS_5_b19', 'joinedS_5_b20'],
            'saturation': [],
            'numerical': ['joinedS_5_n3', 'joinedS_5_n4', 'joinedS_5_n11', 'joinedS_5_n12', 'joinedS_5_n13', 'joinedS_5_n14', 'joinedS_5_n22', 'joinedS_5_n23', 'joinedS_5_n34', 'joinedS_5_n35'],
            'exit': []
        },
        6: {
            'boolean': ['joinedS_5_b1', 'joinedS_5_b2', 'joinedS_5_b3', 'joinedS_5_b4', 'joinedS_5_b12', 'joinedS_5_b13', 'joinedS_5_b14', 'joinedS_5_b15', 'joinedS_5_b16'],
            'saturation': ['joinedS_5_s1', 'joinedS_5_s2', 'joinedS_5_s5', 'joinedS_5_s6'],
            'numerical': ['joinedS_5_n1', 'joinedS_5_n2', 'joinedS_5_n3', 'joinedS_5_n4', 'joinedS_5_n15', 'joinedS_5_n16', 'joinedS_5_n17', 'joinedS_5_n18', 'joinedS_5_n19', 'joinedS_5_n20', 'joinedS_5_n21', 'joinedS_5_n22', 'joinedS_5_n23'],
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

