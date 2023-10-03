import sys, os
from core.src.components import Network, Routes, DetectorBuilder
tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
sys.path.append(tools)
import traci

class TwoCrossroadsNetwork:


    ### Networks ###

    def double_crossroad_network(self, config):
        """
        Réseau routier représentant deux carrefours simples consécutifs
        :param config: Configuration du réseau routier
        :return: Retourne un objet Network représentant le réseau routier créé
        """
        net = Network()

        net.add_node(id='c1', x=0, y=0, type='traffic_light', tl='c1')
        net.add_node(id='c2', x=config['middle_len'], y=0, type='traffic_light', tl='c2')

        net.add_node(id='w', x=-config['w_e_len'], y=0)
        net.add_node(id='e', x=config['middle_len'] + config['default_len'], y=0)

        net.add_node(id='s1', x=0, y=-config['s1_n1_len'])
        net.add_node(id='n1', x=0, y=config['default_len'])

        net.add_node(id='s2', x=config['middle_len'], y=-config['s2_n2_len'])
        net.add_node(id='n2', x=config['middle_len'], y=config['default_len'])

        net.add_type(id='we', values={'numLanes': '1', 'speed': config['w_e_speed']})
        net.add_type(id='s1n1', values={'numLanes': '1', 'speed': config['s1_n1_speed']})
        net.add_type(id='s2n2', values={'numLanes': '1', 'speed': config['s2_n2_speed']})

        net.add_edge(id='edge_wc1', from_node='w', to_node='c1', type='we')
        net.add_edge(id='edge_c1c2', from_node='c1', to_node='c2', type='we')
        net.add_edge(id='edge_c2e', from_node='c2', to_node='e', type='we')
        net.add_edge(id='edge_s1c1', from_node='s1', to_node='c1', type='s1n1')
        net.add_edge(id='edge_c1n1', from_node='c1', to_node='n1', type='s1n1')
        net.add_edge(id='edge_s2c2', from_node='s2', to_node='c2', type='s2n2')
        net.add_edge(id='edge_c2n2', from_node='c2', to_node='n2', type='s2n2')

        net.add_connection(from_edge='edge_wc1', to_edge='edge_c1c2')
        net.add_connection(from_edge='edge_c1c2', to_edge='edge_c2e')
        net.add_connection(from_edge='edge_s1c1', to_edge='edge_c1n1')
        net.add_connection(from_edge='edge_s2c2', to_edge='edge_c2n2')

        net.add_traffic_light_program(id='c1', programID='c1',
                                      phases=[{'duration': config['w1_e1_green_time'], 'state': 'rG'},
                                              {'duration': config['default_yellow_time'], 'state': 'ry'},
                                              {'duration': config['s1_n1_green_time'], 'state': 'Gr'},
                                              {'duration': config['default_yellow_time'], 'state': 'yr'}])

        net.add_traffic_light_program(id='c2', programID='c2',
                                      phases=[{'duration': config['w2_e2_green_time'], 'state': 'rG'},
                                              {'duration': config['default_yellow_time'], 'state': 'ry'},
                                              {'duration': config['s2_n2_green_time'], 'state': 'Gr'},
                                              {'duration': config['default_yellow_time'], 'state': 'yr'}])

        return net


    def double_crossroad_fully_connected_network(self, config):
        """
        Réseau routier représentant deux carrefours consécutifs. Les véhicules peuvent arriver et ressortir des 6 entrées.
        :param config: Configuration du réseau routier
        :return: Retourne un objet Network représentant le réseau routier créé
        """
        net = Network()

        net.add_node(id='c1', x=0, y=0, type='traffic_light', tl='c1')
        net.add_node(id='c2', x=config['default_len'], y=0, type='traffic_light', tl='c2')

        net.add_node(id='w', x=-config['default_len'], y=0)
        net.add_node(id='e', x=config['default_len'] * 2, y=0)

        net.add_node(id='s1', x=0, y=-config['default_len'])
        net.add_node(id='n1', x=0, y=config['default_len'])

        net.add_node(id='s2', x=config['default_len'], y=-config['default_len'])
        net.add_node(id='n2', x=config['default_len'], y=config['default_len'])

        net.add_type(id='default', values={'numLanes': '1', 'speed': config['default_speed']})

        # Edge entrant sur le premier carrefour
        net.add_edge(id='edge_wc1', from_node='w', to_node='c1', type='default')
        net.add_edge(id='edge_s1c1', from_node='s1', to_node='c1', type='default')
        net.add_edge(id='edge_n1c1', from_node='n1', to_node='c1', type='default')
        net.add_edge(id='edge_c2c1', from_node='c2', to_node='c1', type='default')
        # Edge sortant du premier carrefour
        net.add_edge(id='edge_c1c2', from_node='c1', to_node='c2', type='default')
        net.add_edge(id='edge_c1n1', from_node='c1', to_node='n1', type='default')
        net.add_edge(id='edge_c1w', from_node='c1', to_node='w', type='default')
        net.add_edge(id='edge_c1s1', from_node='c1', to_node='s1', type='default')
        # Edge ebtrant sur le second carrefour
        net.add_edge(id='edge_ec2', from_node='e', to_node='c2', type='default')
        net.add_edge(id='edge_s2c2', from_node='s2', to_node='c2', type='default')
        net.add_edge(id='edge_n2c2', from_node='n2', to_node='c2', type='default')
        # Edge sortant du second carrefour
        net.add_edge(id='edge_c2e', from_node='c2', to_node='e', type='default')
        net.add_edge(id='edge_c2n2', from_node='c2', to_node='n2', type='default')
        net.add_edge(id='edge_c2s2', from_node='c2', to_node='s2', type='default')

        # Connexion des arêtes du premier carrefour
        net.add_connection(from_edge='edge_wc1', to_edge='edge_c1c2')
        net.add_connection(from_edge='edge_wc1', to_edge='edge_c1n1')
        net.add_connection(from_edge='edge_wc1', to_edge='edge_c1s1')
        net.add_connection(from_edge='edge_n1c1', to_edge='edge_c1s1')
        net.add_connection(from_edge='edge_n1c1', to_edge='edge_c1c2')
        net.add_connection(from_edge='edge_n1c1', to_edge='edge_c1w')
        net.add_connection(from_edge='edge_s1c1', to_edge='edge_c1c2')
        net.add_connection(from_edge='edge_s1c1', to_edge='edge_c1n1')
        net.add_connection(from_edge='edge_s1c1', to_edge='edge_c1w')
        net.add_connection(from_edge='edge_c2c1', to_edge='edge_c1s1')
        net.add_connection(from_edge='edge_c2c1', to_edge='edge_c1n1')
        net.add_connection(from_edge='edge_c2c1', to_edge='edge_c1w')
        # Connexion des arêtes du second carrefour
        net.add_connection(from_edge='edge_c1c2', to_edge='edge_c2e')
        net.add_connection(from_edge='edge_c1c2', to_edge='edge_c2n2')
        net.add_connection(from_edge='edge_c1c2', to_edge='edge_c2s2')
        net.add_connection(from_edge='edge_s2c2', to_edge='edge_c2n2')
        net.add_connection(from_edge='edge_s2c2', to_edge='edge_c2e')
        net.add_connection(from_edge='edge_s2c2', to_edge='edge_c2c1')
        net.add_connection(from_edge='edge_n2c2', to_edge='edge_c2e')
        net.add_connection(from_edge='edge_n2c2', to_edge='edge_c2s2')
        net.add_connection(from_edge='edge_n2c2', to_edge='edge_c2c1')
        net.add_connection(from_edge='edge_ec2', to_edge='edge_c2n2')
        net.add_connection(from_edge='edge_ec2', to_edge='edge_c2s2')
        net.add_connection(from_edge='edge_ec2', to_edge='edge_c2c1')

        net.add_traffic_light_program(id='c1', programID='c1',
                                      phases=[{'duration': config['default_green_time'], 'state': 'rrrGGGrrrGGG'},
                                              {'duration': config['default_yellow_time'], 'state': 'rrryyyrrryyy'},
                                              {'duration': config['default_green_time'], 'state': 'GGGrrrGGGrrr'},
                                              {'duration': config['default_yellow_time'], 'state': 'yyyrrryyyrrr'}])

        net.add_traffic_light_program(id='c2', programID='c2',
                                      phases=[{'duration': config['default_green_time'], 'state': 'GGGrrrGGGrrr'},
                                              {'duration': config['default_yellow_time'], 'state': 'yyyrrryyyrrr'},
                                              {'duration': config['default_green_time'], 'state': 'rrrGGGrrrGGG'},
                                              {'duration': config['default_yellow_time'], 'state': 'rrryyyrrryyy'}])

        return net



    ### Routes ###

    def double_crossroad_routes(self, config):
        """
        Routes pour deux carrefours simples consécutifs
        :param config: Configuration du réseau routier
        :return: Retourne un objet Routes représentant les routes du réseau routier créé
        """
        routes = Routes()

        routes.add_vType(id='car0')

        routes.add_route(id='route_s1n1', type='car0', from_edge='edge_s1c1', to_edge='edge_c1n1')
        routes.add_route(id='route_s2n2', type='car0', from_edge='edge_s2c2', to_edge='edge_c2n2')

        routes.add_flow(id='flow_we', from_edge='edge_wc1', to_edge='edge_c2e', end=config['stop_generation_time'],
                        vehsPerHour=config['w_e_flow'], vType='car0')
        routes.add_flow(id='flow_s1n1', route='route_s1n1', end=config['stop_generation_time'],
                        vehsPerHour=config['s1_n1_flow'], vType='car0')
        routes.add_flow(id='flow_s2n2', route='route_s2n2', end=config['stop_generation_time'],
                        vehsPerHour=config['s2_n2_flow'], vType='car0')
        return routes


    def double_crossroad_fully_connected_multi_flow(self, config):
        """
        Réseau en carefour simple totalement connecté, dont les flux vont varier au cours de la simulation.
        Il faut placer dans config['coeff_matrix'] une matrice de taille (12, x), avec x le nombre de modifications de flows
        Cette matrice contient des coefficients compris entre 0 exclus et 1, qui seront multiplié par les nombre de voitures totaux du vecteur de charges
        Il faut également placer un vecteur de charge dans config['load_vector'], de taille x, qui contient le nombre de voiture total du réseau
        Enfin, il faut définir config['nb_ticks'] qui définit le nombre de ticks de chaque période.

        :param config: Configuration du réseau routier
        :return: Retourne un objet Routes représentant les routes du réseau routier créé
        """

        matrice_coeffs = config['coeff_matrix']
        vecteur_charge = config['load_vector']
        nb_ticks = config['nb_ticks']

        routes = Routes()

        routes.add_vType(id='car0')

        for i in range(matrice_coeffs.shape[1]):
            vecteur_coeffs = matrice_coeffs[:, i]
            flows = vecteur_coeffs * vecteur_charge[i]
            flow_start = nb_ticks * i
            flow_end = nb_ticks * (i + 1)

            # Flux venant du nord 1
            routes.add_flow(id=f'{i}_flow_n1n2', from_edge='edge_n1c1', to_edge='edge_c2n2', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[0], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_n1e', from_edge='edge_n1c1', to_edge='edge_c2e', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[1], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_n1s2', from_edge='edge_n1c1', to_edge='edge_c2s2', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[2], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_n1s1', from_edge='edge_n1c1', to_edge='edge_c1s1', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[3], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_n1w', from_edge='edge_n1c1', to_edge='edge_c1w', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[4], vType='car0', distribution='binomial')
            # Flux venant du nord 2
            routes.add_flow(id=f'{i}_flow_n2e', from_edge='edge_n2c2', to_edge='edge_c2e', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[5], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_n2s2', from_edge='edge_n2c2', to_edge='edge_c2s2', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[6], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_n2s1', from_edge='edge_n2c2', to_edge='edge_c1s1', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[7], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_n2w', from_edge='edge_n2c2', to_edge='edge_c1w', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[8], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_n2n1', from_edge='edge_n2c2', to_edge='edge_c1n1', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[9], vType='car0', distribution='binomial')
            # Flux venant de l'ouest
            routes.add_flow(id=f'{i}_flow_es2', from_edge='edge_ec2', to_edge='edge_c2s2', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[10], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_es1', from_edge='edge_ec2', to_edge='edge_c1s1', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[11], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_ew', from_edge='edge_ec2', to_edge='edge_c1w', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[12], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_en1', from_edge='edge_ec2', to_edge='edge_c1n1', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[13], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_en2', from_edge='edge_ec2', to_edge='edge_c2n2', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[14], vType='car0', distribution='binomial')
            # Flux venant du sud 2
            routes.add_flow(id=f'{i}_flow_s2s1', from_edge='edge_s2c2', to_edge='edge_c1s1', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[15], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_s2w', from_edge='edge_s2c2', to_edge='edge_c1w', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[16], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_s2n1', from_edge='edge_s2c2', to_edge='edge_c1n1', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[17], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_s2n2', from_edge='edge_s2c2', to_edge='edge_c2n2', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[18], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_s2e', from_edge='edge_s2c2', to_edge='edge_c2e', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[19], vType='car0', distribution='binomial')
            # Flux venant du sud 1
            routes.add_flow(id=f'{i}_flow_s1w', from_edge='edge_s1c1', to_edge='edge_c1w', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[20], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_s1n1', from_edge='edge_s1c1', to_edge='edge_c1n1', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[21], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_s1n2', from_edge='edge_s1c1', to_edge='edge_c2n2', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[22], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_s1e', from_edge='edge_s1c1', to_edge='edge_c2e', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[23], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_s1s2', from_edge='edge_s1c1', to_edge='edge_c2s2', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[24], vType='car0', distribution='binomial')
            # Flux venant de l'est
            routes.add_flow(id=f'{i}_flow_wn1', from_edge='edge_wc1', to_edge='edge_c1n1', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[25], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_wn2', from_edge='edge_wc1', to_edge='edge_c2n2', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[26], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_we', from_edge='edge_wc1', to_edge='edge_c2e', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[27], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_ws2', from_edge='edge_wc1', to_edge='edge_c2s2', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[28], vType='car0', distribution='binomial')
            routes.add_flow(id=f'{i}_flow_ws1', from_edge='edge_wc1', to_edge='edge_c1s1', begin=flow_start, end=flow_end,
                            vehsPerHour=flows[29], vType='car0', distribution='binomial')

        return routes



    ### Detectors ###
    def no_detectors(self, config):
        """
        Fonction qui n'ajoute aucun détecteurs à un réseau.

        :param config: Configuration du réseau routier
        :return: Retourne un objet Detector vide
        """

        return DetectorBuilder()

    def detecteurs_numeriques_double_carrefour(self, config):
        """
        Fonction qui ajoute des détecteurs entre deux carrefours d'un réseau entre deux carrefours
        de sorte à simuler une communication entre les deux.

        :param config: Configuration du réseau routier
        :return: Retourne un objet Detector représentant les détecteurs mentionnés
        """

        detectors = DetectorBuilder()

        # Premier carrefour
        detectors.add_laneAreaDetector(id="wc1", lane="edge_wc1_0")
        detectors.add_laneAreaDetector(id="s1c1", lane="edge_s1c1_0")
        detectors.add_laneAreaDetector(id="n1c1", lane="edge_n1c1_0")
        detectors.add_laneAreaDetector(id="c2c1", lane="edge_c2c1_0")
        # Deuxième carrefour
        detectors.add_laneAreaDetector(id="c1c2", lane="edge_c1c2_0")
        detectors.add_laneAreaDetector(id="ec2", lane="edge_ec2_0")
        detectors.add_laneAreaDetector(id="n2c2", lane="edge_n2c2_0")
        detectors.add_laneAreaDetector(id="s2c2", lane="edge_s2c2_0")

        return detectors

    def detecteurs_booleens_double_carrefour(self, config):
        """
        Fonction qui ajoute des détecteurs à deux carrefours d'un réseau à deux carrefours.
        Ces détecteurs ne renvoient que la présence d'un véhicule prêt à passer au feu.

        :param config: Configuration du réseau routier
        :return: Retourne un objet Additional représentant les détecteurs mentionnés
        """

        detectors = DetectorBuilder()

        detector_length = config["boolean_detector_length"]
        lane_len = config['default_len']

        # Premier carrefour
        detectors.add_laneAreaDetector(id="wc1", lane="edge_wc1_0", pos=(lane_len - detector_length - 7.2))
        detectors.add_laneAreaDetector(id="s1c1", lane="edge_s1c1_0", pos=(lane_len - detector_length - 7.2))
        detectors.add_laneAreaDetector(id="n1c1", lane="edge_n1c1_0", pos=(lane_len - detector_length - 7.2))
        detectors.add_laneAreaDetector(id="c2c1", lane="edge_c2c1_0", pos=(lane_len - detector_length - 7.2 * 2))
        # Deuxième carrefour
        detectors.add_laneAreaDetector(id="c1c2", lane="edge_c1c2_0", pos=(lane_len - detector_length - 7.2 * 2))
        detectors.add_laneAreaDetector(id="ec2", lane="edge_ec2_0", pos=(lane_len - detector_length - 7.2))
        detectors.add_laneAreaDetector(id="n2c2", lane="edge_n2c2_0", pos=(lane_len - detector_length - 7.2))
        detectors.add_laneAreaDetector(id="s2c2", lane="edge_s2c2_0", pos=(lane_len - detector_length - 7.2))

        return detectors




    ### Strategies ###

    def feux_a_detection_booleenne_double_carrefour(self, config):
        """
        Sur chaque voie arrivant devant un feu, il existe un détecteur qui a pour charge d'évaluer si un véhicule est présent dans les
        7 derniers mêtres avant le feu. S'il y en a un, et que le feu est vert, alors on garde le feu vert. S'il n'y en a pas, et que le feu
        est vert, et qu'il y a un véhicule en attente sur une voie perpendiculaire, alors on change de feu. L'opération ne peut pas se faire
        avant "min_duration_tl" step depuis le dernier changement, et se fait obligatoirement après "max_duration_tl" step depuis le dernier
        changement.

        Ne fonctionne que pour un réseau à deux carrefours.

        Paramètres à mettre dans la config :
        - "min_duration_tl" (int) : Le nombre de pas minimum de durée d'un feu
        - "max_duration_tl" (int) : Le nombre de pas maximum de durée d'un feu
        """

        step = 0
        cooldownStep1 = 0  # Nombre de pas depuis la dernière actualisation du feu 1
        cooldownStep2 = 0  # Nombre de pas depuis la dernière actualisation du feu 2

        FEU_1_VERT_HORIZONTAL = 0
        FEU_1_VERT_VERTICAL = 2
        FEU_2_VERT_HORIZONTAL = 2
        FEU_2_VERT_VERTICAL = 0

        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]

        while step < config['simulation_duration']:
            traci.simulationStep()

            # Premier carrefour
            if cooldownStep1 > min_duration_tl:
                if traci.trafficlight.getPhase("c1") == FEU_1_VERT_HORIZONTAL:
                    quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(
                        "n1c1") >= 1 or traci.lanearea.getLastStepVehicleNumber("s1c1") >= 1)
                    autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(
                        "wc1") == 0 and traci.lanearea.getLastStepVehicleNumber("c2c1") == 0)
                    if (quelqun_en_attente and autre_voie_vide) or cooldownStep1 > max_duration_tl:
                        traci.trafficlight.setPhase("c1", FEU_1_VERT_HORIZONTAL + 1)  # Passage au orange
                        cooldownStep1 = 0

                elif traci.trafficlight.getPhase("c1") == FEU_1_VERT_VERTICAL:
                    quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(
                        "wc1") >= 1 or traci.lanearea.getLastStepVehicleNumber("c2c1") >= 1)
                    autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(
                        "n1c1") == 0 and traci.lanearea.getLastStepVehicleNumber("s1c1") == 0)
                    if (quelqun_en_attente and autre_voie_vide) or cooldownStep1 > max_duration_tl:
                        traci.trafficlight.setPhase("c1", FEU_1_VERT_VERTICAL + 1)
                        cooldownStep1 = 0

            # Deuxième carrefour
            if cooldownStep2 > min_duration_tl:
                if traci.trafficlight.getPhase("c2") == FEU_2_VERT_HORIZONTAL:
                    quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(
                        "n2c2") >= 1 or traci.lanearea.getLastStepVehicleNumber("s2c2") >= 1)
                    autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(
                        "ec2") == 0 and traci.lanearea.getLastStepVehicleNumber("c1c2") == 0)
                    if (quelqun_en_attente and autre_voie_vide) or cooldownStep2 > max_duration_tl:
                        traci.trafficlight.setPhase("c2", FEU_2_VERT_HORIZONTAL + 1)
                        cooldownStep2 = 0

                elif traci.trafficlight.getPhase("c2") == FEU_2_VERT_VERTICAL:
                    quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(
                        "ec2") >= 1 or traci.lanearea.getLastStepVehicleNumber("c1c2") >= 1)
                    autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(
                        "n2c2") == 0 and traci.lanearea.getLastStepVehicleNumber("s2c2") == 0)
                    if (quelqun_en_attente and autre_voie_vide) or cooldownStep2 > max_duration_tl:
                        traci.trafficlight.setPhase("c2",
                                                    FEU_2_VERT_VERTICAL + 1)  # WE orange, SN red, then normal transition to phase 0
                        cooldownStep2 = 0

            step += 1
            cooldownStep1 += 1
            cooldownStep2 += 1

    def feux_a_seuils_communicants_avec_anticipation_double_carrefour(self, config):
        """
        On détermine un seuil pour chaque feu du réseau. Lorsque le feu est rouge pour une voie du feu, et que le nombre de véhicules en attente
        est supérieur ou égal au seuil, alors le feu passe au vert. Le feu doit rester vert un minimum de temps avant de pouvoir à nouveau changer.

        Paramètres à mettre dans la config :
        - "min_duration_tl" (int) : Le nombre de pas minimum de durée d'un feu
        - "max_duration_tl" (int) : Le nombre de pas maximum de durée d'un feu
        - "seuil_vehicules" (int) : Le nombre de véhicules en attente nécessaires pour déclencher un changement de feu. (sur une voie uniquement)
        """

        step = 0
        cooldownStep1 = 0  # Nombre de pas depuis la dernière actualisation du feu 1
        cooldownStep2 = 0  # Nombre de pas depuis la dernière actualisation du feu 2

        FEU_1_ROUGE_HORIZONTAL = 2
        FEU_1_ROUGE_VERTICAL = 0
        FEU_2_ROUGE_HORIZONTAL = 0
        FEU_2_ROUGE_VERTICAL = 2

        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        nb_vehicules = config["seuil_vehicules"]

        while step < config['simulation_duration']:
            traci.simulationStep()

            # Premier carrefour
            if cooldownStep1 > min_duration_tl:
                if traci.trafficlight.getPhase("c1") == FEU_1_ROUGE_HORIZONTAL:
                    if traci.lanearea.getLastStepVehicleNumber(
                            "wc1") >= nb_vehicules or traci.lanearea.getLastStepVehicleNumber(
                            "c2c1") >= nb_vehicules or cooldownStep1 > max_duration_tl:
                        traci.trafficlight.setPhase("c1", FEU_1_ROUGE_HORIZONTAL + 1)  # Passage au orange
                        cooldownStep1 = 0

                elif traci.trafficlight.getPhase("c1") == FEU_1_ROUGE_VERTICAL:
                    if traci.lanearea.getLastStepVehicleNumber(
                            "s1c1") >= nb_vehicules or traci.lanearea.getLastStepVehicleNumber(
                            "n1c1") >= nb_vehicules or cooldownStep1 > max_duration_tl:
                        traci.trafficlight.setPhase("c1", FEU_1_ROUGE_VERTICAL + 1)
                        cooldownStep1 = 0

            # Deuxième carrefour
            if cooldownStep2 > min_duration_tl:
                if traci.trafficlight.getPhase("c2") == FEU_2_ROUGE_HORIZONTAL:
                    if traci.lanearea.getLastStepVehicleNumber(
                            "c1c2") >= nb_vehicules or traci.lanearea.getLastStepVehicleNumber(
                            "ec2") >= nb_vehicules or cooldownStep2 > max_duration_tl:
                        traci.trafficlight.setPhase("c2", FEU_2_ROUGE_HORIZONTAL + 1)
                        cooldownStep2 = 0

                elif traci.trafficlight.getPhase("c2") == FEU_2_ROUGE_VERTICAL:
                    if traci.lanearea.getLastStepVehicleNumber(
                            "s2c2") >= nb_vehicules or traci.lanearea.getLastStepVehicleNumber(
                            "n2c2") >= nb_vehicules or cooldownStep2 > max_duration_tl:
                        traci.trafficlight.setPhase("c2",
                                                    FEU_2_ROUGE_VERTICAL + 1)  # WE orange, SN red, then normal transition to phase 0
                        cooldownStep2 = 0

            step += 1
            cooldownStep1 += 1
            cooldownStep2 += 1

    def feux_a_seuils_communicants_sans_anticipation_double_carrefour(self, config):
        """
        On détermine un seuil pour chaque feu du réseau. Lorsque le feu est rouge pour une voie du feu, et que le nombre de véhicules en attente et en course
        est supérieur ou égal au seuil, alors le feu passe au vert. Le feu doit rester vert un minimum de temps avant de pouvoir à nouveau changer.

        Paramètres à mettre dans la config :
        - "min_duration_tl" (int) : Le nombre de pas minimum de durée d'un feu
        - "max_duration_tl" (int) : Le nombre de pas maximum de durée d'un feu
        - "seuil_vehicules" (int) : Le nombre de véhicules en attente nécessaires pour déclencher un changement de feu. (sur une voie uniquement)
        """

        step = 0
        cooldownStep1 = 0  # Nombre de pas depuis la dernière actualisation du feu 1
        cooldownStep2 = 0  # Nombre de pas depuis la dernière actualisation du feu 2

        FEU_1_ROUGE_HORIZONTAL = 2
        FEU_1_ROUGE_VERTICAL = 0
        FEU_2_ROUGE_HORIZONTAL = 0
        FEU_2_ROUGE_VERTICAL = 2

        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        nb_vehicules = config["seuil_vehicules"]

        while step < config['simulation_duration']:
            traci.simulationStep()

            # Premier carrefour
            if cooldownStep1 > min_duration_tl:
                if traci.trafficlight.getPhase("c1") == FEU_1_ROUGE_HORIZONTAL:
                    if traci.lanearea.getJamLengthVehicle("wc1") >= nb_vehicules or traci.lanearea.getJamLengthVehicle(
                            "c2c1") >= nb_vehicules or cooldownStep1 > max_duration_tl:
                        traci.trafficlight.setPhase("c1", FEU_1_ROUGE_HORIZONTAL + 1)  # Passage au orange
                        cooldownStep1 = 0

                elif traci.trafficlight.getPhase("c1") == FEU_1_ROUGE_VERTICAL:
                    if traci.lanearea.getJamLengthVehicle("s1c1") >= nb_vehicules or traci.lanearea.getJamLengthVehicle(
                            "n1c1") >= nb_vehicules or cooldownStep1 > max_duration_tl:
                        traci.trafficlight.setPhase("c1", FEU_1_ROUGE_VERTICAL + 1)
                        cooldownStep1 = 0

            # Deuxième carrefour
            if cooldownStep2 > min_duration_tl:
                if traci.trafficlight.getPhase("c2") == FEU_2_ROUGE_HORIZONTAL:
                    if traci.lanearea.getJamLengthVehicle("c1c2") >= nb_vehicules or traci.lanearea.getJamLengthVehicle(
                            "ec2") >= nb_vehicules or cooldownStep2 > max_duration_tl:
                        traci.trafficlight.setPhase("c2", FEU_2_ROUGE_HORIZONTAL + 1)
                        cooldownStep2 = 0

                elif traci.trafficlight.getPhase("c2") == FEU_2_ROUGE_VERTICAL:
                    if traci.lanearea.getJamLengthVehicle("s2c2") >= nb_vehicules or traci.lanearea.getJamLengthVehicle(
                            "n2c2") >= nb_vehicules or cooldownStep2 > max_duration_tl:
                        traci.trafficlight.setPhase("c2",
                                                    FEU_2_ROUGE_VERTICAL + 1)  # WE orange, SN red, then normal transition to phase 0
                        cooldownStep2 = 0

            step += 1
            cooldownStep1 += 1
            cooldownStep2 += 1