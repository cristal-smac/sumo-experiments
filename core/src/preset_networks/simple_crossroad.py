import sys, os
from core.src.components import Network, Routes, Additional
tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
sys.path.append(tools)
import traci

class OneCrossroadNetwork:

    ### Networks ###

    def simple_crossroad_network(self, config):
        """
        Réseau routier représentant un carrefour
        :param config: Configuration du réseau routier
        :return: Retourne un objet Network représentant le réseau routier créé
        """

        # Instanciation du réseau routier
        net = Network()

        # Création des noeuds
        net.add_node(id='c', x=0, y=0, type='traffic_light', tl='c')    # Noeud central, croisement
        net.add_node(id='w', x=-config['w_e_len'], y=0)        # Noeud de l'entrée de la route à l'ouest
        net.add_node(id='e', x=config['default_len'], y=0)     # Noeud de sortie de la route à l'est
        net.add_node(id='s', x=0, y=-config['s_n_len'])        # Noeud de l'entrée de la route au sud
        net.add_node(id='n', x=0, y=config['default_len'])     # Noeud de sortie de la route au nord

        # Création des types de voies
        net.add_type(id='we', values={'numLanes': '1', 'speed': config['w_e_speed']})
        net.add_type(id='sn', values={'numLanes': '1', 'speed': config['s_n_speed']})

        # Création des arêtes
        net.add_edge(id='edge_wc', from_node='w', to_node='c', type='we')
        net.add_edge(id='edge_ce', from_node='c', to_node='e', type='we')
        net.add_edge(id='edge_sc', from_node='s', to_node='c', type='sn')
        net.add_edge(id='edge_cn', from_node='c', to_node='n', type='sn')

        # Création des connexions entre les arêtes
        net.add_connection(from_edge='edge_wc', to_edge='edge_ce')
        net.add_connection(from_edge='edge_sc', to_edge='edge_cn')

        # Création des feux routiers
        net.add_traffic_light_program(id='c', programID='c', phases=[{'duration': config['w_e_green_time'], 'state': 'rG'},
                                                                     {'duration': config['default_yellow_time'], 'state': 'ry'},
                                                                     {'duration': config['s_n_green_time'], 'state': 'Gr'},
                                                                     {'duration': config['default_yellow_time'], 'state': 'yr'}])

        return net


    def simple_crossroad_turn_network(self, config):
        """
        Réseau routier représentant un carrefour avec possibilité de tourner vers l'ouest ou au sud.
        :param config: Configuration du réseau routier
        :return: Retourne un objet Network représentant le réseau routier créé
        """
        net = Network()

        net.add_node(id='c', x=0, y=0, type='traffic_light', tl='c')    # Noeud central, croisement
        net.add_node(id='w', x=-config['w_e_len'], y=0)        # Noeud de l'entrée de la route à l'ouest
        net.add_node(id='e', x=config['default_len'], y=0)     # Noeud de sortie de la route à l'est
        net.add_node(id='s', x=0, y=-config['s_n_len'])        # Noeud de l'entrée de la route au sud
        net.add_node(id='n', x=0, y=config['default_len'])     # Noeud de sortie de la route au nord

        net.add_type(id='we', values={'numLanes': '1', 'speed': config['w_e_speed']})
        net.add_type(id='sn', values={'numLanes': '1', 'speed': config['s_n_speed']})

        net.add_edge(id='edge_wc', from_node='w', to_node='c', type='we')
        net.add_edge(id='edge_ce', from_node='c', to_node='e', type='we')
        net.add_edge(id='edge_sc', from_node='s', to_node='c', type='sn')
        net.add_edge(id='edge_cn', from_node='c', to_node='n', type='sn')

        net.add_edge(id='edge_cw', from_node='c', to_node='w', type='we')
        net.add_edge(id='edge_cs', from_node='c', to_node='s', type='sn')

        net.add_connection(from_edge='edge_wc', to_edge='edge_ce')
        net.add_connection(from_edge='edge_sc', to_edge='edge_cn')
        net.add_connection(from_edge='edge_sc', to_edge='edge_cw')
        net.add_connection(from_edge='edge_wc', to_edge='edge_cs')

        net.add_traffic_light_program(id='c', programID='c', phases=[{'duration': config['w_e_green_time'], 'state': 'rrGG'},
                                                                     {'duration': config['default_yellow_time'], 'state': 'rryy'},
                                                                     {'duration': config['s_n_green_time'], 'state': 'GGrr'},
                                                                     {'duration': config['default_yellow_time'], 'state': 'yyrr'}])


        return net


    def simple_crossroad_fully_connected_network(self, config):
        """
        Réseau routier représentant un carrefour
        Les véhciules peuvent arriver et repartir de chacun des voies
        :param config: Configuration du réseau routier
        :return: Retourne un objet Network représentant le réseau routier créé
        """

        # Instanciation du réseau routier
        net = Network()

        # Création des noeuds
        net.add_node(id='c', x=0, y=0, type='traffic_light', tl='c')    # Noeud central, croisement
        net.add_node(id='w', x=-config['default_len'], y=0)        # Noeud ouest
        net.add_node(id='e', x=config['default_len'], y=0)     # Noeud est
        net.add_node(id='s', x=0, y=-config['default_len'])        # Noeud sud
        net.add_node(id='n', x=0, y=config['default_len'])     # Noeud nord

        # Création des types de voies
        net.add_type(id='default', values={'numLanes': '1', 'speed': config['default_speed']})

        # Création des arêtes vers le centre
        net.add_edge(id='edge_wc', from_node='w', to_node='c', type='default')
        net.add_edge(id='edge_sc', from_node='s', to_node='c', type='default')
        net.add_edge(id='edge_ec', from_node='e', to_node='c', type='default')
        net.add_edge(id='edge_nc', from_node='n', to_node='c', type='default')

        # Création des arêtes vers la sortie
        net.add_edge(id='edge_ce', from_node='c', to_node='e', type='default')
        net.add_edge(id='edge_cn', from_node='c', to_node='n', type='default')
        net.add_edge(id='edge_cw', from_node='c', to_node='w', type='default')
        net.add_edge(id='edge_cs', from_node='c', to_node='s', type='default')


        # Création des connexions entre les arêtes
        # Venant de l'ouest
        net.add_connection(from_edge='edge_wc', to_edge='edge_ce')
        net.add_connection(from_edge='edge_wc', to_edge='edge_cs')
        net.add_connection(from_edge='edge_wc', to_edge='edge_cn')
        # Venant du nord
        net.add_connection(from_edge='edge_nc', to_edge='edge_cs')
        net.add_connection(from_edge='edge_nc', to_edge='edge_ce')
        net.add_connection(from_edge='edge_nc', to_edge='edge_cw')
        # Venant de l'est
        net.add_connection(from_edge='edge_ec', to_edge='edge_cs')
        net.add_connection(from_edge='edge_ec', to_edge='edge_cn')
        net.add_connection(from_edge='edge_ec', to_edge='edge_cw')
        # Venant du sud
        net.add_connection(from_edge='edge_sc', to_edge='edge_cn')
        net.add_connection(from_edge='edge_sc', to_edge='edge_ce')
        net.add_connection(from_edge='edge_sc', to_edge='edge_cw')

        # Création des feux routiers
        net.add_traffic_light_program(id='c', programID='c', phases=[{'duration': config['default_green_time'], 'state': 'rrrGGGrrrGGG'},
                                                                     {'duration': config['default_yellow_time'], 'state': 'rrryyyrrryyy'},
                                                                     {'duration': config['default_green_time'], 'state': 'GGGrrrGGGrrr'},
                                                                     {'duration': config['default_yellow_time'], 'state': 'yyyrrryyyrrr'}])

        return net



    ### Routes ###

    def simple_crossroad_routes(self, config):
        """
        Routes pour un carrefour simple
        :param config: Configuration du réseau routier
        :return: Retourne un objet Routes représentant les routes du réseau routier créé
        """
        # Instanciation des routes
        routes = Routes()

        # Création du type de véhicule
        routes.add_vType(id='car0')

        # Création des routes
        routes.add_route(id='route_we', type='car0', from_edge='edge_wc', to_edge='edge_ce')
        routes.add_route(id='route_sn', type='car0', from_edge='edge_sc', to_edge='edge_cn')

        # Création des flux
        routes.add_flow(id='flow_we', route='route_we', end=config['stop_generation_time'], vehsPerHour=config['w_e_flow'], vType='car0')
        routes.add_flow(id='flow_sn', route='route_sn', end=config['stop_generation_time'], vehsPerHour=config['s_n_flow'], vType='car0')

        return routes



    def simple_crossroad_turn_routes(self, config):
        """
        Routes pour un carrefour avec possibilité de tourner vers l'ouest ou au sud.
        :param config: Configuration du réseau routier
        :return: Retourne un objet Routes représentant les routes du réseau routier créé
        """
        routes = Routes()

        routes.add_vType(id='car0')

        routes.add_flow(id='flow_we_straight', from_edge='edge_wc', to_edge='edge_ce', end=config['stop_generation_time'], vehsPerHour=config['w_e_straight_flow'], vType='car0')
        routes.add_flow(id='flow_we_right', from_edge='edge_wc', to_edge='edge_cs', end=config['stop_generation_time'], vehsPerHour=config['w_e_right_flow'], vType='car0')
        routes.add_flow(id='flow_sn_straight', from_edge='edge_sc', to_edge='edge_cn', end=config['stop_generation_time'], vehsPerHour=config['s_n_straight_flow'], vType='car0')
        routes.add_flow(id='flow_sn_left', from_edge='edge_sc', to_edge='edge_cw', end=config['stop_generation_time'], vehsPerHour=config['s_n_left_flow'], vType='car0')

        return routes


    def simple_crossroad_fully_connected_routes(self, config):
        """
        Routes pour un carrefour complètement connecté
        :param config: Configuration du réseau routier
        :return: Retourne un objet Routes représentant les routes du réseau routier créé
        """
        routes = Routes()

        routes.add_vType(id='car0')

        # Flux venant du nord
        routes.add_flow(id='flow_ns', from_edge='edge_nc', to_edge='edge_cs', end=config['simulation_duration'], vehsPerHour=config['default_flow'], vType='car0')
        routes.add_flow(id='flow_ne', from_edge='edge_nc', to_edge='edge_ce', end=config['simulation_duration'], vehsPerHour=config['default_flow'], vType='car0')
        routes.add_flow(id='flow_nw', from_edge='edge_nc', to_edge='edge_cw', end=config['simulation_duration'], vehsPerHour=config['default_flow'], vType='car0')
        # Flux venant de l'est
        routes.add_flow(id='flow_es', from_edge='edge_ec', to_edge='edge_cs', end=config['simulation_duration'], vehsPerHour=config['default_flow'], vType='car0')
        routes.add_flow(id='flow_en', from_edge='edge_ec', to_edge='edge_cn', end=config['simulation_duration'], vehsPerHour=config['default_flow'], vType='car0')
        routes.add_flow(id='flow_ew', from_edge='edge_ec', to_edge='edge_cw', end=config['simulation_duration'], vehsPerHour=config['default_flow'], vType='car0')
        # Flux venant du sud
        routes.add_flow(id='flow_se', from_edge='edge_sc', to_edge='edge_ce', end=config['simulation_duration'], vehsPerHour=config['default_flow'], vType='car0')
        routes.add_flow(id='flow_sn', from_edge='edge_sc', to_edge='edge_cn', end=config['simulation_duration'], vehsPerHour=config['default_flow'], vType='car0')
        routes.add_flow(id='flow_sw', from_edge='edge_sc', to_edge='edge_cw', end=config['simulation_duration'], vehsPerHour=config['default_flow'], vType='car0')
        # Flux venant de l'ouest
        routes.add_flow(id='flow_we', from_edge='edge_wc', to_edge='edge_ce', end=config['simulation_duration'], vehsPerHour=config['default_flow'], vType='car0')
        routes.add_flow(id='flow_wn', from_edge='edge_wc', to_edge='edge_cn', end=config['simulation_duration'], vehsPerHour=config['default_flow'], vType='car0')
        routes.add_flow(id='flow_ws', from_edge='edge_wc', to_edge='edge_cs', end=config['simulation_duration'], vehsPerHour=config['default_flow'], vType='car0')

        return routes



    def simple_crossroad_fully_connected_multi_flow(self, config):
        """
        Réseau en careffour simple totalement connecté, dont les flux vont varier au cours de la simulation.
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

            vecteur_coeffs = matrice_coeffs[:,i]
            flows = vecteur_coeffs * vecteur_charge[i]
            flow_start = nb_ticks * i
            flow_end = nb_ticks * (i+1)

            # Flux venant du nord
            routes.add_flow(id=f'{i}_flow_ne', from_edge='edge_nc', to_edge='edge_ce', begin=flow_start, end=flow_end, vehsPerHour=flows[1], vType='car0', distribution="binomial")
            routes.add_flow(id=f'{i}_flow_ns', from_edge='edge_nc', to_edge='edge_cs', begin=flow_start, end=flow_end, vehsPerHour=flows[0], vType='car0', distribution="binomial")
            routes.add_flow(id=f'{i}_flow_nw', from_edge='edge_nc', to_edge='edge_cw', begin=flow_start, end=flow_end, vehsPerHour=flows[2], vType='car0', distribution="binomial")
            # Flux venant de l'est
            routes.add_flow(id=f'{i}_flow_es', from_edge='edge_ec', to_edge='edge_cs', begin=flow_start, end=flow_end, vehsPerHour=flows[3], vType='car0', distribution="binomial")
            routes.add_flow(id=f'{i}_flow_ew', from_edge='edge_ec', to_edge='edge_cw', begin=flow_start, end=flow_end, vehsPerHour=flows[5], vType='car0', distribution="binomial")
            routes.add_flow(id=f'{i}_flow_en', from_edge='edge_ec', to_edge='edge_cn', begin=flow_start, end=flow_end, vehsPerHour=flows[4], vType='car0', distribution="binomial")
            # Flux venant du sud
            routes.add_flow(id=f'{i}_flow_sw', from_edge='edge_sc', to_edge='edge_cw', begin=flow_start, end=flow_end, vehsPerHour=flows[8], vType='car0', distribution="binomial")
            routes.add_flow(id=f'{i}_flow_sn', from_edge='edge_sc', to_edge='edge_cn', begin=flow_start, end=flow_end, vehsPerHour=flows[7], vType='car0', distribution="binomial")
            routes.add_flow(id=f'{i}_flow_se', from_edge='edge_sc', to_edge='edge_ce', begin=flow_start, end=flow_end, vehsPerHour=flows[6], vType='car0', distribution="binomial")
            # Flux venant de l'ouest
            routes.add_flow(id=f'{i}_flow_wn', from_edge='edge_wc', to_edge='edge_cn', begin=flow_start, end=flow_end, vehsPerHour=flows[10], vType='car0', distribution="binomial")
            routes.add_flow(id=f'{i}_flow_we', from_edge='edge_wc', to_edge='edge_ce', begin=flow_start, end=flow_end, vehsPerHour=flows[9], vType='car0', distribution="binomial")
            routes.add_flow(id=f'{i}_flow_ws', from_edge='edge_wc', to_edge='edge_cs', begin=flow_start, end=flow_end, vehsPerHour=flows[11], vType='car0', distribution="binomial")

        return routes



    ### Additionals ###

    def no_additionals(self, config):
        """
        Fonction qui n'ajoute aucun élément additionnels à un réseau.

        :param config: Configuration du réseau routier
        :return: Retourne un objet Additional vide
        """

        additional = Additional()
        return additional

    def detecteurs_numeriques_simple_carrefour(self, config):
        """
        Fonction qui ajoute des détecteurs numériques sur toute les voies d'entrée du carrefour d'un réseau
        à carrefour unique.

        :param config: Configuration du réseau routier
        :return: Retourne un objet Additional représentant les détecteurs mentionnés
        """

        additional = Additional()

        additional.add_laneAreaDetector(id="wc", lane="edge_wc_0")
        additional.add_laneAreaDetector(id="sc", lane="edge_sc_0")
        additional.add_laneAreaDetector(id="nc", lane="edge_nc_0")
        additional.add_laneAreaDetector(id="ec", lane="edge_ec_0")

        return additional

    def detecteurs_booleens_simple_carrefour(self, config):
        """
        Fonction qui ajoute des détecteurs booléens sur toute les voies d'entrée du carrefour d'un réseau
        à carrefour unique.

        :param config: Configuration du réseau routier
        :return: Retourne un objet Additional représentant les détecteurs mentionnés
        """

        additional = Additional()

        detector_length = config["boolean_detector_length"]
        lane_len = config['default_len']

        additional.add_laneAreaDetector(id="wc", lane="edge_wc_0", pos=(lane_len - detector_length - 7.2))
        additional.add_laneAreaDetector(id="sc", lane="edge_sc_0", pos=(lane_len - detector_length - 7.2))
        additional.add_laneAreaDetector(id="nc", lane="edge_nc_0", pos=(lane_len - detector_length - 7.2))
        additional.add_laneAreaDetector(id="ec", lane="edge_ec_0", pos=(lane_len - detector_length - 7.2))

        return additional




    ### Strategies ###

    def feux_a_detection_booleenne_simple_carrefour(self, config):
        """
        Sur chaque voie arrivant devant un feu, il existe un détecteur qui a pour charge d'évaluer si un véhicule est présent dans les
        7 derniers mêtres avant le feu. S'il y en a un, et que le feu est vert, alors on garde le feu vert. S'il n'y en a pas, et que le feu
        est vert, et qu'il y a un véhicule en attente sur une voie perpendiculaire, alors on change de feu. L'opération ne peut pas se faire
        avant "min_duration_tl" step depuis le dernier changement, et se fait obligatoirement après "max_duration_tl" step depuis le dernier
        changement.

        Ne fonctionne que pour un réseau à un carrefour.

        Paramètres à mettre dans la config :
        - "min_duration_tl" (int) : Le nombre de pas minimum de durée d'un feu
        - "max_duration_tl" (int) : Le nombre de pas maximum de durée d'un feu
        """

        step = 0
        cooldownStep = 0  # Nombre de pas depuis la dernière actualisation du feu

        FEU_VERT_HORIZONTAL = 0
        FEU_VERT_VERTICAL = 2

        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]

        while step < config['simulation_duration']:
            traci.simulationStep()

            if cooldownStep > min_duration_tl:
                if traci.trafficlight.getPhase("c") == FEU_VERT_HORIZONTAL:
                    quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(
                        "nc") >= 1 or traci.lanearea.getLastStepVehicleNumber("sc") >= 1)
                    autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(
                        "wc") == 0 and traci.lanearea.getLastStepVehicleNumber("ec") == 0)
                    if (quelqun_en_attente and autre_voie_vide) or cooldownStep > max_duration_tl:
                        traci.trafficlight.setPhase("c", FEU_VERT_HORIZONTAL + 1)  # Passage au orange
                        cooldownStep = 0

                elif traci.trafficlight.getPhase("c") == FEU_VERT_VERTICAL:
                    quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(
                        "wc") >= 1 or traci.lanearea.getLastStepVehicleNumber("ec") >= 1)
                    autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(
                        "nc") == 0 and traci.lanearea.getLastStepVehicleNumber("sc") == 0)
                    if (quelqun_en_attente and autre_voie_vide) or cooldownStep > max_duration_tl:
                        traci.trafficlight.setPhase("c", FEU_VERT_VERTICAL + 1)
                        cooldownStep = 0

            step += 1
            cooldownStep += 1

    def feux_a_seuils_communicants_avec_anticipation_simple_carrefour(self, config):
        """
        On détermine un seuil pour chaque feu du réseau. Lorsque le feu est rouge pour une voie du feu, et que le nombre de véhicules en attente
        est supérieur ou égal au seuil, alors le feu passe au vert. Le feu doit rester vert un minimum de temps avant de pouvoir à nouveau changer.

        Ne fonctionne qu'avec un réseau avec un unique carrefour

        Paramètres à mettre dans la config :
        - "min_duration_tl" (int) : Le nombre de pas minimum de durée d'un feu
        - "max_duration_tl" (int) : Le nombre de pas maximum de durée d'un feu
        - "seuil_vehicules" (int) : Le nombre de véhicules en attente nécessaires pour déclencher un changement de feu. (sur une voie uniquement)
        """

        step = 0
        cooldownStep = 0  # Nombre de pas depuis la dernière actualisation du feu 1

        FEU_ROUGE_HORIZONTAL = 2
        FEU_ROUGE_VERTICAL = 0

        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        nb_vehicules = config["seuil_vehicules"]

        while step < config['simulation_duration']:
            traci.simulationStep()

            # Premier carrefour
            if cooldownStep > min_duration_tl:
                if traci.trafficlight.getPhase("c") == FEU_ROUGE_HORIZONTAL:
                    if traci.lanearea.getLastStepVehicleNumber(
                            "wc") >= nb_vehicules or traci.lanearea.getLastStepVehicleNumber(
                            "ec") >= nb_vehicules or cooldownStep > max_duration_tl:
                        traci.trafficlight.setPhase("c", FEU_ROUGE_HORIZONTAL + 1)  # Passage au orange
                        cooldownStep = 0

                elif traci.trafficlight.getPhase("c") == FEU_ROUGE_VERTICAL:
                    if traci.lanearea.getLastStepVehicleNumber(
                            "sc") >= nb_vehicules or traci.lanearea.getLastStepVehicleNumber(
                            "nc") >= nb_vehicules or cooldownStep > max_duration_tl:
                        traci.trafficlight.setPhase("c", FEU_ROUGE_VERTICAL + 1)
                        cooldownStep = 0

            step += 1
            cooldownStep += 1

    def feux_a_seuils_communicants_sans_anticipation_simple_carrefour(self, config):
        """
        On détermine un seuil pour chaque feu du réseau. Lorsque le feu est rouge pour une voie du feu, et que le nombre de véhicules en attente et en course
        sur l'autre voie est supérieur ou égal au seuil, alors le feu passe au vert. Le feu doit rester vert un minimum de temps avant de pouvoir à nouveau changer.

        Paramètres à mettre dans la config :
        - "min_duration_tl" (int) : Le nombre de pas minimum de durée d'un feu
        - "max_duration_tl" (int) : Le nombre de pas maximum de durée d'un feu
        - "seuil_vehicules" (int) : Le nombre de véhicules en attente nécessaires pour déclencher un changement de feu. (sur une voie uniquement)
        """

        step = 0
        cooldownStep = 0  # Nombre de pas depuis la dernière actualisation du feu 1

        FEU_ROUGE_HORIZONTAL = 2
        FEU_ROUGE_VERTICAL = 0

        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        nb_vehicules = config["seuil_vehicules"]

        while step < config['simulation_duration']:
            traci.simulationStep()

            # Premier carrefour
            if cooldownStep > min_duration_tl:
                if traci.trafficlight.getPhase("c") == FEU_ROUGE_HORIZONTAL:
                    if traci.lanearea.getJamLengthVehicle("wc") >= nb_vehicules or traci.lanearea.getJamLengthVehicle(
                            "ec") >= nb_vehicules or cooldownStep > max_duration_tl:
                        traci.trafficlight.setPhase("c", FEU_ROUGE_HORIZONTAL + 1)  # Passage au orange
                        cooldownStep = 0

                elif traci.trafficlight.getPhase("c") == FEU_ROUGE_VERTICAL:
                    if traci.lanearea.getJamLengthVehicle("sc") >= nb_vehicules or traci.lanearea.getJamLengthVehicle(
                            "nc") >= nb_vehicules or cooldownStep > max_duration_tl:
                        traci.trafficlight.setPhase("c", FEU_ROUGE_VERTICAL + 1)
                        cooldownStep = 0

            step += 1
            cooldownStep += 1