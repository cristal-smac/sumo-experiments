import numpy as np
import sys, os
from core.src.components import Network, Routes, DetectorBuilder
tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
sys.path.append(tools)
import traci

class SquareNetwork:


    ### Network ###

    def random_square_crossroad_network(self, config):
        """
        Génère un réseau de carrefours carré, c'est à dire avec `nb_roads_by_side` horizontales et le même nombre de verticales.
        Les coordonnées des points sont modifiées pour "casser" le maillage en carré du réseau.
        Le réseau contient donc `nb_roads_by_side` * `nb_roads_by_side` carrefours tricolores.
        Le paramètre `nb_roads_by_side` doit être mis dans la config sous la forme config['nb_roads_by_side']
        :param config: Configuration du réseau routier
        :return: Retourne un objet Routes représentant les routes du réseau routier créé
        """

        net = Network()

        nb_roads_by_side = config["nb_roads_by_side"]

        # On vérifie la valeur de nb_roads_by_side
        if nb_roads_by_side <= 1:
            raise ValueError("nb_roads_by_side doit être supérieur à 1.")

        coordonnees_noeud = range(nb_roads_by_side + 2)
        valeurs_decalage_x = [np.random.uniform(0, config["default_len"]) for _ in range(nb_roads_by_side + 2)]
        valeurs_decalage_y = [np.random.uniform(0, config["default_len"]) for _ in range(nb_roads_by_side + 2)]
        decalage_x = dict(zip(coordonnees_noeud, valeurs_decalage_x))
        decalage_y = dict(zip(coordonnees_noeud, valeurs_decalage_y))

        # On ajoute les noeuds au réseau
        for x in range(nb_roads_by_side + 2):
            for y in range(nb_roads_by_side + 2):

                # Pas de noeuds dans les coins du réseau
                est_un_coin = ((x == y) and (x == 0 or x == nb_roads_by_side + 1)) \
                              or (x == 0 and y == nb_roads_by_side + 1) \
                              or (y == 0 and x == nb_roads_by_side + 1)

                if not est_un_coin:
                    # Si le noeud est en bordure, on le configure comme une entrée de flux
                    if (x in [0, nb_roads_by_side + 1]) or (y in [0, nb_roads_by_side + 1]):
                        net.add_node(id=f'x{x}-y{y}', x=x * config["default_len"] + decalage_x[x],
                                     y=y * config["default_len"] + decalage_y[y])
                    # Sinon le noeud est un feu tricolore
                    else:
                        net.add_node(id=f'x{x}-y{y}', x=x * config["default_len"] + decalage_x[x],
                                     y=y * config["default_len"] + decalage_y[y], type='traffic_light', tl=f'x{x}-y{y}')

        # On ajoute un type de edge au réseau
        net.add_type(id='default', values={'numLanes': 1, 'speed': config['default_speed']})

        # On ajoute les arrêtes au réseau
        for x in range(nb_roads_by_side + 2):
            for y in range(nb_roads_by_side + 2):

                # Pas de noeuds dans les coins du réseau
                est_un_coin = ((x == y) and (x == 0 or x == nb_roads_by_side + 1)) \
                              or (x == 0 and y == nb_roads_by_side + 1) \
                              or (y == 0 and x == nb_roads_by_side + 1)

                if not est_un_coin:
                    # On lie le noeud actuel avec ses voisins gauche et droite (si présents et pas sur les bordures)
                    if y not in [0, nb_roads_by_side + 1]:
                        if x < nb_roads_by_side + 1:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x + 1}-y{y}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x + 1}-y{y}', type='default')
                        if x > 0:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x - 1}-y{y}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x - 1}-y{y}', type='default')
                    # On lie le noeud actuel avec ses voisins haut et bas (si présents et pas sur les bordures)
                    if x not in [0, nb_roads_by_side + 1]:
                        if y < nb_roads_by_side + 1:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x}-y{y + 1}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x}-y{y + 1}', type='default')
                        if y > 0:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x}-y{y - 1}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x}-y{y - 1}', type='default')

        # On créé les connexions du réseau
        for x in range(nb_roads_by_side + 2):
            for y in range(nb_roads_by_side + 2):

                # Pas de noeuds dans les coins du réseau
                est_un_coin = ((x == y) and (x == 0 or x == nb_roads_by_side + 1)) \
                              or (x == 0 and y == nb_roads_by_side + 1) \
                              or (y == 0 and x == nb_roads_by_side + 1)

                if not est_un_coin:

                    # On lie le edge de droite et de gauche avec tous les autres de leur carrefour (si présents)
                    # Si le noeud est une bordure haute ou basse, pas de connexion
                    if y not in [0, nb_roads_by_side + 1]:
                        # A droite
                        if x != nb_roads_by_side:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x + 1}-y{y}',
                                               to_edge=f'edge_x{x + 1}-y{y}_x{x + 2}-y{y}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x + 1}-y{y}',
                                               to_edge=f'edge_x{x + 1}-y{y}_x{x + 1}-y{y + 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x + 1}-y{y}',
                                               to_edge=f'edge_x{x + 1}-y{y}_x{x + 1}-y{y - 1}')
                        # A gauche
                        if x > 1:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x - 1}-y{y}',
                                               to_edge=f'edge_x{x - 1}-y{y}_x{x - 2}-y{y}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x - 1}-y{y}',
                                               to_edge=f'edge_x{x - 1}-y{y}_x{x - 1}-y{y + 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x - 1}-y{y}',
                                               to_edge=f'edge_x{x - 1}-y{y}_x{x - 1}-y{y - 1}')

                    # On lie le edge du haut et du bas avec tous les autres de leur carrefour (si présents)
                    # Si le noeud est une bordure gauche ou droite, pas de connexion
                    if x not in [0, nb_roads_by_side + 1]:
                        # En haut
                        if y < nb_roads_by_side:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y + 1}',
                                               to_edge=f'edge_x{x}-y{y + 1}_x{x}-y{y + 2}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y + 1}',
                                               to_edge=f'edge_x{x}-y{y + 1}_x{x + 1}-y{y + 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y + 1}',
                                               to_edge=f'edge_x{x}-y{y + 1}_x{x - 1}-y{y + 1}')
                        # En bas
                        if y > 1:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y - 1}',
                                               to_edge=f'edge_x{x}-y{y - 1}_x{x}-y{y - 2}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y - 1}',
                                               to_edge=f'edge_x{x}-y{y - 1}_x{x + 1}-y{y - 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y - 1}',
                                               to_edge=f'edge_x{x}-y{y - 1}_x{x - 1}-y{y - 1}')

        # On ajoute les programmes de feux tricolores pour chaque carrefour
        for x in range(1, nb_roads_by_side + 1):
            for y in range(1, nb_roads_by_side + 1):
                net.add_traffic_light_program(id=f'x{x}-y{y}', programID=f'c-x{x}-y{y}',
                                              phases=[{'duration': config['default_green_time'], 'state': 'rrrGGGrrrGGG'},
                                                      {'duration': config['default_yellow_time'], 'state': 'rrryyyrrryyy'},
                                                      {'duration': config['default_green_time'], 'state': 'GGGrrrGGGrrr'},
                                                      {'duration': config['default_yellow_time'], 'state': 'yyyrrryyyrrr'}])

        return net


    def square_crossroad_network(self, config):
        """
        Génère un réseau de carrefours carré, c'est à dire avec `nb_roads_by_side` horizontales et le même nombre de verticales.
        Le réseau contient donc `nb_roads_by_side` * `nb_roads_by_side` carrefours tricolores.
        Le paramètre `nb_roads_by_side` doit être mis dans la config sous la forme config['nb_roads_by_side']
        :param config: Configuration du réseau routier
        :return: Retourne un objet Routes représentant les routes du réseau routier créé
        """

        net = Network()

        nb_roads_by_side = config["nb_roads_by_side"]

        # On vérifie la valeur de nb_roads_by_side
        if nb_roads_by_side <= 1:
            raise ValueError("nb_roads_by_side doit être supérieur à 1.")

        # On ajoute les noeuds au réseau
        for x in range(nb_roads_by_side + 2):
            for y in range(nb_roads_by_side + 2):

                # Pas de noeuds dans les coins du réseau
                est_un_coin = ((x == y) and (x == 0 or x == nb_roads_by_side + 1)) \
                              or (x == 0 and y == nb_roads_by_side + 1) \
                              or (y == 0 and x == nb_roads_by_side + 1)

                if not est_un_coin:
                    # Si le noeud est en bordure, on le configure comme une entrée de flux
                    if (x in [0, nb_roads_by_side + 1]) or (y in [0, nb_roads_by_side + 1]):
                        net.add_node(id=f'x{x}-y{y}', x=x * config["default_len"], y=y * config["default_len"])
                    # Sinon le noeud est un feu tricolore
                    else:
                        net.add_node(id=f'x{x}-y{y}', x=x * config["default_len"], y=y * config["default_len"],
                                     type='traffic_light', tl=f'x{x}-y{y}')

        # On ajoute un type de edge au réseau
        net.add_type(id='default', values={'numLanes': 1, 'speed': config['default_speed']})

        # On ajoute les arrêtes au réseau
        for x in range(nb_roads_by_side + 2):
            for y in range(nb_roads_by_side + 2):

                # Pas de noeuds dans les coins du réseau
                est_un_coin = ((x == y) and (x == 0 or x == nb_roads_by_side + 1)) \
                              or (x == 0 and y == nb_roads_by_side + 1) \
                              or (y == 0 and x == nb_roads_by_side + 1)

                if not est_un_coin:
                    # On lie le noeud actuel avec ses voisins gauche et droite (si présents et pas sur les bordures)
                    if y not in [0, nb_roads_by_side + 1]:
                        if x < nb_roads_by_side + 1:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x + 1}-y{y}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x + 1}-y{y}', type='default')
                        if x > 0:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x - 1}-y{y}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x - 1}-y{y}', type='default')
                    # On lie le noeud actuel avec ses voisins haut et bas (si présents et pas sur les bordures)
                    if x not in [0, nb_roads_by_side + 1]:
                        if y < nb_roads_by_side + 1:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x}-y{y + 1}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x}-y{y + 1}', type='default')
                        if y > 0:
                            net.add_edge(id=f'edge_x{x}-y{y}_x{x}-y{y - 1}', from_node=f'x{x}-y{y}',
                                         to_node=f'x{x}-y{y - 1}', type='default')

        # On créé les connexions du réseau
        for x in range(nb_roads_by_side + 2):
            for y in range(nb_roads_by_side + 2):

                # Pas de noeuds dans les coins du réseau
                est_un_coin = ((x == y) and (x == 0 or x == nb_roads_by_side + 1)) \
                              or (x == 0 and y == nb_roads_by_side + 1) \
                              or (y == 0 and x == nb_roads_by_side + 1)

                if not est_un_coin:

                    # On lie le edge de droite et de gauche avec tous les autres de leur carrefour (si présents)
                    # Si le noeud est une bordure haute ou basse, pas de connexion
                    if y not in [0, nb_roads_by_side + 1]:
                        # A droite
                        if x != nb_roads_by_side:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x + 1}-y{y}',
                                               to_edge=f'edge_x{x + 1}-y{y}_x{x + 2}-y{y}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x + 1}-y{y}',
                                               to_edge=f'edge_x{x + 1}-y{y}_x{x + 1}-y{y + 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x + 1}-y{y}',
                                               to_edge=f'edge_x{x + 1}-y{y}_x{x + 1}-y{y - 1}')
                        # A gauche
                        if x > 1:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x - 1}-y{y}',
                                               to_edge=f'edge_x{x - 1}-y{y}_x{x - 2}-y{y}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x - 1}-y{y}',
                                               to_edge=f'edge_x{x - 1}-y{y}_x{x - 1}-y{y + 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x - 1}-y{y}',
                                               to_edge=f'edge_x{x - 1}-y{y}_x{x - 1}-y{y - 1}')

                    # On lie le edge du haut et du bas avec tous les autres de leur carrefour (si présents)
                    # Si le noeud est une bordure gauche ou droite, pas de connexion
                    if x not in [0, nb_roads_by_side + 1]:
                        # En haut
                        if y < nb_roads_by_side:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y + 1}',
                                               to_edge=f'edge_x{x}-y{y + 1}_x{x}-y{y + 2}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y + 1}',
                                               to_edge=f'edge_x{x}-y{y + 1}_x{x + 1}-y{y + 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y + 1}',
                                               to_edge=f'edge_x{x}-y{y + 1}_x{x - 1}-y{y + 1}')
                        # En bas
                        if y > 1:
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y - 1}',
                                               to_edge=f'edge_x{x}-y{y - 1}_x{x}-y{y - 2}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y - 1}',
                                               to_edge=f'edge_x{x}-y{y - 1}_x{x + 1}-y{y - 1}')
                            net.add_connection(from_edge=f'edge_x{x}-y{y}_x{x}-y{y - 1}',
                                               to_edge=f'edge_x{x}-y{y - 1}_x{x - 1}-y{y - 1}')

        # On ajoute les programmes de feux tricolores pour chaque carrefour
        for x in range(1, nb_roads_by_side + 1):
            for y in range(1, nb_roads_by_side + 1):
                net.add_traffic_light_program(id=f'x{x}-y{y}', programID=f'c-x{x}-y{y}',
                                              phases=[{'duration': config['default_green_time'], 'state': 'rrrGGGrrrGGG'},
                                                      {'duration': config['default_yellow_time'], 'state': 'rrryyyrrryyy'},
                                                      {'duration': config['default_green_time'], 'state': 'GGGrrrGGGrrr'},
                                                      {'duration': config['default_yellow_time'], 'state': 'yyyrrryyyrrr'}])

        return net



    ### Routes ###

    def square_crossroad_routes(self, config):
        """
        Génère les routes pour un réseau de carrefours carré.
        Le paramètre `nb_roads_by_side` doit être mis dans la config sous la forme config['nb_roads_by_side']
        :param config: Configuration du réseau routier
        :return: Retourne un objet Routes représentant les routes du réseau routier créé
        """

        routes = Routes()

        nb_roads_by_side = config["nb_roads_by_side"]

        # On intègre tous les edges en bordure dans une liste.
        # Ils sont stocké dans le sens des aiguilles d'une montre, en partant du edge x0y1-x1y1.
        liste_entrees = []
        liste_sorties = []
        for x in range(1, nb_roads_by_side + 1):
            liste_entrees.append(f'edge_x{x}-y{nb_roads_by_side + 1}_x{x}-y{nb_roads_by_side}')
            liste_sorties.append(f'edge_x{x}-y{nb_roads_by_side}_x{x}-y{nb_roads_by_side + 1}')
        for y in range(nb_roads_by_side, 0, -1):
            liste_entrees.append(f'edge_x{nb_roads_by_side + 1}-y{y}_x{nb_roads_by_side}-y{y}')
            liste_sorties.append(f'edge_x{nb_roads_by_side}-y{y}_x{nb_roads_by_side + 1}-y{y}')
        for x in range(nb_roads_by_side, 0, -1):
            liste_entrees.append(f'edge_x{x}-y0_x{x}-y1')
            liste_sorties.append(f'edge_x{x}-y1_x{x}-y0')
        for y in range(1, nb_roads_by_side + 1):
            liste_entrees.append(f'edge_x0-y{y}_x1-y{y}')
            liste_sorties.append(f'edge_x1-y{y}_x0-y{y}')

        # On ajoute le type de véhicule
        routes.add_vType(id='car0')

        for i in range(len(liste_entrees)):
            for j in range(len(liste_sorties)):
                # Un véhicule ne peut pas sortir pas là où il est entré
                if i != j:
                    routes.add_flow(id=f"flow_{liste_entrees[i]}_{liste_entrees[j]}",
                                    from_edge=liste_entrees[i],
                                    to_edge=liste_sorties[j],
                                    end=config['simulation_duration'],
                                    vehsPerHour=config['default_flow'],
                                    vType='car0')

        return routes


    def square_crossroad_routes_multi_flow(self, config):
        """
        Génère les routes pour un réseau de carrefours carré.
        Le paramètre `nb_roads_by_side` doit être mis dans la config sous la forme config['nb_roads_by_side']
        :param config: Configuration du réseau routier
        :return: Retourne un objet Routes représentant les routes du réseau routier créé
        """

        routes = Routes()

        nb_roads_by_side = config["nb_roads_by_side"]
        load_vector = config["load_vector"]
        coeff_matrix = config["coeff_matrix"]
        nb_ticks = config["nb_ticks"]

        # On intègre tous les edges en bordure dans une liste.
        # Ils sont stocké dans le sens des aiguilles d'une montre, en partant du edge x0y1-x1y1.
        liste_entrees = []
        liste_sorties = []
        for x in range(1, nb_roads_by_side + 1):
            liste_entrees.append(f'edge_x{x}-y{nb_roads_by_side + 1}_x{x}-y{nb_roads_by_side}')
            liste_sorties.append(f'edge_x{x}-y{nb_roads_by_side}_x{x}-y{nb_roads_by_side + 1}')
        for y in range(nb_roads_by_side, 0, -1):
            liste_entrees.append(f'edge_x{nb_roads_by_side + 1}-y{y}_x{nb_roads_by_side}-y{y}')
            liste_sorties.append(f'edge_x{nb_roads_by_side}-y{y}_x{nb_roads_by_side + 1}-y{y}')
        for x in range(nb_roads_by_side, 0, -1):
            liste_entrees.append(f'edge_x{x}-y0_x{x}-y1')
            liste_sorties.append(f'edge_x{x}-y1_x{x}-y0')
        for y in range(1, nb_roads_by_side + 1):
            liste_entrees.append(f'edge_x0-y{y}_x1-y{y}')
            liste_sorties.append(f'edge_x1-y{y}_x0-y{y}')

        # On ajoute le type de véhicule
        routes.add_vType(id='car0')

        for period in range(len(load_vector)):

            vecteur_coeffs = coeff_matrix[:, period]
            flows = vecteur_coeffs * load_vector[period]
            flow_start = nb_ticks * period
            flow_end = nb_ticks * (period + 1)
            compteur = 0

            for i in range(len(liste_entrees)):
                for j in range(len(liste_sorties)):
                    # Un véhicule ne peut pas sortir pas là où il est entré
                    if i != j:
                        routes.add_flow(id=f"flow_{liste_entrees[i]}_{liste_entrees[j]}_{period}",
                                        from_edge=liste_entrees[i],
                                        to_edge=liste_sorties[j],
                                        begin=flow_start,
                                        end=flow_end,
                                        vehsPerHour=flows[compteur],
                                        vType='car0',
                                        distribution="binomial")
                        compteur += 1

        return routes



    ### Additionals ###

    def no_detectors(self, config):
        """
        Fonction qui n'ajoute aucun détecteur à un réseau.

        :param config: Configuration du réseau routier
        :return: Retourne un objet DetectorBuilder vide
        """

        return DetectorBuilder()

    def detecteurs_numeriques_reseau_carre(self, config):
        """
        Fonction qui ajoute des détecteurs numeriques entre les carrefours d'un réseau carré
        de sorte à simuler une communication entre carrefours voisins.

        :param config: Configuration du réseau routier
        :return: Retourne un objet DetectorBuilder représentant les détecteurs mentionnés
        """

        detectors = DetectorBuilder()

        nb_roads_by_side = config["nb_roads_by_side"]

        # On ajoute les arrêtes au réseau
        for x in range(nb_roads_by_side + 2):
            for y in range(nb_roads_by_side + 2):

                # Pas de noeuds dans les coins du réseau
                est_un_coin = ((x == y) and (x == 0 or x == nb_roads_by_side + 1)) \
                              or (x == 0 and y == nb_roads_by_side + 1) \
                              or (y == 0 and x == nb_roads_by_side + 1)

                if not est_un_coin:
                    if y not in [0, nb_roads_by_side + 1]:
                        if x < nb_roads_by_side:
                            detectors.add_laneAreaDetector(id=f'detector_x{x}-y{y}_x{x + 1}-y{y}',
                                                            lane=f'edge_x{x}-y{y}_x{x + 1}-y{y}_0')
                        if x > 1:
                            detectors.add_laneAreaDetector(id=f'detector_x{x}-y{y}_x{x - 1}-y{y}',
                                                            lane=f'edge_x{x}-y{y}_x{x - 1}-y{y}_0')
                    if x not in [0, nb_roads_by_side + 1]:
                        if y < nb_roads_by_side:
                            detectors.add_laneAreaDetector(id=f'detector_x{x}-y{y}_x{x}-y{y + 1}',
                                                            lane=f'edge_x{x}-y{y}_x{x}-y{y + 1}_0')
                        if y > 1:
                            detectors.add_laneAreaDetector(id=f'detector_x{x}-y{y}_x{x}-y{y - 1}',
                                                            lane=f'edge_x{x}-y{y}_x{x}-y{y - 1}_0')

        return detectors

    def detecteurs_booleens_reseau_carre(self, config):
        """
        Fonction qui ajoute des détecteurs booleens entre les carrefours d'un réseau carré.

        :param config: Configuration du réseau routier
        :return: Retourne un objet DetectorBuilder représentant les détecteurs mentionnés
        """

        detectors = DetectorBuilder()

        nb_roads_by_side = config["nb_roads_by_side"]

        # On ajoute les arrêtes au réseau
        for x in range(nb_roads_by_side + 2):
            for y in range(nb_roads_by_side + 2):

                # Pas de noeuds dans les coins du réseau
                est_un_coin = ((x == y) and (x == 0 or x == nb_roads_by_side + 1)) \
                              or (x == 0 and y == nb_roads_by_side + 1) \
                              or (y == 0 and x == nb_roads_by_side + 1)

                detector_length = config["boolean_detector_length"]
                lane_len = config['default_len']

                if not est_un_coin:
                    if y not in [0, nb_roads_by_side + 1]:
                        if x < nb_roads_by_side:
                            if x == 0:
                                detectors.add_laneAreaDetector(id=f'detector_x{x}-y{y}_x{x + 1}-y{y}',
                                                                lane=f'edge_x{x}-y{y}_x{x + 1}-y{y}_0',
                                                                pos=(lane_len - detector_length - 7.2))
                            else:
                                detectors.add_laneAreaDetector(id=f'detector_x{x}-y{y}_x{x + 1}-y{y}',
                                                                lane=f'edge_x{x}-y{y}_x{x + 1}-y{y}_0',
                                                                pos=(lane_len - detector_length - 7.2 * 2))
                        if x > 1:
                            if x == nb_roads_by_side + 1:
                                detectors.add_laneAreaDetector(id=f'detector_x{x}-y{y}_x{x - 1}-y{y}',
                                                                lane=f'edge_x{x}-y{y}_x{x - 1}-y{y}_0',
                                                                pos=(lane_len - detector_length - 7.2))
                            else:
                                detectors.add_laneAreaDetector(id=f'detector_x{x}-y{y}_x{x - 1}-y{y}',
                                                                lane=f'edge_x{x}-y{y}_x{x - 1}-y{y}_0',
                                                                pos=(lane_len - detector_length - 7.2 * 2))
                    if x not in [0, nb_roads_by_side + 1]:
                        if y < nb_roads_by_side:
                            if y == 0:
                                detectors.add_laneAreaDetector(id=f'detector_x{x}-y{y}_x{x}-y{y + 1}',
                                                                lane=f'edge_x{x}-y{y}_x{x}-y{y + 1}_0',
                                                                pos=(lane_len - detector_length - 7.2))
                            else:
                                detectors.add_laneAreaDetector(id=f'detector_x{x}-y{y}_x{x}-y{y + 1}',
                                                                lane=f'edge_x{x}-y{y}_x{x}-y{y + 1}_0',
                                                                pos=(lane_len - detector_length - 7.2 * 2))
                        if y > 1:
                            if y == nb_roads_by_side + 1:
                                detectors.add_laneAreaDetector(id=f'detector_x{x}-y{y}_x{x}-y{y - 1}',
                                                                lane=f'edge_x{x}-y{y}_x{x}-y{y - 1}_0',
                                                                pos=(lane_len - detector_length - 7.2))
                            else:
                                detectors.add_laneAreaDetector(id=f'detector_x{x}-y{y}_x{x}-y{y - 1}',
                                                                lane=f'edge_x{x}-y{y}_x{x}-y{y - 1}_0',
                                                                pos=(lane_len - detector_length - 7.2 * 2))

        return detectors



    ### Strategies ###

    def feux_a_detection_booleenne_reseau_carre(self, config):
        """
        Sur chaque voie arrivant devant un feu, il existe un détecteur qui a pour charge d'évaluer si un véhicule est présent dans les
        7 derniers mêtres avant le feu. S'il y en a un, et que le feu est vert, alors on garde le feu vert. S'il n'y en a pas, et que le feu
        est vert, et qu'il y a un véhicule en attente sur une voie perpendiculaire, alors on change de feu. L'opération ne peut pas se faire
        avant "min_duration_tl" step depuis le dernier changement, et se fait obligatoirement après "max_duration_tl" step depuis le dernier
        changement.

        Ne fonctionne que pour un réseau carré.

        Paramètres à mettre dans la config :
        - "min_duration_tl" (int) : Le nombre de pas minimum de durée d'un feu
        - "max_duration_tl" (int) : Le nombre de pas maximum de durée d'un feu
        - "nb_roads_by_side" (int) : Le nombre de feu sur un côté du réseau carré
        """

        FEU_VERT_HORIZONTAL = 0
        FEU_VERT_VERTICAL = 2

        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        nb_roads_by_side = config["nb_roads_by_side"]

        cooldownStep = np.zeros(
            (nb_roads_by_side, nb_roads_by_side))  # Nombre de pas depuis la dernière actualisation du feu
        step = 0

        while step < config['simulation_duration']:
            traci.simulationStep()

            for x in range(1, nb_roads_by_side + 1):
                for y in range(1, nb_roads_by_side + 1):

                    if cooldownStep[x - 1, y - 1] > min_duration_tl:
                        if traci.trafficlight.getPhase(f'x{x}-y{y}') == FEU_VERT_HORIZONTAL:
                            quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(
                                f'detector_x{x}-y{y + 1}_x{x}-y{y}') >= 1 \
                                                  or traci.lanearea.getLastStepVehicleNumber(
                                        f'detector_x{x}-y{y - 1}_x{x}-y{y}') >= 1)
                            autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(
                                f'detector_x{x - 1}-y{y}_x{x}-y{y}') == 0 \
                                               and traci.lanearea.getLastStepVehicleNumber(
                                        f'detector_x{x + 1}-y{y}_x{x}-y{y}') == 0)
                            if (quelqun_en_attente and autre_voie_vide) or cooldownStep[x - 1, y - 1] > max_duration_tl:
                                traci.trafficlight.setPhase(f'x{x}-y{y}', FEU_VERT_HORIZONTAL + 1)  # Passage au orange
                                cooldownStep[x - 1, y - 1] = 0

                        elif traci.trafficlight.getPhase(f'x{x}-y{y}') == FEU_VERT_VERTICAL:
                            quelqun_en_attente = (traci.lanearea.getLastStepVehicleNumber(
                                f'detector_x{x - 1}-y{y}_x{x}-y{y}') >= 1 \
                                                  or traci.lanearea.getLastStepVehicleNumber(
                                        f'detector_x{x + 1}-y{y}_x{x}-y{y}') >= 1)
                            autre_voie_vide = (traci.lanearea.getLastStepVehicleNumber(
                                f'detector_x{x}-y{y + 1}_x{x}-y{y}') == 0 \
                                               and traci.lanearea.getLastStepVehicleNumber(
                                        f'detector_x{x}-y{y - 1}_x{x}-y{y}') == 0)
                            if (quelqun_en_attente and autre_voie_vide) or cooldownStep[x - 1, y - 1] > max_duration_tl:
                                traci.trafficlight.setPhase(f'x{x}-y{y}', FEU_VERT_VERTICAL + 1)
                                cooldownStep[x - 1, y - 1] = 0

                    cooldownStep[x - 1, y - 1] += 1

            step += 1

    def feux_a_seuils_communicants_avec_anticipation_reseau_carre(self, config):
        """
        On détermine un seuil pour chaque feu du réseau. Lorsque le feu est rouge pour une voie du feu, et que le nombre de véhicules en attente
        est supérieur ou égal au seuil, alors le feu passe au vert. Le feu doit rester vert un minimum de temps avant de pouvoir à nouveau changer.

        Ne fonctionne qu'avec un réseau carré

        Paramètres à mettre dans la config :
        - "min_duration_tl" (int) : Le nombre de pas minimum de durée d'un feu
        - "max_duration_tl" (int) : Le nombre de pas maximum de durée d'un feu
        - "seuil_vehicules" (int) : Le nombre de véhicules en attente nécessaires pour déclencher un changement de feu. (sur une voie uniquement)
        - "nb_roads_by_side" (int) : Le nombre de feu sur un côté du réseau carré
        """

        FEU_ROUGE_HORIZONTAL = 2
        FEU_ROUGE_VERTICAL = 0

        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        nb_vehicules = config["seuil_vehicules"]
        nb_roads_by_side = config["nb_roads_by_side"]

        cooldownStep = np.zeros(
            (nb_roads_by_side, nb_roads_by_side))  # Nombre de pas depuis la dernière actualisation du feu
        step = 0

        while step < config['simulation_duration']:
            traci.simulationStep()

            for x in range(1, nb_roads_by_side + 1):
                for y in range(1, nb_roads_by_side + 1):

                    if cooldownStep[x - 1, y - 1] > min_duration_tl:

                        if traci.trafficlight.getPhase(f'x{x}-y{y}') == FEU_ROUGE_HORIZONTAL:

                            if traci.lanearea.getLastStepVehicleNumber(
                                    f'detector_x{x - 1}-y{y}_x{x}-y{y}') >= nb_vehicules \
                                    or traci.lanearea.getLastStepVehicleNumber(
                                f'detector_x{x + 1}-y{y}_x{x}-y{y}') >= nb_vehicules \
                                    or cooldownStep[x - 1, y - 1] > max_duration_tl:
                                traci.trafficlight.setPhase(f'x{x}-y{y}', FEU_ROUGE_HORIZONTAL + 1)  # Passage au orange
                                cooldownStep[x - 1, y - 1] = 0


                        elif traci.trafficlight.getPhase(f'x{x}-y{y}') == FEU_ROUGE_VERTICAL:

                            if traci.lanearea.getLastStepVehicleNumber(
                                    f'detector_x{x}-y{y + 1}_x{x}-y{y}') >= nb_vehicules \
                                    or traci.lanearea.getLastStepVehicleNumber(
                                f'detector_x{x}-y{y - 1}_x{x}-y{y}') >= nb_vehicules \
                                    or cooldownStep[x - 1, y - 1] > max_duration_tl:
                                traci.trafficlight.setPhase(f'x{x}-y{y}', FEU_ROUGE_VERTICAL + 1)  # Passage au orange
                                cooldownStep[x - 1, y - 1] = 0

                    cooldownStep[x - 1, y - 1] += 1

            step += 1

    def feux_a_seuils_communicants_sans_anticipation_reseau_carre(self, config):
        """
        On détermine un seuil pour chaque feu du réseau. Lorsque le feu est rouge pour une voie du feu, et que le nombre de véhicules en attente et en course
        sur l'autre voie est supérieur ou égal au seuil, alors le feu passe au vert. Le feu doit rester vert un minimum de temps avant de pouvoir à nouveau changer.

        Ne fonctionne qu'avec un réseau carré.

        Paramètres à mettre dans la config :
        - "min_duration_tl" (int) : Le nombre de pas minimum de durée d'un feu
        - "max_duration_tl" (int) : Le nombre de pas maximum de durée d'un feu
        - "seuil_vehicules" (int) : Le nombre de véhicules en attente nécessaires pour déclencher un changement de feu. (sur une voie uniquement)
        - "nb_roads_by_side" (int) : Le nombre de feu sur un côté du réseau carré
        """

        FEU_ROUGE_HORIZONTAL = 2
        FEU_ROUGE_VERTICAL = 0

        min_duration_tl = config["min_duration_tl"]
        max_duration_tl = config["max_duration_tl"]
        nb_vehicules = config["seuil_vehicules"]
        nb_roads_by_side = config["nb_roads_by_side"]

        cooldownStep = np.zeros(
            (nb_roads_by_side, nb_roads_by_side))  # Nombre de pas depuis la dernière actualisation du feu
        step = 0

        while step < config['simulation_duration']:
            traci.simulationStep()

            for x in range(1, nb_roads_by_side + 1):
                for y in range(1, nb_roads_by_side + 1):

                    if cooldownStep[x - 1, y - 1] > min_duration_tl:

                        if traci.trafficlight.getPhase(f'x{x}-y{y}') == FEU_ROUGE_HORIZONTAL:

                            if traci.lanearea.getJamLengthVehicle(f'detector_x{x - 1}-y{y}_x{x}-y{y}') >= nb_vehicules \
                                    or traci.lanearea.getJamLengthVehicle(
                                f'detector_x{x + 1}-y{y}_x{x}-y{y}') >= nb_vehicules \
                                    or cooldownStep[x - 1, y - 1] > max_duration_tl:
                                traci.trafficlight.setPhase(f'x{x}-y{y}', FEU_ROUGE_HORIZONTAL + 1)  # Passage au orange
                                cooldownStep[x - 1, y - 1] = 0


                        elif traci.trafficlight.getPhase(f'x{x}-y{y}') == FEU_ROUGE_VERTICAL:

                            if traci.lanearea.getJamLengthVehicle(f'detector_x{x}-y{y + 1}_x{x}-y{y}') >= nb_vehicules \
                                    or traci.lanearea.getJamLengthVehicle(
                                f'detector_x{x}-y{y - 1}_x{x}-y{y}') >= nb_vehicules \
                                    or cooldownStep[x - 1, y - 1] > max_duration_tl:
                                traci.trafficlight.setPhase(f'x{x}-y{y}', FEU_ROUGE_VERTICAL + 1)  # Passage au orange
                                cooldownStep[x - 1, y - 1] = 0

                    cooldownStep[x - 1, y - 1] += 1

            step += 1
