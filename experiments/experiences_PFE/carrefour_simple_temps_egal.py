import sys, os

from core.src.experiment import Experiment
from core.src.util import import_flows_parameters_from_csv
from core.src.preset_networks import OneCrossroadNetwork

sys.path.append(os.path.join(sys.path[0],'..'))


if __name__ == '__main__':
    """
    Expérience visant à comparer différentes métriques sur un carrefour simple selon la stratégie de feux, et d'autres paramètres.*

    TRES_PEU_DENSE = 75 v/h sur une voie
    PEU_DENSE = 150 v/h sur une voie
    MOYENNEMENT_DENSE = 225 v/h sur une voie
    DENSE = 300 v/h sur une voie
    TRES_DENSE = 375 v/h sur une voie

    """


    matrices_flux = ["../../csv/parameters/simple_carrefour/homogene.csv",
                    "../../csv/parameters/simple_carrefour/heterogene1.csv",
                    "../../csv/parameters/simple_carrefour/heterogene2.csv"]
    lane_length = 100
    gt_value = [10, 20, 30, 40, 50, 60]
    yt_value = 3
    speed_value = 30

    params_file = "../../csv/parameters/test_densite_simple_carrefour.csv"

    network = OneCrossroadNetwork()

    e = Experiment(f'test_reseau_fully_connected',
                   network=network.simple_crossroad_fully_connected_network,
                   routes=network.simple_crossroad_fully_connected_multi_flow,
                   detectors=network.no_detectors)


    load_vector, coeff_matrix = import_flows_parameters_from_csv(params_file)

    nb_ticks = 300       # Nombre de ticks par période de flux

    # Variables de flux
    e.set_variable('coeff_matrix', coeff_matrix)
    e.set_variable('load_vector', load_vector)
    e.set_variable("params_file", params_file)
    e.set_variable('nb_ticks', nb_ticks)
    
    e.set_variable('default_len', lane_length)
    e.set_variable('default_speed', speed_value)
    e.set_variable('default_green_time', gt_value[2])
    e.set_variable('default_yellow_time', yt_value)

    e.set_simulation_time(nb_ticks * (len(load_vector) + 1) + 1) # x * nb_ticks du durée + 1* nb_ticks pour observer la dernière configuration

    e.run(gui=True)

    e.cleanFiles(delete_summary=True, delete_queue=True)