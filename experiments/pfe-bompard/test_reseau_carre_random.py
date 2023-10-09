import sys, os
sys.path.append(os.path.join(sys.path[0],'..'))
from core.src.experiment import Experiment
from core.src.util import import_flows_parameters_from_csv
from core.src.preset_networks import SquareNetwork

if __name__ == '__main__':
    """
    Expérience dans laquelle on teste simplement le réseau carré implémenté dans networks.py.
    Les véhicules doivent être capable d'arriver de n'importe quelle direction et de pouvoir aller dans n'importe laquelle.
    """

    flow_value = 20
    lane_length = 100
    gt_value = 30
    yt_value = 3
    speed_value = 30
    square_side_length = 2
    params_file = "../../csv/parameters/grille_2x2.csv"

    network = SquareNetwork()

    e = Experiment(f'test_reseau_carre_random',
                   network=network.generate_random_infrastructures,
                   routes=network.generate_flows_with_matrix)

    load_vector, coeff_matrix = import_flows_parameters_from_csv(params_file)

    period_time = 900       # Nombre de ticks par période de flux

    # Variables de flux
    e.set_variable('coeff_matrix', coeff_matrix)
    e.set_variable('load_vector', load_vector)
    e.set_variable("params_file", params_file)
    e.set_variable('period_time', period_time)

    e.set_variable("square_side_length", square_side_length)
    e.set_variable("min_duration_tl", 10)
    e.set_variable("max_duration_tl", 30)
    e.set_variable("vehicle_threshold", 8)
    e.set_variable("flow_density", 100)
    e.set_variable('lane_length', lane_length)
    e.set_variable('max_speed', speed_value)
    e.set_variable('green_time', gt_value)
    e.set_variable('yellow_time', yt_value)

    e.set_simulation_time(period_time * (len(load_vector) + 1) + 1) # x * nb_ticks du durée + 1* nb_ticks pour observer la dernière configuration

    e.run(gui=True)

    e.cleanFiles(delete_summary=True, delete_queue=True)
    

