import sys, os
sys.path.append(os.path.join(sys.path[0],'..'))
from core.src.experiment import Experiment
from core.src.util import import_flows_parameters_from_csv
from core.src.preset_networks import TwoCrossroadsNetwork

if __name__ == '__main__':
    """
    Expérience dans laquelle on teste simplement le réseau à double carrefour fully connected implémenté dans networks.py.
    Les véhicules doivent être capable d'arriver de n'importe quelle direction et de pouvoir aller dans n'importe laquelle.
    """

    params_file = "../../csv/parameters/double_carrefour.csv"

    lane_length = 100
    gt_value = 30
    yt_value = 3
    speed_value = 30

    network = TwoCrossroadsNetwork()

    e = Experiment('test_multi_flux', network=network.generate_infrastructures, flows=network.generate_flows_with_matrix)

    e.set_parameter('lane_length', lane_length)
    e.set_parameter('max_speed', speed_value)
    e.set_parameter('green_time', gt_value)
    e.set_parameter('yellow_time', yt_value)

    load_vector, coeff_matrix = import_flows_parameters_from_csv(params_file)

    nb_ticks = 300       # Nombre de ticks par période de flux

    e.set_parameter('coeff_matrix', coeff_matrix)
    e.set_parameter('load_vector', load_vector)
    e.set_parameter('period_time', nb_ticks)

    e.run(gui=True)

    e.export_results_to_csv('../../csv/exp_carrefour_double_fully_connected.csv', sampling_rate=600)

    e.clean_files()