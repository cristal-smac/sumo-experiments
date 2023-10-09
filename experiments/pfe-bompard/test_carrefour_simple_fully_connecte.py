import sys, os

from core.src.experiment import Experiment
from core.src.util import import_flows_parameters_from_csv
from core.src.preset_networks import OneCrossroadNetwork

sys.path.append(os.path.join(sys.path[0],'..'))


if __name__ == '__main__':
    """
    Expérience dans laquelle on teste simplement le réseau carré implémenté dans networks.py.
    Les véhicules doivent être capable d'arriver de n'importe quelle direction et de pouvoir aller dans n'importe laquelle.
    """

    params_file = "../../csv/parameters/simple_carrefour.csv"

    flow_value = 100
    lane_length = 100
    gt_value = 30
    yt_value = 3
    speed_value = 30
    square_side_length = 2

    network = OneCrossroadNetwork()

    e = Experiment(f'test_reseau_fully_connected', network=network.generate_infrastructures, flows=network.generate_flows_with_matrix)

    load_vector, coeff_matrix = import_flows_parameters_from_csv(params_file)

    period_time = 300       # Nombre de ticks par période de flux

    # Variables de flux
    e.set_parameter('coeff_matrix', coeff_matrix)
    e.set_parameter('load_vector', load_vector)
    e.set_parameter("params_file", params_file)
    e.set_parameter('period_time', period_time)

    e.set_parameter('lane_length', lane_length)
    e.set_parameter('max_speed', speed_value)
    e.set_parameter('green_time', gt_value)
    e.set_parameter('yellow_time', yt_value)
    e.set_parameter('flow_density', flow_value)

    e.set_parameter('simulation_duration', 3601)

    e.run(gui=True)

    e.clean_files()