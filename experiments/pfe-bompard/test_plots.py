import sys, os
sys.path.append(os.path.join(sys.path[0],'..'))
from core.src.experiment import Experiment
from core.src.util import import_flows_parameters_from_csv
from core.src.preset_networks import OneCrossroadNetwork
from core.src.plot import ResultsPlotter

if __name__ == '__main__':

    if os.path.exists("../../csv/exp_plots.csv"):
        os.remove("../../csv/exp_plots.csv")

    params_file = "../../csv/parameters/simple_carrefour.csv"

    lane_length = 100
    gt_value = 30
    yt_value = 3
    speed_value = 30
    seuils = [i for i in range(3, 10)]

    network = OneCrossroadNetwork()

    for seuil in seuils:

        e = Experiment(f'test_detecteurs_seuil_{seuil}',
                       network=network.generate_infrastructures,
                       routes=network.generate_flows_with_matrix,
                       detectors=network.generate_numerical_detectors)

        # Variables de réseau
        e.set_variable('lane_length', lane_length)
        e.set_variable('max_speed', speed_value)
        e.set_variable('green_time', gt_value)
        e.set_variable('yellow_time', yt_value)
        e.set_variable('boolean_detector_length', 7.5)

        load_vector, coeff_matrix = import_flows_parameters_from_csv(params_file)

        nb_ticks = 300       # Nombre de ticks par période de flux

        # Variables de flux
        e.set_variable('coeff_matrix', coeff_matrix)
        e.set_variable('load_vector', load_vector)
        e.set_variable('period_time', nb_ticks)

        # Temps de simulation
        e.set_simulation_time(nb_ticks * (len(load_vector) + 1) + 1) # x * nb_ticks du durée + 1* nb_ticks pour observer la dernière configuration

        # Variables Traci
        e.set_variable('min_duration_tl', 10)
        e.set_variable('max_duration_tl', 30)
        e.set_variable('vehicle_threshold', seuil)

        e.run_traci(network.numerical_detection_all_vehicles)

        e.export_results_to_csv("../../csv/exp_plots.csv", sampling_rate=50)

        e.cleanFiles(delete_summary=True, delete_queue=True)

    rp = ResultsPlotter("../../csv/exp_plots.csv", sampling_rate=50, max_ticks=(nb_ticks * (len(load_vector) + 1)), save_folder="../../img/test")
    rp.plot_all(3, 3)
