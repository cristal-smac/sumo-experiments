import sys, os
sys.path.append(os.path.join(sys.path[0],'..'))
from core.src.experiment import Experiment
from core.src.preset_networks import OneCrossroadNetwork

if __name__ == '__main__':
    """
    Expérience dans laquelle on a un carrefour avec deux directions possibles : Ouest -> Est et Sud -> Nord.
    Nous faisons varier les flux de véhicules, la longueur des voies, les temps de changement de feux, les vitesses maximales.
    """
    flux_values = [100, 300, 600, 900]
    lane_length = [50, 150, 300, 600]
    gt_values = [10, 30, 60, 90, 120, 240]
    speed_values = [15, 30, 50]

    couples_done = []

    nb_exp = len(lane_length) * len(lane_length) *  len(speed_values) * len(speed_values) * len(flux_values) * len(flux_values) * len(gt_values) * len(gt_values)
    i = 0

    network = OneCrossroadNetwork()

    for len_horizontal in lane_length:
        for len_vertical in lane_length:
            for speed_horizontal in speed_values:
                for speed_vertical in speed_values:
                    for flux_horizontal in flux_values:
                        for flux_vertical in flux_values:
                            for gt_horizontal in gt_values:
                                for gt_vertical in gt_values:

                                    couple1 = (len_horizontal, len_vertical, speed_horizontal, speed_vertical, flux_horizontal, flux_vertical, 3, gt_horizontal, gt_vertical)
                                    couple2 = (len_vertical, len_horizontal, speed_vertical, speed_horizontal, flux_vertical, flux_horizontal, 3, gt_vertical, gt_horizontal)

                                    if couple1 not in couples_done and couple2 not in couples_done:

                                        e = Experiment(f'exp_carrefour_simple_tous_parametres_variables{i}',
                                                       network=network.simple_crossroad_network,
                                                       routes=network.simple_crossroad_routes,
                                                       additionals=network.no_additionals)
                                        
                                        e.set_variable('default_len', 100)
                                        e.set_variable('w_e_len', len_horizontal)
                                        e.set_variable('s_n_len', len_vertical)

                                        e.set_variable('stop_generation_time', 900)
                                        e.set_variable('w_e_flow', flux_horizontal)
                                        e.set_variable('s_n_flow', flux_vertical)

                                        e.set_variable('w_e_speed', speed_horizontal)
                                        e.set_variable('s_n_speed', speed_vertical)

                                        e.set_variable('w_e_green_time', gt_horizontal)
                                        e.set_variable('s_n_green_time', gt_vertical)
                                        e.set_variable('default_yellow_time', 3)

                                        e.set_simulation_time(3601)

                                        e.run()

                                        e.exportResultsToFile('../../csv/exp_carrefour_simple_tous_parametres_variables.csv', sampling_rate=900)

                                        e.cleanFiles(delete_summary=True, delete_queue=True)

                                        couples_done.append(couple1)
                                        couples_done.append(couple2)
                                    
                                    i += 1
                                    print(f'{i}/{nb_exp} => {(i/nb_exp)*100}%')

