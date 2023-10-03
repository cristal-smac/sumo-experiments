import sys, os
sys.path.append(os.path.join(sys.path[0],'..'))
from core.src.experiment import Experiment
from core.src.preset_networks import OneCrossroadNetwork

if __name__ == '__main__':
    """
    Expérience dans laquelle on a un carrefour avec deux directions possibles : Ouest -> Est et Sud -> Nord.
    Nous faisons varier les flux de véhicules, la longueur des voies, les temps de changement de feux, les vitesses maximales.
    Dans cette expérience, les véhicules ont la possibilité de tourner au carrefour.
    Nous faisons donc varier la proportion de véhicules qui ont la possibilité de tourner au carrefour.
    """
    flux_values = [100, 300, 600, 900]
    lane_length = [50, 150, 300, 600]
    gt_values = [10, 30, 60, 90, 120, 240]
    speed_values = [15, 30, 50]
    flows_proportion = [0.1, 0.3, 0.5, 0.7, 0.9]

    nb_exp = len(lane_length) * len(lane_length) *  len(speed_values) * len(speed_values) * len(flux_values) * len(flux_values) * len(gt_values) * len(gt_values)
    i = 0

    network = OneCrossroadNetwork()

    for len_horizontal in lane_length:
        for len_vertical in lane_length:
            for speed_horizontal in speed_values:
                for speed_vertical in speed_values:
                    for flux_horizontal in flux_values:
                        for flux_vertical in flux_values:
                            for proportion_horizontal in flows_proportion:
                                for proportion_vertical in flows_proportion:
                                    for gt_horizontal in gt_values:
                                        for gt_vertical in gt_values:

                                            e = Experiment(f'exp_carrefour_simple_tourner_{i}',
                                                           network=network.simple_crossroad_turn_network,
                                                           routes=network.simple_crossroad_turn_routes,
                                                           additionals=network.no_additionals)
                                            
                                            e.set_variable('default_len', 100)
                                            e.set_variable('w_e_len', len_horizontal)
                                            e.set_variable('s_n_len', len_vertical)

                                            e.set_variable('stop_generation_time', 900)
                                            e.set_variable('w_e_straight_flow', flux_horizontal * proportion_horizontal)
                                            e.set_variable('w_e_right_flow', flux_horizontal * (1 - proportion_horizontal))

                                            e.set_variable('s_n_straight_flow', flux_vertical * proportion_vertical)
                                            e.set_variable('s_n_left_flow', flux_vertical * (1 - proportion_vertical))

                                            e.set_variable('w_e_speed', speed_horizontal)
                                            e.set_variable('s_n_speed', speed_vertical)

                                            e.set_variable('w_e_green_time', gt_horizontal)
                                            e.set_variable('s_n_green_time', gt_vertical)
                                            e.set_variable('default_yellow_time', 3)

                                            e.set_simulation_time(3601)

                                            e.run()

                                            e.exportResultsToFile('../../csv/exp_carrefour_simple_tourner.csv', sampling_rate=900)

                                            e.cleanFiles(delete_summary=True, delete_queue=True)

                                            i += 1
                                            print(f'{i}/{nb_exp} => {(i/nb_exp)*100}%')

