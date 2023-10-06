import sys, os
sys.path.append(os.path.join(sys.path[0],'..'))
from core.src.experiment import Experiment
from core.src.preset_networks import TwoCrossroadsNetwork

if __name__ == '__main__':
    """
    Expérience dans laquelle on a deux carrefours simples consécutifs : Ouest -> Est et deux fois Sud -> Nord.
    Nous faisons varier les flux de véhicules, la longueur des voies, les temps de changement de feux, les vitesses maximales.
    Les véhicules ne peuvent pas tourner au carrefour, ils peuvent seulement continuer leur route.
    """
    flux_values = [100, 300, 600]
    lane_length = [100, 300, 600]
    gt_values = [10, 30, 60, 90, 120]
    speed_values = [30]

    nb_exp = len(lane_length)**4 * len(speed_values)**3 * len(flux_values)**3 * len(gt_values)**4
    i = 0

    network = TwoCrossroadsNetwork()
    
    for middle_len in lane_length:
        for len_horizontal in lane_length:
            for len_vertical1 in lane_length:
                for len_vertical2 in lane_length:
                    for speed_horizontal in speed_values:
                        for speed_vertical1 in speed_values:
                            for speed_vertical2 in speed_values:
                                for flux_horizontal in flux_values:
                                    for flux_vertical1 in flux_values:
                                        for flux_vertical2 in flux_values:
                                            for gt_horizontal1 in gt_values:
                                                for gt_horizontal2 in gt_values:
                                                    for gt_vertical1 in gt_values:
                                                        for gt_vertical2 in gt_values:

                                                            e = Experiment(f'exp_carrefour_double_tous_parametres_variables{i}',
                                                                           network=network.double_crossroad_network,
                                                                           routes=network.double_crossroad_routes,
                                                                           detectors=network.no_detectors)
                                    
                                                            e.set_variable('default_len', 100)
                                                            e.set_variable('w_e_len', len_horizontal)
                                                            e.set_variable('s1_n1_len', len_vertical1)
                                                            e.set_variable('s2_n2_len', len_vertical2)
                                                            e.set_variable('middle_len', middle_len)

                                                            e.set_variable('stop_generation_time', 900)
                                                            e.set_variable('w_e_flow', flux_horizontal)
                                                            e.set_variable('s1_n1_flow', flux_vertical1)
                                                            e.set_variable('s2_n2_flow', flux_vertical2)

                                                            e.set_variable('w_e_speed', speed_horizontal)
                                                            e.set_variable('s1_n1_speed', speed_vertical1)
                                                            e.set_variable('s2_n2_speed', speed_vertical2)

                                                            e.set_variable('w1_e1_green_time', gt_horizontal1)
                                                            e.set_variable('s1_n1_green_time', gt_vertical1)
                                                            e.set_variable('w2_e2_green_time', gt_horizontal2)
                                                            e.set_variable('s2_n2_green_time', gt_vertical2)
                                                            e.set_variable('default_yellow_time', 3)

                                                            e.set_simulation_time(3601)

                                                            e.run()

                                                            e.export_results_to_csv('../../csv/exp_carrefour_double_tous_parametres_variables.csv', sampling_rate=900)

                                                            e.cleanFiles(delete_summary=True, delete_queue=True)   
                                    
                                                            i += 1
                                                            print(f'{i}/{nb_exp} => {(i/nb_exp)*100}%')