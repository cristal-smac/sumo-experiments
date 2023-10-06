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

    nb_exp = len(lane_length)**4 * len(speed_values) * len(flux_values)**3 * len(gt_values)**4
    i = 0

    network = TwoCrossroadsNetwork()
    
    for middle_len in lane_length:
        for len_horizontal in lane_length:
            for len_vertical1 in lane_length:
                for len_vertical2 in lane_length:
                    for max_speed in speed_values:
                        for flux_horizontal in flux_values:
                            for flux_vertical1 in flux_values:
                                for flux_vertical2 in flux_values:
                                    for gt_horizontal1 in gt_values:
                                        for gt_horizontal2 in gt_values:
                                            for gt_vertical1 in gt_values:
                                                for gt_vertical2 in gt_values:

                                                    e = Experiment(f'exp_carrefour_double_tous_parametres_variables{i}',
                                                                   network=network.generate_infrastructures,
                                                                   routes=network.generate_flows_only_ahead)

                                                    e.set_variable('lane_length', 100)
                                                    e.set_variable('west_length', len_horizontal)
                                                    e.set_variable('east_length', len_horizontal)
                                                    e.set_variable('north_1_length', len_vertical1)
                                                    e.set_variable('south_1_length', len_vertical1)
                                                    e.set_variable('north_2_length', len_vertical2)
                                                    e.set_variable('south_2_length', len_vertical2)
                                                    e.set_variable('center_length', middle_len)

                                                    e.set_variable('stop_generation_time', 900)
                                                    e.set_variable('flow_density_west', flux_horizontal)
                                                    e.set_variable('flow_density_east', flux_horizontal)
                                                    e.set_variable('flow_density_north_1', flux_vertical1)
                                                    e.set_variable('flow_density_south_1', flux_vertical1)
                                                    e.set_variable('flow_density_north_2', flux_vertical2)
                                                    e.set_variable('flow_density_south_2', flux_vertical2)

                                                    e.set_variable('max_speed', max_speed)

                                                    e.set_variable('green_time_west_east_1', gt_horizontal1)
                                                    e.set_variable('green_time_north_south_1', gt_vertical1)
                                                    e.set_variable('green_time_west_east_2', gt_horizontal2)
                                                    e.set_variable('green_time_north_south_2', gt_vertical2)
                                                    e.set_variable('yellow_time', 3)

                                                    e.set_simulation_time(3601)

                                                    e.run()

                                                    e.export_results_to_csv('../../csv/exp_carrefour_double_tous_parametres_variables.csv', sampling_rate=900)

                                                    e.cleanFiles(delete_summary=True, delete_queue=True)

                                                    i += 1
                                                    print(f'{i}/{nb_exp} => {round((i/nb_exp)*100, 4)}%')