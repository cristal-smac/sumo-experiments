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

    nb_exp = len(lane_length) * len(lane_length) *  len(speed_values) * len(flux_values) * len(flux_values) * len(gt_values) * len(gt_values)
    i = 0

    network = OneCrossroadNetwork()

    for len_horizontal in lane_length:
        for len_vertical in lane_length:
            for max_speed in speed_values:
                for flux_horizontal in flux_values:
                    for flux_vertical in flux_values:
                        for gt_horizontal in gt_values:
                            for gt_vertical in gt_values:

                                e = Experiment(f'exp_carrefour_simple_tourner_{i}', network=network.generate_infrastructures, flows=network.generate_flows_all_directions)

                                e.set_parameter('west_length', len_horizontal)
                                e.set_parameter('east_length', len_horizontal)
                                e.set_parameter('north_length', len_vertical)
                                e.set_parameter('south_length', len_vertical)

                                e.set_parameter('stop_generation_time', 900)
                                e.set_parameter('flow_density_east', flux_horizontal)
                                e.set_parameter('flow_density_west', flux_horizontal)

                                e.set_parameter('flow_density_north', flux_vertical)
                                e.set_parameter('flow_density_south', flux_vertical)

                                e.set_parameter('max_speed', max_speed)

                                e.set_parameter('green_time_west_east', gt_horizontal)
                                e.set_parameter('green_time_north_south', gt_vertical)
                                e.set_parameter('yellow_time', 3)

                                e.set_parameter('simulation_duration', 3601)

                                e.run()

                                e.export_results_to_csv('../../csv/exp_carrefour_simple_tourner.csv', sampling_rate=900)

                                e.clean_files()

                                i += 1
                                print(f'{i}/{nb_exp} => {round((i/nb_exp)*100, 3)}%')

