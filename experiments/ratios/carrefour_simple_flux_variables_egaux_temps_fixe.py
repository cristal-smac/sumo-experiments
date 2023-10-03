import sys, os
sys.path.append(os.path.join(sys.path[0],'..'))
from core.src.experiment import Experiment
from core.src.preset_networks import OneCrossroadNetwork
import os

if __name__ == '__main__':
    """
    Expérience dans laquelle on a un carrefour avec deux directions possibles : Ouest -> Est et Sud -> Nord.
    Nous faisons varier uniquement les flux de véhicules.
    """
    output_files = ''
    legend = ''

    network = OneCrossroadNetwork()

    for i in range (11):
        if i != 0:
            output_files += ','
            legend += ','

        t_g = 30

        n_vehicles_h = 100 + 50 * i

        e = Experiment(f'exp_carrefour_simple_flux_variables_egaux_temps_fixe{i}',
                       network=network.simple_crossroad_network,
                       routes=network.simple_crossroad_routes,
                       additionals=network.no_additionals)

        e.set_variable('stop_generation_time', 900)
        e.set_variable('w_e_flow', n_vehicles_h)
        e.set_variable('s_n_flow', n_vehicles_h)

        e.set_variable('default_len', 100)
        e.set_variable('w_e_len', 100)
        e.set_variable('s_n_len', 100)

        e.set_variable('w_e_speed', 15)
        e.set_variable('s_n_speed', 15)

        e.set_variable('w_e_green_time', t_g)
        e.set_variable('s_n_green_time', t_g)
        e.set_variable('default_yellow_time', 5)

        e.run()

        e.cleanFiles(delete_summary=False, delete_queue=True)

        output_files += f'{e.files["summaryxml"]}'
        legend += f'"Flux = {n_vehicles_h}"'

    os.system('mkdir -p ../../img/flux_variables_egaux_temps_fixe/')
    os.system(f'python $SUMO_HOME/tools/visualization/plot_summary.py -i {output_files} -m meanTravelTime --title "Temps de parcours moyen avec Tv = {t_g}" -o ../../img/flux_variables_egaux_temps_fixe/flux_variables_temps_egaux.png -l {legend}')