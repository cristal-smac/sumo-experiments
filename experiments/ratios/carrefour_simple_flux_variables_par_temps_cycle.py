import sys, os
sys.path.append(os.path.join(sys.path[0],'..'))
from core.src.experiment import Experiment
from core.src.preset_networks import OneCrossroadNetwork
import os

if __name__ == '__main__':
    """
    Expérience dans laquelle on fait varier le flux Sud -> Nord de 100 à 700 véhicules par heure
    Le flux horizontal est fixé à 400 véhicules par heure.
    Les temps de feu vert sont calculés de façon à convserver le ratio FluxSN/FluxWE = Tv/Tg.
    """
    flux = [100, 200, 300, 400, 500, 600, 700]
    temps_base = [10, 20, 30, 60, 120, 240]
    flux_horizontal = 400
    i = 0

    network = OneCrossroadNetwork()

    for flux_vertical in flux:

        output_files = ''
        legend = ''

        for temps_vert in temps_base:
            if temps_vert != temps_base[0]:
                output_files += ','
                legend += ','
            
            # Calcul des temps de feu vert sur les deux voies
            we_t = temps_vert * (flux_horizontal / (flux_vertical + flux_horizontal))
            sn_t = temps_vert * (flux_vertical / (flux_vertical + flux_horizontal))

            # Instanciation de l'expérience
            e = Experiment(f'carrefour_simple_flux_variables_par_temps{i}',
                           network=network.simple_crossroad_network,
                           routes=network.generate_flows_only_ahead,
                           detectors=network.no_detectors)

            # Configuration de l'expérience
            e.set_variable('stop_generation_time', 900)
            e.set_variable('w_e_flow', flux_horizontal)
            e.set_variable('s_n_flow', flux_vertical)

            e.set_variable('default_len', 100)
            e.set_variable('w_e_len', 100)
            e.set_variable('s_n_len', 100)
            e.set_variable('w_e_speed', 16.89)
            e.set_variable('s_n_speed', 16.89)

            e.set_variable('default_yellow_time', 3)
            e.set_variable('w_e_green_time', we_t)
            e.set_variable('s_n_green_time', sn_t)

            # Lancement de l'expérience
            e.run()

            e.cleanFiles(delete_summary=False, delete_queue=True)

            output_files += f'{e.files["summaryxml"]}'
            legend += f'"Temps cycle = {temps_vert}"'
            #legend += f'"Cycle duration = {temps_vert}"'

            i += 1

        os.system('mkdir -p ../../img/flux_variables_par_cycle/')
        os.system(f'python3 $SUMO_HOME/tools/visualization/plot_summary.py -i {output_files} -m meanTravelTime --title "Temps de parcours moyen avec flux de {flux_horizontal} et {flux_vertical}/h" -o ../../img/flux_variables_par_cycle/{flux_horizontal}_{flux_vertical}.png -l {legend}')
        output_files = output_files.replace(',', ' ')
        os.system(f'rm {output_files}')
        output_files = output_files.replace('.xml', '.csv')
        os.system(f'rm {output_files}')
