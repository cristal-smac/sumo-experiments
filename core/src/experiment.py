import os, sys
import pandas as pd
import numpy as np
from os.path import exists
tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
sys.path.append(tools)
import traci

class Experiment:
    """
    Classe qui représente un expérience à réaliser avec SUMO
    """
    def __init__(self, name, network, routes, detectors):
        """
        Constructeur de la classe Experiment
        :param name: Nom de l'expérience
        :param network: Réseau de l'expérience (classe Network)
        :param routes: Routes de l'expérience (classe Routes)
        :param detectors: Détecteurs de l'expérience
        """
        self.network = network
        self.routes = routes
        self.detectors = detectors
        self.name = name
        self.config = {'nom_exp':name}
        self.files = {}

    def run(self, gui = False, seed = None):
        """
        Lance l'expérience
        """
        self.generateFileNames()
        self.routes(self.config).build(self.files)
        self.network(self.config).build(self.files)
        self.detectors(self.config).build(self.files)
        os.system(f'$SUMO_HOME/bin/netconvert -n {self.files["nodes"]} -e {self.files["edges"]} -x {self.files["connections"]} -i {self.files["trafic_light_programs"]} -t {self.files["types"]} -o {self.files["network"]}')
        args = self.buildArguments()
        if gui:
            if seed is None:
                os.system(f'$SUMO_HOME/bin/sumo-gui {args} --random')
            else:
                os.system(f'$SUMO_HOME/bin/sumo-gui {args} --seed {seed}')
        else:
            if seed is None:
                os.system(f'$SUMO_HOME/bin/sumo {args} --random')
            else:
                os.system(f'$SUMO_HOME/bin/sumo {args} --seed {seed}')
        os.system(f'python3 $SUMO_HOME/tools/xml/xml2csv.py {self.files["queuexml"]} --separator ","')
        os.system(f'python3 $SUMO_HOME/tools/xml/xml2csv.py {self.files["summaryxml"]} --separator ","')



    def run_traci(self, fonction_traci, gui=False):
        """
        Lance l'expérience avec l'API Traci
        Le comportement de Traci est choisi par la fonction "fonction_traci".
        Si le nombre de véhicule en attente dépasse 6, le feu passe au vert.
        """
        self.generateFileNames()
        self.routes(self.config).build(self.files)
        self.network(self.config).build(self.files)
        self.detectors(self.config).build(self.files)
        os.system(f'$SUMO_HOME/bin/netconvert -n {self.files["nodes"]} -e {self.files["edges"]} -x {self.files["connections"]} -i {self.files["trafic_light_programs"]} -t {self.files["types"]} -o {self.files["network"]}')
        args = self.buildArguments()

        # Spécifique Traci
        if gui:
            traci.start(["sumo-gui"] + args.split())
        else:
            traci.start(["sumo"] + args.split())
            
        fonction_traci(self.config)

        traci.close()

        os.system(f'python3 $SUMO_HOME/tools/xml/xml2csv.py {self.files["queuexml"]} --separator ","')
        os.system(f'python3 $SUMO_HOME/tools/xml/xml2csv.py {self.files["summaryxml"]} --separator ","')



    def cleanFiles(self, delete_summary=False, delete_queue=False):
        """
        Supprime les fichiers de l'expérience
        :param delete_summary: Supprime le fichier de sortie summary de l'expérience si True
        :param delete_queue: Supprime le fichier de sortie queue de l'expérience si True
        """
        for file in self.files.values():
            if os.path.exists(file):
                if (file == self.files['summaryxml'] or file == self.files['summarycsv']):
                    if delete_summary:
                        os.remove(file)
                elif (file == self.files['queuexml'] or file == self.files['queuecsv']):
                    if delete_queue:
                        os.remove(file)
                else:
                    os.remove(file)

    def buildArguments(self):
        """
        Construit les arguments de lancement de SUMO
        """
        args = ''
        args += f'-n {self.files["network"]} '
        args += f'-r {self.files["routes"]} '
        args += f'-a {self.files["detectors"]} '
        args += f'--summary {self.files["summaryxml"]} '
        args += f'--queue-output {self.files["queuexml"]} '
        if 'simulation_duration' in self.config:
            args += f'-e {self.config["simulation_duration"]} '
        return args

    def set_variable(self, key, value):
        self.config[key] = value
        return self

    def set_simulation_time(self, value):
        self.config['simulation_duration'] = value
        return self

    def exportResultsToFile(self, filename, sampling_rate):
        """
        Exporte les résultats de l'expérience dans un fichier CSV
        :param filename: Nom du fichier CSV
        :param sampling_rate: le nombre de secondes entre deux mesures
        """
        summary_data = pd.read_csv(self.files["summarycsv"])
        # queue_data = pd.read_csv(self.files["queuecsv"])


        # queue_data.replace('edge_sc_0', 1, inplace=True)
        # queue_data.replace('edge_wc_0', 2, inplace=True)
        # queue_data.replace(':c_1_0', 3, inplace=True)
        # queue_data.replace(np.nan, 0, inplace=True)
        # queue_data['lane_id'] = pd.to_numeric(queue_data['lane_id'])

        # queue_data.drop_duplicates(subset=['data_timestep'], keep='first', inplace=True)

        samples = np.arange(start=sampling_rate, stop=summary_data.shape[0], step=sampling_rate)

        if not exists(filename):
            f = open(filename, 'w')

            # Columns titles
            for key in self.config:
                # Pour éviter les matrices dans le fichier de résultats
                if not (key == "coeff_matrix" or key == "load_vector"):
                    f.write(f'{key},')
            for s in samples:
                f.write(f'loaded_{s},inserted_{s},running_{s},waiting_{s},ended_{s},halting_{s},stopped_{s},meanWaitingTime_{s},meanTravelTime_{s},meanSpeed_{s}')
                if s != samples[-1]:
                    f.write(',')

            f.write('\n')

        with open(filename, 'a') as f:
            # Data
            for key in self.config:
                if not (key == "coeff_matrix" or key == "load_vector"):
                    f.write(f'{self.config[key]},')
            
            # flow_ratio = self.config["w_e_flow"] / self.config["s_n_flow"]
            # gt_ratio = self.config["w_e_green_time"] / self.config["s_n_green_time"]

            # if math.isclose(flow_ratio, gt_ratio, rel_tol=0, abs_tol=0.11):
            #     f.write(f'1,')
            # else:
            #     f.write(f'0,')

            for s in samples:
                loaded = summary_data.loc[s]['step_loaded']
                inserted = summary_data.loc[s]['step_inserted']
                running = summary_data.loc[s]['step_running']
                waiting = summary_data.loc[s]['step_waiting']
                ended = summary_data.loc[s]['step_ended']
                halting = summary_data.loc[s]['step_halting']
                stopped = summary_data.loc[s]['step_stopped']
                meanWaitingTime = summary_data.loc[s]['step_meanWaitingTime']
                meanTravelTime = summary_data.loc[s]['step_meanTravelTime']
                meanSpeed = summary_data.loc[s]['step_meanSpeed']
                f.write(f'{loaded},{inserted},{running},{waiting},{ended},{halting},{stopped},{meanWaitingTime},{meanTravelTime},{meanSpeed}')
                if s != samples[-1]:
                    f.write(',')

            f.write('\n')

    def generateFileNames(self):
        """
        Génère les noms des fichiers qui seront générés par le programme
        Les noms sont sauvegardés dans l'attribut files
        """
        self.files = {
            'nodes': f'{self.name}.nod.xml',
            'edges': f'{self.name}.edg.xml',
            'types': f'{self.name}.typ.xml',
            'connections': f'{self.name}.con.xml',
            'trafic_light_programs': f'{self.name}.ttl.xml',
            'routes': f'{self.name}.rou.xml',
            'network': f'{self.name}.net.xml',
            'detectors': f'{self.name}.det.xml',
            'detectors': f'{self.name}.det.xml',
            'summaryxml': f'summary_{self.name}.xml',
            'summarycsv': f'summary_{self.name}.csv',
            'queuexml': f'queue_{self.name}.xml',
            'queuecsv': f'queue_{self.name}.csv',
        }



