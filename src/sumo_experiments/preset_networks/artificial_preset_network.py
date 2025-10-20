import os
import sys

from sumo_experiments.preset_networks import Network
import libsumo as traci


class ArtificialNetwork(Network):
    """
    Abstract class for artificial preset networks.
    """

    def __init__(self, name):
        """
        Generates file names for all SUMO config files.
        """
        super().__init__()
        self.file_names = {
            'nodes': f'{name}.nod.xml',
            'edges': f'{name}.edg.xml',
            'types': f'{name}.typ.xml',
            'connections': f'{name}.con.xml',
            'trafic_light_programs': f'{name}.ttl.xml',
            'routes': f'{name}.rou.xml',
            'network': f'{name}.net.xml',
            'detectors': f'{name}.det.xml',
            'detectors_out': 'detectors.out',
            'additionnals': f'{name}.add.xml'
        }

    def run(self, traci_function, simulation_duration=None, gui=False, seed=None, no_warnings=True, nb_threads=1, time_to_teleport=150):
        try:
            args = self.build_arguments(simulation_duration, seed, no_warnings, nb_threads, time_to_teleport)
            if gui:
                traci.start(["sumo-gui"] + args.split())
            else:
                traci.start(["sumo"] + args.split())
            res = traci_function(traci)
            traci.close()
        except Exception as err:
            print("Error during simulation :", sys.exc_info()[0])
            print("OS error: {0}".format(err))
            res = None
        self.clean_files()
        return res

    def build_arguments(self, simulation_duration, seed, no_warnings, nb_threads, time_to_teleport):
        """
        Build the arguments to launch SUMO with a command line.
        """
        args = ''
        args += f'-n {self.file_names["network"]} '
        args += f'-r {self.file_names["routes"]} '
        args += f'-a {self.file_names["detectors"]} '
        # if simulation_duration is not None:
        #     args += f'-e {simulation_duration + 1} '
        if seed is not None:
            args += f'--seed {seed} '
        else:
            args += '--random '
        if no_warnings:
            args += '--no-warnings '
        args += f'--threads {nb_threads} '
        args += f'--time-to-teleport {time_to_teleport}'
        return args

    def clean_files(self):
        """
        Clean all simulation files.
        """
        for file in self.file_names.values():
            if os.path.exists(file):
                os.remove(file)