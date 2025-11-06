import os
import sys

from sumo_experiments.preset_networks import Network
import libsumo as traci
import traceback

class ArtificialNetwork(Network):
    """
    Abstract class for artificial preset networks.
    """

    def __init__(self, name):
        """
        Generates file names for all SUMO config files.
        :param name: name of the simulation, used to name the different simulation files
        :type name: str
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

    def run(self, traci_function, gui=False, seed=None, no_warnings=True, nb_threads=1, time_to_teleport=150):
        """
        Run the simulation.
        :param traci_function: The function using TraCi package and that can control infrastructures.
        :type: function
        :param gui: True to run SUMO in graphical mode. False otherwise.
        :type gui: bool
        :param seed: The seed of the simulation. Same seeds = same simulations.
        :type seed: int
        :param no_warnings: If set to True, no warnings when executing SUMO.
        :type no_warnings: bool
        :param nb_threads: Number of thread to run SUMO
        :type nb_threads: int
        :param time_to_teleport: The time for a vehicle to teleport when the network is blocked
        :type time_to_teleport: int
        """
        try:
            args = self.build_arguments(seed, no_warnings, nb_threads, time_to_teleport)
            if gui:
                traci.start(["sumo-gui"] + args.split() + ['--waiting-time-memory', '1000000'])
            else:
                traci.start(["sumo"] + args.split() + ['--waiting-time-memory', '1000000'])
            res = traci_function(traci)
            traci.close()
        except Exception as err:
            print("Error during simulation :", sys.exc_info()[0])
            print("OS error: {0}".format(err))
            print(traceback.format_exc())
            res = None
        self.clean_files()
        return res

    def build_arguments(self, seed, no_warnings, nb_threads, time_to_teleport):
        """
        Build the arguments to launch SUMO with a command line.
        """
        args = ''
        args += f'-n {self.file_names["network"]} '
        args += f'-r {self.file_names["routes"]} '
        args += f'-a {self.file_names["detectors"]} '
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

    def detector2tlid(self):
        mapping = {}
        for tlid, phases in self.TLS_DETECTORS.items():
            for phase, detectors in phases.items():
                # numerical detectors
                numerical = detectors['numerical']
                for detector in numerical:
                    mapping.setdefault(detector, [])
                    mapping[detector].append(tlid)
        return mapping