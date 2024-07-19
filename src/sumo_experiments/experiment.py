import os

import traci

class Experiment:
    """
    This class creates and runs experiments with the SUMO simulator.
    The aim of this class is to easily generate all the configuration files to run a SUMO simulation.
    It also provides some useful functions to manage configuration files, and export data from simulations.

    To run an experiment, you need to define a function that generates the infrastructures for a network,
    and another that generates the flows. You can also add some optional functions generating optional
    elements of the network. If you don't want to define a full network, some preset networks are
    available, but you need to define the parameters to use it. After this, you can run the simulation
    with the function run that will build config files and launch SUMO. You can also run a simulation with Traci
    if you define a strategy that controls the infrastructures simulation step by simulation step. We highly
    recommand to use the clean files function after your experiments, to clean the repository from all
    configuration files.
    """

    def __init__(self, name, flows=None, net=None, infrastructures=None, detectors=None, full_line_command=None):
        """
        Init of class.
        :param name: The name of the experiment
        :type name: str
        :param flows: The flows of the network.
        :type flows: FlowBuilder
        :param net: The name of the file that define the network to use. Can not be used if infrastructures is set.
        :type net: str
        :param infrastructures: The infrastructures of the network. Can not be used if net is set.
        :type infrastructures: InfrastructureBuilder
        :param detectors: The function that creates network detectors. Must return a DetectorBuilder object.
        :type detectors: DetectorBuilder
        :param full_line_command: The full line command to run the sim. Don't use if not required. Override all the other params (except name).
        :type full_line_command: str
        """
        assert net is None or infrastructures is None, "infrastructures and net parameters can not both be set."
        self.infrastructures = infrastructures
        self.net = net
        self.flows = flows
        self.detectors = detectors
        self.name = name
        self.files = {}
        self.full_line_command = full_line_command

    def run(self, simulation_duration, gui=False, seed=None, no_warnings=True, nb_threads=1):
        """
        Launch an SUMO simulation with the network configuration.
        First build the configuration files and then launch SUMO.
        :param simulation_duration: The simulation duration in simulation steps
        :type simulation_duration: int
        :param gui: True to run SUMO in graphical mode. False otherwise.
        :type gui: bool
        :param seed: The seed of the simulation. Same seeds = same simulations.
        :type seed: int
        :param no_warnings: If set to True, no warnings when executing SUMO.
        :type no_warnings: bool
        :param nb_threads: Number of thread to run SUMO
        :type nb_threads: int
        """
        if self.full_line_command is None:
            self.generate_file_names()
            if not isinstance(self.flows, str):
                self.flows.build(self.files)
            else:
                self.files['routes'] = self.flows
            if self.infrastructures is not None:
                self.infrastructures.build(self.files, no_warnings)
            if self.detectors is not None:
                self.detectors.build(self.files)

            if self.net is None:
                cmd = f'$SUMO_HOME/bin/netconvert -n {self.files["nodes"]} -e {self.files["edges"]} -x {self.files["connections"]} -i {self.files["trafic_light_programs"]} -t {self.files["types"]} -o {self.files["network"]}'
                if no_warnings:
                    cmd += ' --no-warnings'
                os.system(cmd)
            args = self.build_arguments(simulation_duration, seed, no_warnings, nb_threads)

            if gui:
                os.system(f'$SUMO_HOME/bin/sumo-gui {args}')
            else:
                os.system(f'$SUMO_HOME/bin/sumo {args}')
        else:
            os.system(self.full_line_command + ' --time-to-teleport -1')

    def run_traci(self, traci_function, gui=False, seed=None, no_warnings=True, nb_threads=1):
        """
        Launch an SUMO simulation with the network configuration.
        First build the configuration files and then launch SUMO.
        This function uses TraCi, which means that the infrastructures can be controlled
        simulation steps by simpulation steps. So a function using TraCi must be defined
        to run SUMO with TraCi.
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
        """
        if self.full_line_command is None:
            self.generate_file_names()
            self.flows.build(self.files)
            self.infrastructures.build(self.files)
            if self.detectors is not None:
                self.detectors.build(self.files)
            cmd = f'$SUMO_HOME/bin/netconvert -n {self.files["nodes"]} -e {self.files["edges"]} -x {self.files["connections"]} -i {self.files["trafic_light_programs"]} -t {self.files["types"]} -o {self.files["network"]}'
            if no_warnings:
                cmd += ' --no-warnings'
            os.system(cmd)
            args = self.build_arguments(None, seed, no_warnings, nb_threads)

            if gui:
                traci.start(["sumo-gui"] + args.split())
            else:
                traci.start(["sumo"] + args.split())
        else:
            traci.start((self.full_line_command + ' --time-to-teleport -1').split())

        res = traci_function()

        traci.close()

        return res

    def clean_files(self, except_trip_info=False, except_emission=False):
        """
        Delete simulation config files.
        :param except_trip_info: Doesn't delete trip info csv file if True
        :type except_trip_info: bool
        """
        for file in self.files.values():
            if os.path.exists(file):
                os.remove(file)

    def build_arguments(self, simulation_duration, seed, no_warnings, nb_threads):
        """
        Build the arguments to launch SUMO with a command line.
        """
        args = ''
        if self.net is None:
            args += f'-n {self.files["network"]} '
        else:
            args += f'-n {self.net} '
        args += f'-r {self.files["routes"]} '
        if self.detectors is not None:
            args += f'-a {self.files["detectors"]} '
        if simulation_duration is not None:
            args += f'-e {simulation_duration + 1} '
        if seed is not None:
            args += f'--seed {seed} '
        else:
            args += '--random '
        if no_warnings:
            args += '--no-warnings '
        args += f'--threads {nb_threads} '
        args += '--time-to-teleport -1 '
        return args

    def generate_file_names(self):
        """
        Generates file names for all SUMO config files.
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
            'detectors_out': 'detectors.out',
            'additionnals': f'{self.name}.add.xml'
        }



