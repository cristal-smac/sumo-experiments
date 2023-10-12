import os, sys
import pandas as pd
import numpy as np
from os.path import exists

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


    def __init__(self, name, infrastructures, flows, detectors=None):
        """
        Init of class.
        :param name: The name of the experiment
        :type name: str
        :param infrastructures: The function that creates the network infrastructures. Must return a InfrastructureBuilder object.
        :type infrastructures: function
        :param flows: The function that creates network flows. Must return a FlowBuilder object.
        :type flows: function
        :param detectors: The function that creates network detectors. Must return a DetectorBuilder object.
        :type detectors: function
        """
        self.infrastructures = infrastructures
        self.flows = flows
        self.detectors = detectors
        self.name = name
        self.config = {'exp_name': name}
        self.files = {}

    def run(self, gui=False, seed=None):
        """
        Launch an SUMO simulation with the network configuration.
        First build the configuration files and then launch SUMO.
        :param gui: True to run SUMO in graphical mode. False otherwise.
        :type gui: bool
        :param seed: The seed of the simulation. Same seeds = same simulations.
        :type seed: int
        """
        self.generate_file_names()
        self.flows(self.config).build(self.files)
        self.infrastructures(self.config).build(self.files)
        if self.detectors is not None:
            self.detectors(self.config).build(self.files)
        os.system(f'$SUMO_HOME/bin/netconvert -n {self.files["nodes"]} -e {self.files["edges"]} -x {self.files["connections"]} -i {self.files["trafic_light_programs"]} -t {self.files["types"]} -o {self.files["network"]}')
        args = self.build_arguments()
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



    def run_traci(self, traci_function, gui=False, seed=None):
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
        """
        self.generate_file_names()
        self.flows(self.config).build(self.files)
        self.infrastructures(self.config).build(self.files)
        self.detectors(self.config).build(self.files)
        os.system(f'$SUMO_HOME/bin/netconvert -n {self.files["nodes"]} -e {self.files["edges"]} -x {self.files["connections"]} -i {self.files["trafic_light_programs"]} -t {self.files["types"]} -o {self.files["network"]}')
        args = self.build_arguments()

        if gui:
            if seed is None:
                traci.start(["sumo-gui"] + args.split() + ['--random'])
            else:
                traci.start(["sumo-gui"] + args.split() + ['--seed', seed])
        else:
            if seed is None:
                traci.start(["sumo"] + args.split() + ['--random'])
            else:
                traci.start(["sumo"] + args.split() + ['--seed', seed])
            
        traci_function(self.config)

        traci.close()

        os.system(f'python3 $SUMO_HOME/tools/xml/xml2csv.py {self.files["queuexml"]} --separator ","')
        os.system(f'python3 $SUMO_HOME/tools/xml/xml2csv.py {self.files["summaryxml"]} --separator ","')



    def clean_files(self, except_summary=False, except_queue=False):
        """
        Delete simulation config files.
        :param except_summary: Doesn't delete summary files if True
        :type except_summary: bool
        :param except_queue: Doesn't delete queue files if
        :type except_queue: bool
        """
        for file in self.files.values():
            if os.path.exists(file):
                if (file == self.files['summaryxml'] or file == self.files['summarycsv']):
                    if not except_summary:
                        os.remove(file)
                elif (file == self.files['queuexml'] or file == self.files['queuecsv']):
                    if not except_queue:
                        os.remove(file)
                else:
                    os.remove(file)

    def build_arguments(self):
        """
        Build the arguments to launch SUMO with a command line.
        """
        args = ''
        args += f'-n {self.files["network"]} '
        args += f'-r {self.files["routes"]} '
        if self.detectors is not None:
            args += f'-a {self.files["detectors"]} '
        args += f'--summary {self.files["summaryxml"]} '
        args += f'--queue-output {self.files["queuexml"]} '
        if 'simulation_duration' in self.config:
            args += f'-e {self.config["simulation_duration"]} '
        return args

    def set_parameter(self, name, value):
        """
        Add a parameter to the experiment configuration.
        :param name: The name of the parameter
        :type name: str
        :param value: The value of the parameter
        """
        self.config[name] = value

    def export_results_to_csv(self, filename, sampling_rate):
        """
        Export experiment results in a CSV file.
        This function appends results of the experiment to the rest of file. It doesn't overwrite.
        Creates the file if it doesn't exist.
        :param filename: CSV file name
        :type filename: str
        :param sampling_rate: Number of simulation steps between two measurements
        :type sampling_rate: int
        """
        summary_data = pd.read_csv(self.files["summarycsv"])
        # queue_data = pd.read_csv(self.files["queuecsv"])

        samples = np.arange(start=sampling_rate, stop=summary_data.shape[0], step=sampling_rate)

        if not exists(filename):
            f = open(filename, 'w')

            # Columns titles
            for key in self.config:
                # To avoid matrices in CSV
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

            for s in samples:
                loaded = summary_data.loc[s]['step_loaded']
                inserted = summary_data.loc[s]['step_inserted']
                running = summary_data.loc[s]['step_running']
                waiting = summary_data.loc[s]['step_waiting']
                ended = summary_data.loc[s]['step_ended']
                halting = summary_data.loc[s]['step_halting']
                stopped = summary_data.loc[s]['step_stopped']
                mean_waiting_time = summary_data.loc[s]['step_meanWaitingTime']
                mean_travel_time = summary_data.loc[s]['step_meanTravelTime']
                mean_speed = summary_data.loc[s]['step_meanSpeed']
                f.write(f'{loaded},{inserted},{running},{waiting},{ended},{halting},{stopped},{mean_waiting_time},{mean_travel_time},{mean_speed}')
                if s != samples[-1]:
                    f.write(',')

            f.write('\n')

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
            'summaryxml': f'summary_{self.name}.xml',
            'summarycsv': f'summary_{self.name}.csv',
            'queuexml': f'queue_{self.name}.xml',
            'queuecsv': f'queue_{self.name}.csv',
            'detectors_out': 'detectors.out'
        }



