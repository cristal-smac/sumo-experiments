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

    def run(self, gui=False, seed=None, no_warnings=True):
        """
        Launch an SUMO simulation with the network configuration.
        First build the configuration files and then launch SUMO.
        :param gui: True to run SUMO in graphical mode. False otherwise.
        :type gui: bool
        :param seed: The seed of the simulation. Same seeds = same simulations.
        :type seed: int
        :param no_warnings: If set to True, no warnings when executing SUMO.
        :type no_warnings: bool
        """
        self.generate_file_names()
        self.flows(self.config).build(self.files)
        self.infrastructures(self.config).build(self.files)
        if self.detectors is not None:
            self.detectors(self.config).build(self.files)

        os.system(f'$SUMO_HOME/bin/netconvert -n {self.files["nodes"]} -e {self.files["edges"]} -x {self.files["connections"]} -i {self.files["trafic_light_programs"]} -t {self.files["types"]} -o {self.files["network"]}')
        args = self.build_arguments(seed, no_warnings)

        if gui:
            os.system(f'$SUMO_HOME/bin/sumo-gui {args}')
        else:
            os.system(f'$SUMO_HOME/bin/sumo {args}')

    def run_traci(self, traci_function, gui=False, seed=None, no_warnings=True):
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
        """
        self.generate_file_names()
        self.flows(self.config).build(self.files)
        self.infrastructures(self.config).build(self.files)
        if self.detectors is not None:
            self.detectors(self.config).build(self.files)
        os.system(f'$SUMO_HOME/bin/netconvert -n {self.files["nodes"]} -e {self.files["edges"]} -x {self.files["connections"]} -i {self.files["trafic_light_programs"]} -t {self.files["types"]} -o {self.files["network"]}')
        args = self.build_arguments(seed, no_warnings)

        if gui:
            traci.start(["sumo-gui"] + args.split())
        else:
            traci.start(["sumo"] + args.split())

        res = traci_function(self.config)

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

    def build_arguments(self, seed, no_warnings):
        """
        Build the arguments to launch SUMO with a command line.
        """
        args = ''
        args += f'-n {self.files["network"]} '
        args += f'-r {self.files["routes"]} '
        if self.detectors is not None:
            args += f'-a {self.files["detectors"]} '
        if 'simulation_duration' in self.config:
            args += f'-e {self.config["simulation_duration"] + 1} '
        if seed is not None:
            args += f'--seed {seed} '
        else:
            args += '--random '
        if no_warnings:
            args += '--no-warnings'
        return args

    def set_parameter(self, name, value):
        """
        Add a parameter to the experiment configuration.
        :param name: The name of the parameter
        :type name: str
        :param value: The value of the parameter
        """
        self.config[name] = value

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
            'detectors_out': 'detectors.out'
        }



