import numpy as np

from sumo_experiments.agents import Agent
import traci


class LQFAgent(Agent):
    """
    Implements a longest queue first agent. An intersection managed by this agent must be equipped with numerical detectors
    on the entry lanes, that count how many vehicles are currently present on the watched lane.
    At each period t, the agent selects the phase with the longest waiting queue for the next period.
    """

    def __init__(self,
                 id_intersection,
                 id_tls_program,
                 intersection_relations,
                 period,
                 counted_vehicles='all',
                 yellow_time=None):
        """
        Init of class.
        :param id_intersection: The id of the intersection the agent will control
        :type id_intersection: str
        :param id_tls_program: The id of the traffic light program related to the intersection
        :type id_tls_program: str
        :param period: The duration of a period (in seconds).
        :type period: int
        :param counted_vehicles: The vehicles that will be count. 'all' means that all vehicles detected by the numerical
        detector will be counted. 'stopped' means that only stopped vehicles detected will be counted.
        :type counted_vehicles: str
        :param yellow_time: The duration of yellow phases. If None, yellow phase will remain as declared in the network definition.
        :type yellow_time: dict
        :param intersection_relations: The relations for this intersection.
        :type intersection_relations: dict
        """
        super().__init__()
        self.started = False
        self.id_intersection = id_intersection
        self.id_tls_program = id_tls_program
        self.period = period
        self.yellow_time = yellow_time
        self.current_phase = 0
        self.countdown = 0
        self.relations = intersection_relations
        self.current_max_time_index = 0
        if counted_vehicles == 'all':
            self.count_function = traci.lanearea.getLastStepVehicleNumber
        elif counted_vehicles == 'stopped':
            self.count_function = traci.lanearea.getJamLengthVehicle
        else:
            raise ValueError('counted_vehicles argument is not valid.')

    def choose_action(self):
        """
        If the threshold is passed for a defined lane and the minimum time is exceeded, switch to a phase that is green
        for this lane.
        :return: True if the agent switched to another phase, False otherwise
        :rtype: bool
        """
        if not self.started:
            self._start_agent()
            return True
        if 'y' not in traci.trafficlight.getRedYellowGreenState(self.id_tls_program):
            if self.countdown == 0:
                traci.trafficlight.setPhase(self.id_tls_program, self.current_phase)
            if self.countdown >= self.period:
                next_phase = self.current_phase
                longest_queue = 0
                for i in range(self.nb_phases):
                    traci.trafficlight.setPhase(self.id_tls_program, i)
                    if 'y' not in traci.trafficlight.getRedYellowGreenState(self.id_tls_program):
                        green_detectors = self._detectors_green_lanes()
                        phase_queue = sum([self.count_function(det.id) for det in green_detectors])
                        if phase_queue > longest_queue:
                            next_phase = i
                            longest_queue = phase_queue
                traci.trafficlight.setPhase(self.id_tls_program, self.current_phase)
                if not next_phase == self.current_phase:
                    traci.trafficlight.setPhase(self.id_tls_program, self.current_phase + 1)
                    self.current_phase = next_phase
                self.countdown = 0
                return True
            self.countdown += 1
        return False

    def _detectors_green_lanes(self):
        """
        Return the detectors related to green lanes entry for a phase.
        :return: The list of all concerned detectors
        :rtype: list
        """
        detectors = []
        current_phase = traci.trafficlight.getRedYellowGreenState(self.id_tls_program)
        phases = traci.trafficlight.getControlledLinks(self.id_tls_program)
        for i in range(len(current_phase)):
            link = current_phase[i]
            if link == 'g' or link == 'G':
                link_infos = phases[i]
                for info in link_infos:
                    lane = info[0]
                    lane_number = int(lane.split('_')[-1])
                    edge = lane[:-2]
                    edge_index = self.relations['related_edges'].index(edge)
                    detector = self.relations['related_numerical_detectors'][edge_index][lane_number]
                    if detector not in detectors:
                        detectors.append(detector)
        return detectors

    def _compute_pressure(self, entry_detectors, exit_detectors):
        """
        Compute the pressure for a phase. The pressure is computed in vehicles.
        :param entry_detectors: The entry detectors of the phase.
        :type entry_detectors: list
        :param exit_detectors: The exit detectors of the phase.
        :type exit_detectors: list
        :return: The pressure of the phase.
        :rtype: int
        """
        pressure = 0
        for detector in entry_detectors:
            pressure += self.count_function(detector.id)
        for detector in exit_detectors:
            pressure -= self.count_function(detector.id)
        return pressure

    def _start_agent(self):
        """
        Start an agent at the beginning of the simulation.
        """
        self.nb_phases = len(traci.trafficlight.getAllProgramLogics(self.id_tls_program)[0].phases)
        self.phases_index = {}
        tl_logic = traci.trafficlight.getAllProgramLogics(self.id_tls_program)[0]
        phase_index = 0
        nb_phase = 0
        for phase in tl_logic.phases:
            if 'y' in phase.state:
                if self.yellow_time is not None:
                    phase.duration = self.yellow_time
                    phase.minDur = self.yellow_time
                    phase.maxDur = self.yellow_time
            else:
                phase.duration = 10000
                phase.maxDur = 10000
                phase.minDur = 10000
                self.phases_index[nb_phase] = phase_index
                phase_index += 1
            nb_phase += 1
        traci.trafficlight.setProgramLogic(self.id_tls_program, tl_logic)
        traci.trafficlight.setPhase(self.id_tls_program, 0)
        traci.trafficlight.setPhaseDuration(self.id_tls_program, 10000)
        self.started = True