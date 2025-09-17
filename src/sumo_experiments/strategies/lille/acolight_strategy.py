import traci
from tensorflow.python.distribute.device_util import current

from sumo_experiments.strategies.bologna import BolognaStrategy
from queue import SimpleQueue
import operator

from sumo_experiments.strategies.lille.lille_strategy import LilleStrategy


class AcolightStrategyLille(LilleStrategy):
    """
    Implement an Acolight agent for all intersections of the Bologna network.
    """

    def __init__(self, min_phase_durations, max_phase_durations):
        """
        Init of class
        """
        super().__init__()
        self.started = False
        self.time = {identifiant: 0 for identifiant in self.TLS_DETECTORS}
        self.next_phase = {identifiant: 0 for identifiant in self.TLS_DETECTORS}
        self.priority_pile = {identifiant: [] for identifiant in self.TLS_DETECTORS}
        self.prio = {identifiant: False for identifiant in self.TLS_DETECTORS}
        self.current_cycle = {identifiant: [] for identifiant in self.TLS_DETECTORS}
        self.current_yellow_time = {identifiant: 0 for identifiant in self.TLS_DETECTORS}
        self.is_phase = {identifiant: True for identifiant in self.TLS_DETECTORS}
        self.min_phase_durations = min_phase_durations
        self.max_phase_durations = max_phase_durations
        self.yellow_time = 3

        self.nb_switch = {identifiant: 0 for identifiant in self.TLS_DETECTORS}
        self.phases_occurences = {identifiant: {} for identifiant in self.TLS_DETECTORS}

    def run_all_agents(self):
        """
        Process agents to make one action each.
        :return: Nothing
        """
        if not self.started:
            self._start_agents()
            return True
        else:
            for id_tls in self.TL_IDS:
                current_phase = traci.trafficlight.getPhase(id_tls)
                current_state = traci.trafficlight.getRedYellowGreenState(id_tls)
                # If yellow phase
                if current_phase not in self.TLS_DETECTORS[id_tls]:
                    if 'y' in current_state:
                        if self.yellow_time - self.current_yellow_time[id_tls] <= 0:
                            #if current_phase == self.next_phase[id_tls] - 1 or self.next_phase[id_tls] == 0:
                            traci.trafficlight.setPhase(id_tls, self.next_phase[id_tls])
                            self.current_cycle[id_tls].append(self.next_phase[id_tls])
                            self.current_yellow_time[id_tls] = 1
                        else:
                            self.current_yellow_time[id_tls] += 1
                else:
                    # Counting phase occurences
                    if current_phase not in self.phases_occurences[id_tls]:
                        self.phases_occurences[id_tls][current_phase] = 1
                    else:
                        self.phases_occurences[id_tls][current_phase] += 1
                    # Real behaviour
                    if self.time[id_tls] >= self.min_phase_durations[id_tls]:
                        if self.time[id_tls] >= self.max_phase_durations[id_tls]:
                            self.switch_next_phase(id_tls)
                        elif not self.are_vehicles_passing(id_tls):
                            self.switch_next_phase(id_tls)
                        elif self.blocked_vehicles(id_tls) and self.time[id_tls] > 3:
                            self.switch_next_phase(id_tls)
                    #if not self.prio[id_tls]:
                    if len(self.priority_pile[id_tls]) == 0 and self.prio[id_tls]:
                        self.add_prio_phases(id_tls)
                    self.time[id_tls] += 1

    def switch_next_phase(self, id_tls):
        """
        Switch the traffic light id_tls to the next
        """
        self.nb_switch[id_tls] += 1
        current_phase = traci.trafficlight.getPhase(id_tls)
        next_phase = self.get_next_phase(id_tls)
        if next_phase != current_phase:
            self.next_phase[id_tls] = next_phase
            if traci.trafficlight.getPhase(id_tls) == traci.trafficlight.getPhase(id_tls) - 1:
                traci.trafficlight.setPhase(id_tls, 0)
            else:
                traci.trafficlight.setPhase(id_tls, traci.trafficlight.getPhase(id_tls) + 1)
            self.time[id_tls] = 0
        else:
            #self.time[id_tls] = self.min_phase_durations[id_tls] + 1
            phases_available = list(self.TLS_DETECTORS[id_tls].keys())
            index = phases_available.index(next_phase)
            if index == len(phases_available) - 1:
                next_phase = 0
            else:
                next_phase = phases_available[index + 1]
            self.next_phase[id_tls] = next_phase
            if traci.trafficlight.getPhase(id_tls) == traci.trafficlight.getPhase(id_tls) - 1:
                traci.trafficlight.setPhase(id_tls, 0)
            else:
                traci.trafficlight.setPhase(id_tls, traci.trafficlight.getPhase(id_tls) + 1)
            self.time[id_tls] = 0


    def add_prio_phases(self, id_tls):
        """
        Add the current priority phases (the phases with saturated lanes) to the pile.
        """
        current_phase = traci.trafficlight.getPhase(id_tls)
        for phase in self.TLS_DETECTORS[id_tls]:
            detectors = self.TLS_DETECTORS[id_tls][phase]['saturation']
            #if any([any([traci.vehicle.isStopped(veh) == True for veh in traci.lanearea.getLastStepVehicleIDs(det)]) for det in detectors]) and phase != current_phase:
            if any([0 < traci.lanearea.getLastStepMeanSpeed(det) < 0.5 and traci.lanearea.getLastStepVehicleNumber(det) > 0 for det in detectors]) and phase != current_phase:
                if phase not in self.priority_pile[id_tls]:
                    self.priority_pile[id_tls].append(phase)

    def is_cycle_complete(self, id_tls):
        """
        Return True if the cycle of a controller is complete. A complete cycle is a cycle where all the phases
        of the controller appear at least one time, and max 1 time for the least represented one.
        """
        counts = []
        phases = self.TLS_DETECTORS[id_tls]
        for phase in phases:
            counts.append(self.current_cycle[id_tls].count(phase))
        if len(self.current_cycle[id_tls]) > 0:
            #if (counts[list(phases).index(self.current_cycle[id_tls][-1])] > 1) or (0 in counts):
            if 0 in counts:
                return False
        else:
            return False
        return True


    def blocked_vehicles(self, id_tls):
        """
        Check if vehicles are blocked on all the green lanes, with the boolean detectors.
        :param id_tls: The id of the traffic light
        :type id_tls: str
        :return: True if there are blocked vehicles on green lanes, false otherwise
        :rtype: int
        """
        current_phase = traci.trafficlight.getPhase(id_tls)
        for det in self.TLS_DETECTORS[id_tls][current_phase]['boolean']:
            if traci.lanearea.getLastStepMeanSpeed(det) > 0.5:
                return False
        return True

    def red_lane_saturated(self, id_tls):
        """
        Check if one red lanes is saturated, with the boolean detectors. If more than one lane is saturated, doesn't occur
        :param id_tls: The id of the traffic light
        :type id_tls: str
        :return: The next phase saturated if there are saturated red lanes, None otherwise
        :rtype: bool
        """
        current_phase = traci.trafficlight.getPhase(id_tls)
        detectors = set()
        for phase in self.TLS_DETECTORS[id_tls]:
            if phase != current_phase:
                detectors.update(self.TLS_DETECTORS[id_tls][phase]['saturation'])
        for det in detectors:
            if traci.lanearea.getJamLengthVehicle(det) > 0:
                return True
        return False


    def are_vehicles_passing(self, id_tls):
        """
        Check if vehicles are still passing the intersection, with the boolean detectors.
        :param id_tls: The id of the traffic light
        :type id_tls: str
        :return: True if vehicles are still passing the intersection, False otherwise
        :rtype: bool
        """
        current_phase = traci.trafficlight.getPhase(id_tls)
        for det in self.TLS_DETECTORS[id_tls][current_phase]['boolean']:
            if traci.lanearea.getLastStepVehicleNumber(det) > 0:
                return True
        return False


    def get_next_phase(self, id_tls):
        """
        Get the next phase for the controller.
        :param id_tls: The id of the traffic light
        :type id_tls: str
        :return: The next phase for the controller
        :rtype: int
        """
        if self.prio[id_tls] and len(self.priority_pile[id_tls]) > 0:
            if len(self.priority_pile[id_tls]) == 1:
                self.prio[id_tls] = False
            return self.priority_pile[id_tls].pop(0)
        else:
            if self.is_cycle_complete(id_tls):
                self.current_cycle[id_tls] = []
                self.prio[id_tls] = True
                return 0
            else:
                for phase in self.TLS_DETECTORS[id_tls]:
                    if phase not in self.current_cycle[id_tls]:
                        for det in self.TLS_DETECTORS[id_tls][phase]['boolean']:
                            if traci.lanearea.getLastStepVehicleNumber(det) > 0:
                                self.prio[id_tls] = True
                                return phase
                for phase in self.TLS_DETECTORS[id_tls]:
                    for det in self.TLS_DETECTORS[id_tls][phase]['boolean']:
                        if traci.lanearea.getLastStepVehicleNumber(det) > 0:
                            self.prio[id_tls] = True
                            return phase
        self.prio[id_tls] = True
        return traci.trafficlight.getPhase(id_tls) # Current phase


    def _start_agents(self):
        """
        Start an agent at the beginning of the simulation.
        """
        for tl in self.TL_IDS:
            tl_logic = traci.trafficlight.getAllProgramLogics(tl)[0]
            nb_phase = 0
            for phase in tl_logic.phases:
                phase.duration = 10000
                phase.maxDur = 10000
                phase.minDur = 10000
                nb_phase += 1
            self.nb_phases = nb_phase
            traci.trafficlight.setProgramLogic(tl, tl_logic)
            traci.trafficlight.setPhase(tl, 0)
            traci.trafficlight.setPhaseDuration(tl, 10000)
        self.started = True