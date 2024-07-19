import traci
from sumo_experiments.strategies.bologna import BolognaStrategy
import operator


class AcolightStrategyBologna(BolognaStrategy):
    """
    Implement an Acolight agent for all intersections of the Bologna network.
    """

    def __init__(self, min_phase_durations, max_phase_durations):
        """
        Init of class
        """
        super().__init__()
        self.started = False
        self.time = {
            '209': 0,
            '210': 0,
            '219': 0,
            '220': 0,
            '221': 0,
            '235': 0,
            '273': 0,
        }
        self.next_phase = {
            '209': 0,
            '210': 0,
            '219': 0,
            '220': 0,
            '221': 0,
            '235': 0,
            '273': 0,
        }
        self.min_phase_durations = min_phase_durations
        self.max_phase_durations = max_phase_durations

    def run_all_agents(self):
        """
        Process agents to make one action each.
        :return: Nothing
        """
        #print(self.time)
        if not self.started:
            self._start_agents()
            return True
        else:
            for id_tls in self.TL_IDS:
                current_phase = traci.trafficlight.getPhase(id_tls)
                if current_phase in self.TLS_DETECTORS[id_tls]:
                    if self.time[id_tls] == 0:
                        traci.trafficlight.setPhase(id_tls, self.next_phase[id_tls])
                        self.time[id_tls] += 1
                    elif self.time[id_tls] >= self.min_phase_durations[id_tls]:
                        if self.time[id_tls] >= self.max_phase_durations[id_tls]:
                            next_phase = self.get_next_phase(id_tls)
                            if next_phase != current_phase:
                                self.next_phase[id_tls] = next_phase
                                traci.trafficlight.setPhase(id_tls, current_phase + 1)
                                self.time[id_tls] = 0
                            else:
                                self.time[id_tls] = self.min_phase_durations[id_tls] + 1
                        elif not self.are_vehicles_passing(id_tls):
                            next_phase = self.get_next_phase(id_tls)
                            if next_phase != current_phase:
                                self.next_phase[id_tls] = next_phase
                                traci.trafficlight.setPhase(id_tls, current_phase + 1)
                                self.time[id_tls] = 0
                            else:
                                self.time[id_tls] = self.min_phase_durations[id_tls] + 1
                        elif self.blocked_vehicles(id_tls):
                            next_phase = self.get_next_phase(id_tls)
                            if next_phase != current_phase:
                                self.next_phase[id_tls] = next_phase
                                traci.trafficlight.setPhase(id_tls, current_phase + 1)
                                self.time[id_tls] = 0
                            else:
                                self.time[id_tls] = self.min_phase_durations[id_tls] + 1
                        elif self.red_lane_saturated(id_tls):
                            next_phase = self.get_next_phase(id_tls)
                            if next_phase != current_phase:
                                self.next_phase[id_tls] = next_phase
                                traci.trafficlight.setPhase(id_tls, current_phase + 1)
                                self.time[id_tls] = 0
                            else:
                                self.time[id_tls] = self.min_phase_durations[id_tls] + 1
                        else:
                            self.time[id_tls] += 1
                    else:
                        self.time[id_tls] += 1

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
            if traci.lanearea.getJamLengthVehicle(det) == 0:
                return False
        return True

    def red_lane_saturated(self, id_tls):
        """
        Check if one red lanes is saturated, with the boolean detectors. If more than one lane is saturated, doesn't occur
        :param id_tls: The id of the traffic light
        :type id_tls: str
        :return: True if there are saturated red lanes, false otherwise
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
        Get the next phase with at least one vehicle waiting.
        :param id_tls: The id of the traffic light
        :type id_tls: str
        :return: True if there are blocked vehicles on green lanes, false otherwise
        :rtype: int
        """
        current_phase = traci.trafficlight.getPhase(id_tls)
        nb_phases = len(traci.trafficlight.getAllProgramLogics(id_tls)[1].phases)
        for i in range(current_phase + 1, current_phase + nb_phases):
            real_phase = i % nb_phases
            if real_phase in self.TLS_DETECTORS[id_tls]:
                for det in self.TLS_DETECTORS[id_tls][real_phase]['boolean']:
                    if traci.lanearea.getLastStepVehicleNumber(det) > 0:
                        return real_phase
        return current_phase


    def _start_agents(self):
        """
        Start an agent at the beginning of the simulation.
        """
        for tl in self.TL_IDS:
            tl_logic = traci.trafficlight.getAllProgramLogics(tl)[1]
            nb_phase = 0
            for phase in tl_logic.phases:
                if nb_phase in self.TLS_DETECTORS[tl]:
                    phase.duration = 10000
                    phase.maxDur = 10000
                    phase.minDur = 10000
                nb_phase += 1
            traci.trafficlight.setProgramLogic(tl, tl_logic)
            traci.trafficlight.setPhase(tl, 0)
            traci.trafficlight.setPhaseDuration(tl, 10000)
        self.started = True