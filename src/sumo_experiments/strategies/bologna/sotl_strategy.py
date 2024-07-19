import traci
from sumo_experiments.strategies.bologna import BolognaStrategy
import operator


class SotlStrategyBologna(BolognaStrategy):
    """
    Implement a SOTL agent for all intersections of the Bologna network.
    """

    def __init__(self, thresholds_switch, thresholds_force, min_phase_durations):
        """
        Init of class
        """
        super().__init__()
        self.started = False
        self.countdowns = {
            '209': 0,
            '210': 0,
            '219': 0,
            '220': 0,
            '221': 0,
            '235': 0,
            '273': 0,
        }
        self.time = {
            '209': 0,
            '210': 0,
            '219': 0,
            '220': 0,
            '221': 0,
            '235': 0,
            '273': 0,
        }
        self.thresholds_switch = thresholds_switch
        self.thresholds_force = thresholds_force
        self.min_phase_durations = min_phase_durations

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
                sum_vehicles = self.compute_vehicles_red_lanes(id_tls)
                if current_phase in self.TLS_DETECTORS[id_tls]:
                    if self.time[id_tls] >= self.min_phase_durations[id_tls]:
                        if self.countdowns[id_tls] >= self.thresholds_switch[id_tls]:
                            if not self.are_vehicles_passing(id_tls) or self.time[id_tls] >= self.thresholds_force[id_tls]:
                                traci.trafficlight.setPhase(id_tls, current_phase + 1)
                                self.countdowns[id_tls] = 0
                                self.time[id_tls] = 0
                            else:
                                self.countdowns[id_tls] += sum_vehicles
                                self.time[id_tls] += 1
                        else:
                            self.countdowns[id_tls] += sum_vehicles
                            self.time[id_tls] += 1
                    else:
                        self.countdowns[id_tls] += sum_vehicles
                        self.time[id_tls] += 1

    def compute_vehicles_red_lanes(self, id_tls):
        """
        Compute the number of vehicles on the red lanes, with the numerical detectors.
        :param id_tls: The id of the traffic light
        :type id_tls: str
        :return: The number of vehicles approaching the traffic light on the red lanes
        :rtype: int
        """
        current_phase = traci.trafficlight.getPhase(id_tls)
        detectors = set()
        for phase in self.TLS_DETECTORS[id_tls]:
            if phase != current_phase:
                detectors.update(self.TLS_DETECTORS[id_tls][phase]['numerical'])
        sum = 0
        for detector in detectors:
            sum += traci.lanearea.getLastStepVehicleNumber(detector)
        return sum

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