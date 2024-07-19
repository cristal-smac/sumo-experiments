import traci
from sumo_experiments.strategies.bologna import BolognaStrategy
import operator


class MaxPressureStrategyBologna(BolognaStrategy):
    """
    Implement a max pressure agent for all intersections of the Bologna network.
    """

    def __init__(self, period_times):
        """
        Init of class
        """
        super().__init__()
        self.started = False
        self.period_times = period_times
        self.countdowns = {
            '209': 0,
            '210': 0,
            '219': 0,
            '220': 0,
            '221': 0,
            '235': 0,
            '273': 0,
        }
        self.to_switch = {
            '209': None,
            '210': None,
            '219': None,
            '220': None,
            '221': None,
            '235': None,
            '273': None,
        }

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
                if current_phase in self.TLS_DETECTORS[id_tls] and self.to_switch[id_tls] is not None:
                    traci.trafficlight.setPhase(id_tls, self.to_switch[id_tls])
                    self.to_switch[id_tls] = None
                    self.countdowns[id_tls] = 0
                if self.countdowns[id_tls] >= self.period_times[id_tls]:
                    if current_phase in self.TLS_DETECTORS[id_tls]:
                        pressures = self._compute_pressure(self.TLS_DETECTORS[id_tls])
                        phase_max_pressure = max(pressures.items(), key=operator.itemgetter(1))[0]
                        if phase_max_pressure != current_phase:
                            self.to_switch[id_tls] = phase_max_pressure
                            traci.trafficlight.setPhase(id_tls, current_phase + 1)
                    else:
                        self.countdowns[id_tls] += 1
                else:
                    self.countdowns[id_tls] += 1
    def _compute_pressure(self, detectors):
        """
        Compute the pressure for a phase. The pressure is computed in vehicles.
        :param detectors: The list of detectors of the intersection.
        :type detectors: dict
        :return: The pressures of all phases.
        :rtype: dict
        """
        pressures = {}
        for phase in detectors:
            pressure = 0
            routes = {}
            nb_vehicles = 0
            for detector in detectors[phase]['numerical']:
                pressure += traci.lanearea.getLastStepOccupancy(detector) * traci.lanearea.getLastStepVehicleNumber(detector)
                vehicles = traci.lanearea.getLastStepVehicleIDs(detector)
                for vehicle in vehicles:
                    direction = traci.vehicle.getRoute(vehicle)[0]
                    if direction in routes:
                        routes[direction] += 1
                    else:
                        routes[direction] = 1
                    nb_vehicles += 1
            for key in routes:
                routes[key] = routes[key] / nb_vehicles
            for detector in detectors[phase]['exit']:
                edge = traci.lanearea.getLaneID(detector).split('_')[0]
                k = routes[edge] if edge in routes else 0
                pressure -= traci.lanearea.getLastStepOccupancy(detector) * traci.lanearea.getLastStepVehicleNumber(detector) * k
            pressures[phase] = pressure
        return pressures

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