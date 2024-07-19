from sumo_experiments.strategies.bologna import BolognaStrategy
import traci


class FixedTimeStrategyBologna(BolognaStrategy):
    """
    Implement a fixed time agent for all intersections of the Bologna network.
    """

    def __init__(self, phase_times=None):
        """
        Init of class
        :param phase_times: The time for each phase of each intersection
        :type phase_times: dict
        """
        super().__init__()
        self.phase_times = phase_times
        self.started = False

    def _start_agents(self):
        """
        Start an agent at the beginning of the simulation.
        """
        for tl in self.TL_IDS:
            assert tl in self.phase_times, "The list of intersections given in the phase_times parameter is not exhaustive."
            tl_logic = traci.trafficlight.getAllProgramLogics(tl)[0]
            nb_phase = 0
            i = 0
            for phase in tl_logic.phases:
                if nb_phase in self.TLS_DETECTORS[tl]:
                    phase.duration = self.phase_times[tl][i]
                    phase.maxDur = self.phase_times[tl][i]
                    phase.minDur = self.phase_times[tl][i]
                    i += 1
                nb_phase += 1
            traci.trafficlight.setProgramLogic(tl, tl_logic)
            traci.trafficlight.setPhase(tl, 0)
        self.started = True

    def run_all_agents(self):
        """
        Process agents to make one action each.
        :return: Nothing
        """
        if not self.started:
            if self.phase_times is not None:
                self._start_agents()
            self.started = True

