from sumo_experiments.agents import Agent
import traci


class FixedTimeAgent(Agent):
    """
    Implements a fixed time agent. The agent has fixed time for each phase and switch to the next phase when this
    time is over.
    """

    def __init__(self, id_intersection, id_tls_program, phases_durations):
        """
        Init of class.
        :param id_intersection: The id of the intersection the agent will control
        :type id_intersection: str
        :param id_tls_program: The id of the traffic light program related to the intersection
        :type id_tls_program: str
        :param phases_durations: The durations of each traffic light phase (length = number of phases), including yellows
        :type phases_durations: iterable object
        """
        super().__init__()
        self.started = False
        self.id_intersection = id_intersection
        self.phases_durations = phases_durations
        self.current_phase = 0
        self.countdown = 0
        self.nb_phases = len(phases_durations)
        self.id_tls_program = id_tls_program

    def choose_action(self):
        """
        Switch to the next phase when countdown is equal to the current phase duration.
        :return: True if the agent switched to another phase, False otherwise
        :rtype: bool
        """
        if not self.started:
            self._start_agent()
            return True

    def _start_agent(self):
        """
        Start the agent at the beginning of the simulation.
        """
        self.nb_phases = len(traci.trafficlight.getAllProgramLogics(self.id_tls_program)[0].phases)
        self.phases_index = {}
        tl_logic = traci.trafficlight.getAllProgramLogics(self.id_tls_program)[0]
        phase_index = 0
        nb_phase = 0
        for phase in tl_logic.phases:
            if 'y' in phase.state:
                phase.duration = 3
                phase.minDur = 3
                phase.maxDur = 3
            else:
                self.phases_index[nb_phase] = phase_index
                phase.duration = self.phases_durations[phase_index]
                phase.maxDur = self.phases_durations[phase_index]
                phase.minDur = self.phases_durations[phase_index]
                phase_index += 1
            nb_phase += 1
        traci.trafficlight.setProgramLogic(self.id_tls_program, tl_logic)
        traci.trafficlight.setPhase(self.id_tls_program, 0)
        self.started = True

