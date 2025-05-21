from pygments.styles.dracula import yellow

from sumo_experiments.agents import Agent
import traci


class FixedTimeAgent(Agent):
    """
    Implements a fixed time agent. The agent has fixed time for each phase and switch to the next phase when this
    time is over.
    """

    def __init__(self, id_intersection, id_tls_program, phases_durations, yellow_time=3):
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
        self.yellow_time = yellow_time
        self.current_yellow_time = 0

    def choose_action(self):
        """
        Switch to the next phase when countdown is equal to the current phase duration.
        :return: True if the agent switched to another phase, False otherwise
        :rtype: bool
        """
        if not self.started:
            self._start_agent()
            return True
        else:
            current_phase = traci.trafficlight.getPhase(self.id_intersection)
            current_state = traci.trafficlight.getRedYellowGreenState(self.id_intersection)
            if 'y' not in current_state:
                if self.countdown >= self.phases_durations[self.phases_index[current_phase]]:
                    traci.trafficlight.setPhase(self.id_intersection, current_phase + 1)
                    self.countdown = 0
                else:
                    self.countdown += 1
            else:
                if self.current_yellow_time >= self.yellow_time:
                    if current_phase + 1 != self.nb_phases:
                        traci.trafficlight.setPhase(self.id_intersection, current_phase + 1)
                    else:
                        traci.trafficlight.setPhase(self.id_intersection, 0)
                    self.current_yellow_time = 0
                else:
                    self.current_yellow_time += 1

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
                phase.duration = 10000
                phase.minDur = 10000
                phase.maxDur = 10000
            else:
                self.phases_index[nb_phase] = phase_index
                phase.duration = 10000 # self.phases_durations[phase_index]
                phase.maxDur = 10000 # self.phases_durations[phase_index]
                phase.minDur = 10000 # self.phases_durations[phase_index]
                phase_index += 1
            nb_phase += 1
        traci.trafficlight.setProgramLogic(self.id_tls_program, tl_logic)
        traci.trafficlight.setPhase(self.id_tls_program, 0)
        self.started = True

