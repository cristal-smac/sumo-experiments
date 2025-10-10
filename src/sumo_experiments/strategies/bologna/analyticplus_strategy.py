from tensorflow.python.distribute.device_util import current

from sumo_experiments.strategies.bologna import BolognaStrategy
from sumo_experiments.agents import AnalyticPlusAgent
from queue import SimpleQueue
import operator

# import libsumo as traci
import traci.constants as tc

DEBUG = False

class AnalyticStrategy(BolognaStrategy):
    """
    Implement an Analytic+ agent for all intersections of the Bologna network.
    """

    def __init__(self, min_phase_durations, max_phase_durations):
        """
        Init of class
        """
        super().__init__()
        self.started = False
        self.min_phase_durations = min_phase_durations
        self.max_phase_durations = max_phase_durations
        self.yellow_time = 3
        self.step_length = 1

        # self.TL_IDS = ['221'] # For testing only one traffic light
        self.time = {k: 0 for k in self.TL_IDS}
        self.next_phase = {k: 0 for k in self.TL_IDS}
        self.priority_pile = {k: [] for k in self.TL_IDS}
        self.prio = {k: False for k in self.TL_IDS}
        self.current_cycle = {k: [] for k in self.TL_IDS}
        self.current_yellow_time = {k: 0 for k in self.TL_IDS}
        self.is_phase = {k: True for k in self.TL_IDS}
        self.nb_switch = {k: 0 for k in self.TL_IDS}
        self.nb_phases = {k: 0 for k in self.TL_IDS}
        self.phases_occurences = {k: {} for k in self.TL_IDS}
        self.agents = {k: AnalyticPlusAgent(self, id_tls_program=k,
                                            yellow_time=self.yellow_time) for k in self.TL_IDS}

    def switch_yellow(self, agent):
        """
        Switch the traffic light id_tls to yellow
        """
        num_phases = len(agent.sumo_phases)
        self.traci.trafficlight.setPhase(agent.ID, (self.traci.trafficlight.getPhase(agent.ID) + 1) % num_phases)

    def run_all_agents(self, traci):
        """
        Process agents to make one action each.
        :return: Nothing
        """
        if not self.started:
            self.traci = traci
            self._start_agents()
            # god mode data subscription
            varIDs_lane = [tc.LAST_STEP_VEHICLE_NUMBER, tc.LAST_STEP_VEHICLE_ID_LIST]
            for lane_id in self.traci.lane.getIDList():
                if lane_id[0]==':': # skip internal lanes
                    continue
                self.traci.lane.subscribe(lane_id, varIDs=varIDs_lane)
            return True
        else:
            lane_data = self.traci.lane.getAllSubscriptionResults()
            for id_tls in self.TL_IDS:
                agent = self.agents[id_tls]
                agent.update(lane_data)
                current_phase = self.traci.trafficlight.getPhase(id_tls)
                current_state = self.traci.trafficlight.getRedYellowGreenState(id_tls)
                # if current_phase not in self.TLS_DETECTORS[id_tls]: # Handle the yellow phases
                if 'y' in current_state:
                    if self.yellow_time - self.current_yellow_time[id_tls] <= 0:
                        # maybe there is a second yellow phase
                        self.switch_yellow(agent)
                        assert self.traci.trafficlight.getPhase(id_tls) != current_phase
                        if not agent.sumo_phases[self.traci.trafficlight.getPhase(id_tls)]['isYellow']: # not a yellow phase, switch to next green
                            self.traci.trafficlight.setPhase(id_tls, self.next_phase[id_tls])
                            agent.current_sumo_phase = agent.sumo_phases[self.next_phase[id_tls]]
                            self.current_cycle[id_tls].append(self.next_phase[id_tls])
                        self.current_yellow_time[id_tls] = 0
                    else:
                        self.current_yellow_time[id_tls] += 1
                else:
                    assert current_phase == self.next_phase[id_tls]
                    if current_phase not in self.phases_occurences[id_tls]:
                        self.phases_occurences[id_tls][current_phase] = 1
                    else:
                        self.phases_occurences[id_tls][current_phase] += 1

                    if self.time[id_tls] >= self.min_phase_durations[id_tls]:
                        # start logic here
                        self.analyticplus_logic(id_tls)

                    #if not self.prio[id_tls]:
                    # if len(self.priority_pile[id_tls]) == 0 and self.prio[id_tls]:
                    #     self.add_prio_phases(id_tls)
                    # if self.agents[id_tls].current_sumo_phase['id'] == current_phase:
                    self.time[id_tls] += 1
                

    def analyticplus_logic(self, id_tls):
        agent = self.agents[id_tls]
        if self.time[id_tls] >= agent.green_duration:
            self.switch_next_phase(id_tls)


    def switch_next_phase(self, id_tls):
        """
        Switch the traffic light id_tls to the next
        """
        agent = self.agents[id_tls]
        self.nb_switch[id_tls] += 1
        current_phase = self.traci.trafficlight.getPhase(id_tls)
        time = self.traci.simulation.getTime()

        action_id, green_time = agent.choose_action(time)
        next_phase = agent.action_phases[action_id]
        if id_tls == '221' and DEBUG:
            print(f"TLS {id_tls} switches from phase {current_phase} to {next_phase} for {green_time}s at time {time}s")
            print(f"action queue: {agent.action_queue}")
        agent.green_duration = green_time

        if next_phase != current_phase:
            self.next_phase[id_tls] = next_phase
            agent.update_last_on(agent.sumo_phases[next_phase], agent.sumo_phases[current_phase], time)
            assert not agent.sumo_phases[next_phase]['isYellow']
            # switch to hopefully the next yellow phase
            self.switch_yellow(agent)
            assert agent.sumo_phases[self.traci.trafficlight.getPhase(id_tls)]['isYellow'] 
            self.time[id_tls] = 0


    def _start_agents(self):
        """
        Start an agent at the beginning of the simulation.
        """
        for tl in self.TL_IDS:
            tl_logic = self.traci.trafficlight.getAllProgramLogics(tl)[-1]
            nb_phase = 0
            for phase in tl_logic.phases:
                phase.duration = 10000
                phase.maxDur = 10000
                phase.minDur = 10000
                nb_phase += 1
            self.nb_phases[tl] = nb_phase
            self.traci.trafficlight.setProgramLogic(tl, tl_logic)
            self.traci.trafficlight.setPhase(tl, 0)
            self.traci.trafficlight.setPhaseDuration(tl, 10000)
            self.agents[tl].reset(self.traci)
        self.started = True