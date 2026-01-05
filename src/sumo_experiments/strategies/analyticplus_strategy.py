from __future__ import annotations

import numpy as np
import re
from collections import deque
from typing import TypedDict, Optional, Union, TypeAlias
from math import ceil


from sumo_experiments.strategies import Strategy

import traci.constants as tc

DEBUG = False


class AnalyticPlusStrategy(Strategy):
    """
    Implement an Analytic+ agent for all intersections of the Bologna network.

    Lämmer, S., & Helbing, D. (2008). Self-control of traffic lights and vehicle flows in urban road networks. Journal of Statistical Mechanics: Theory and Experiment, 2008(04), P04019.
    """

    def __init__(self, network, min_phase_duration=3, T=150, T_max=180, yellow_time=3):
        """
        Init of class
        :param network: The network to deploy the strategy
        :type network: src.sumo_experiments.Network
        :param min_phase_duration: The minimum phase durations
        :type min_phase_duration: int
        :param T: Target service interval
        :type T: int
        :param T_max: Maximum allowable service interval for stabilization
        :type T_max: int
        :param yellow_time: Yellow phases duration for all intersections
        :type yellow_time: int
        """
        super().__init__()
        self.started = False
        self.network = network
        if type(min_phase_duration) is dict:
            self.min_phase_durations = min_phase_duration
        else:
            self.min_phase_durations = {identifiant: min_phase_duration for identifiant in network.TLS_DETECTORS}
        if type(yellow_time) is dict:
            self.yellow_time = yellow_time
        else:
            self.yellow_time = {identifiant: yellow_time for identifiant in network.TLS_DETECTORS}
        self.step_length = 1

        # self.network.TL_IDS = ['221'] # For testing only one traffic light
        self.time = {k: 0 for k in self.network.TL_IDS}
        self.next_phase = {k: 0 for k in self.network.TL_IDS}
        self.priority_pile = {k: [] for k in self.network.TL_IDS}
        self.prio = {k: False for k in self.network.TL_IDS}
        self.current_cycle = {k: [] for k in self.network.TL_IDS}
        self.current_yellow_time = {k: 0 for k in self.network.TL_IDS}
        self.is_phase = {k: True for k in self.network.TL_IDS}
        self.nb_switch = {k: 0 for k in self.network.TL_IDS}
        self.nb_phases = {k: 0 for k in self.network.TL_IDS}
        self.phases_occurences = {k: {} for k in self.network.TL_IDS}
        self.agents = {k: AnalyticPlusAgent(self, id_tls_program=k, T=T, T_max=T_max,
                                            yellow_time=self.yellow_time) for k in self.network.TL_IDS}
        self.phases_durations = {identifiant: [] for identifiant in network.TLS_DETECTORS}
        self.current_phase_duration = {identifiant: 0 for identifiant in network.TLS_DETECTORS}

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
                if lane_id[0] == ':':  # skip internal lanes
                    continue
                self.traci.lane.subscribe(lane_id, varIDs=varIDs_lane)
            return True
        else:
            lane_data = self.traci.lane.getAllSubscriptionResults()
            for id_tls in self.network.TL_IDS:
                agent = self.agents[id_tls]
                agent.update(lane_data)
                current_phase = self.traci.trafficlight.getPhase(id_tls)
                current_state = self.traci.trafficlight.getRedYellowGreenState(id_tls)
                # if current_phase not in self.TLS_DETECTORS[id_tls]: # Handle the yellow phases
                if 'y' in current_state:
                    if self.yellow_time[id_tls] - self.current_yellow_time[id_tls] <= 0:
                        # maybe there is a second yellow phase
                        self.switch_yellow(agent)
                        assert self.traci.trafficlight.getPhase(id_tls) != current_phase
                        if not agent.sumo_phases[self.traci.trafficlight.getPhase(id_tls)]['isYellow']:  # not a yellow phase, switch to next green
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

                    # if not self.prio[id_tls]:
                    # if len(self.priority_pile[id_tls]) == 0 and self.prio[id_tls]:
                    #     self.add_prio_phases(id_tls)
                    # if self.agents[id_tls].current_sumo_phase['id'] == current_phase:
                    self.time[id_tls] += 1
                    self.current_phase_duration[id_tls] += 1

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
        agent.green_duration = green_time

        if next_phase != current_phase:
            self.next_phase[id_tls] = next_phase
            agent.update_last_on(agent.sumo_phases[next_phase], agent.sumo_phases[current_phase], time)
            assert not agent.sumo_phases[next_phase]['isYellow']
            # switch to hopefully the next yellow phase
            self.switch_yellow(agent)
            assert agent.sumo_phases[self.traci.trafficlight.getPhase(id_tls)]['isYellow']
            self.time[id_tls] = 0
            if self.current_phase_duration[id_tls] > 2:
                self.phases_durations[id_tls].append((current_phase, self.current_phase_duration[id_tls]))
            self.current_phase_duration[id_tls] = 0

    def _start_agents(self):
        """
        Start an agent at the beginning of the simulation.
        """
        for tl in self.network.TL_IDS:
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


LaneData: TypeAlias = dict[str, dict[int, Union[float, list[str]]]]
actionID: TypeAlias = int
VEH_NUM = tc.LAST_STEP_VEHICLE_NUMBER
VEH_LIST = tc.LAST_STEP_VEHICLE_ID_LIST


class sumoPhase(TypedDict):
    sumo_movement_idxs: list[int]
    isYellow: bool
    greenPhase: Optional[int]
    yellowPhase: Optional[int]
    yellowDuration: Optional[int]
    id: int
    movement_ids: set[int]
    last_on_time: float


class AnalyticPlusAgent():
    def __init__(self,
                 env,  # strategy environment
                 id_tls_program,
                 T=150,
                 T_max=180,
                 yellow_time=None):
        self.ID = id_tls_program
        self.env = env
        self.step_length = 1
        self.T = T
        self.T_max = T_max
        self.current_sumo_phase: sumoPhase
        self.phase_stats: dict[actionID, dict[str, float]]

        self.green_duration: float = 0
        self.next_act_time: float = 0
        self.last_act_time: float = -1
        self.clearing_time: float = yellow_time
        self.interval_length = 600
        # assert self.action_interval % self.env.step_length == 0

    def choose_action(self, time) -> tuple[actionID, float]:
        """Returns the `action_phase_id` of the next action."""
        self.update_clear_green_time(time)

        if all([np.isclose(x.green_time, 0) for x in self.movements.values()]):
            sumo_phase_id = self.current_sumo_phase['id']
            action = self._sumo2action_phases[sumo_phase_id]
            green_time = self.minimum_green
        else:
            # self.update_priority_idx()
            phases_priority: dict[actionID, dict[str, float]] = {action_idx: {'priority': 0,
                                                                              'green_time': 0}
                                                                 for action_idx in self.action_phases.keys()}

            for action_phase_id, sumo_phase_id in self.action_phases.items():
                phase = self.sumo_phases[sumo_phase_id]
                priority, phase_green = self.get_priority(phase)
                phases_priority[action_phase_id]['priority'] = priority
                phases_priority[action_phase_id]['green_time'] = phase_green

            action = max(phases_priority, key=lambda key: phases_priority[key]['priority'])
            chosen_phase = self.sumo_phases[self.action_phases[action]]
            if False:  # chosen_phase == self.clearing_phase: # shouldn't happen
                raise ValueError('Chosen phase is clearing phase')
                green_time = self.clearing_time[self.ID]
            else:
                green_time = phases_priority[action]['green_time']

            assert green_time > 0

        self.stabilise(time, ghat=green_time)  # updates action queue
        if len(self.action_queue):
            action_phase_id, green_time = self.action_queue.popleft()
            assert green_time > 0
            return action_phase_id, green_time

        return action, green_time

    def get_priority(self, phase):
        phase_green = 0
        phase_saturations = 0
        for move_id in phase['movement_ids']:
            move = self.movements[move_id]
            phase_green = max(phase_green, move.green_time)
            phase_saturations += move.max_saturation
            # penalties += 0 if move_id in self.current_sumo_phase['movement_ids'] else self.clearing_time
        penalties = self.clearing_time[self.ID] * (self.current_sumo_phase['id'] != phase['id'])
        nhat = phase_saturations * phase_green
        priority = nhat / (penalties + phase_green + self.clearing_time[self.ID])
        return priority, phase_green

    def stabilise(self, time, ghat, mode='phase'):
        """
        Implements the stabilisation mechanism of the algorithm, updates the action queue with phases that need to be prioritized
        """
        T = self.T  # seconds
        T_max = self.T_max  # seconds
        # max_arr_rate = max([x.arr_rate for x in self.movements.values()])
        sum_Qphase = sum([stats['ave_arr_rate'] / self.phase_saturations[action_idx] for action_idx, stats in self.phase_stats.items()])
        T_res = T * (1 - sum_Qphase) - self.clearing_time[self.ID] * len(self.action_phases)

        phase_priority_list: list[tuple[sumoPhase, float]] = []

        for action_id, sumo_phase_id in self.action_phases.items():
            phase = self.sumo_phases[sumo_phase_id]
            Q = self.phase_stats[action_id]['arr_rate']
            Q_ave = self.phase_stats[action_id]['ave_arr_rate']
            # Q = np.sum([self.movements[id].arr_rate for id in phase["movement_ids"]])
            Q_max = self.phase_saturations[action_id]
            assert Q <= Q_max
            if phase == self.current_sumo_phase:  # currently active
                # waiting_time = 0
                continue
            else:
                waiting_time = time - phase["last_on_time"]

            phase_green = max([self.movements[id].green_time for id in phase["movement_ids"]])
            assert phase_green == self.phase_stats[action_id]['green_time'], (self.phase_stats[action_id]['green_time'], phase_green)
            z = waiting_time + self.clearing_time[self.ID] + ghat
            n_crit = Q_ave * T * ((T_max - z) / (T_max - T))
            # n_crit = max(n_crit, 0) # n_crit can be negative when z>T_max, which should ideally never happen
            # assert waiting_time <= T_max*1.5
            # waiting = ghat * Q_max
            waiting = ghat * Q
            if waiting > n_crit:
                # if self.ID == DEBUG_ID:
                #     print(f"id: {action_id}, ghat: {ghat}, phase_green: {phase_green}, waiting_time: {waiting_time:.2f}, waiting: {waiting:.2f}, n_crit: {n_crit:.2f}, Q: {Q}, Q_max: {Q_max}")
                green_max = (Q_ave / Q_max) * T + (Q_max / self.sum_Qmax) * T_res
                green_max = green_max - (green_max % self.env.step_length)
                priority, _ = self.get_priority(phase)
                # green_time = min(phase_green,green_max)
                green_time = green_max
                phase_priority_list.append((action_id, priority, green_time))

        # # Sort the list by priority in descending order
        phase_priority_list = sorted(phase_priority_list, key=lambda x: x[1], reverse=True)
        # Add the phases to the queue

        for action_phase_id, _, green_time in phase_priority_list:
            if green_time <= 0:
                continue
            if (action_phase_id not in [k for k, v in self.action_queue]):
                assert green_time > 0
                self.action_queue.append((action_phase_id, green_time))
            else: # update needs 
                idx = [i[0] for i in self.action_queue].index(action_phase_id)
                self.action_queue[idx] = (action_phase_id, green_time)
            # if action_id in [i[0] for i in self.action_queue]:
            #     continue
                # return

    def _start_agent(self):
        """
        Start the agent at the beginning of the simulation.
        """
        pass

    def init_movements(self):
        # movements are road-road mappings
        connections = self.traci.trafficlight.getControlledLinks(self.ID)
        # movements can be a collection of connections
        self.in_lanes_length = {}
        self.out_lanes_length = {}
        in_lanes = set()
        out_lanes = set()
        _move_id_mapper = {}
        for sumo_idx, [(in_lane, out_lane, internal)] in enumerate(connections):
            self.in_lanes_length.update(
                {in_lane: self.traci.lane.getLength(in_lane)})
            self.out_lanes_length.update(
                {out_lane: self.traci.lane.getLength(out_lane)})
            in_edge = self.traci.lane.getEdgeID(in_lane)
            out_edge = self.traci.lane.getEdgeID(out_lane)
            in_lane_length = self.traci.lane.getLength(in_lane)
            out_lane_length = self.traci.lane.getLength(out_lane)
            up_max_speed = self.traci.lane.getMaxSpeed(in_lane)
            down_max_speed = self.traci.lane.getMaxSpeed(in_lane)
            max_speed = min([up_max_speed, down_max_speed])
            move_tuple = (in_edge, out_edge)
            move_id = _move_id_mapper.setdefault(move_tuple, len(_move_id_mapper))
            movement = self.movements.setdefault(move_id, Movement(move_id, self, in_lane_length, out_lane_length, max_speed))
            movement.in_lanes.add(in_lane)
            movement.out_lanes.add(out_lane)
            movement.sumo_movement_idxs.add(sumo_idx)

            in_lanes.add(in_lane)
            out_lanes.add(out_lane)

            # associate phases to movements, only use green phases
            for action_phase_id, sumo_phase_id in self.action_phases.items():
                phase = self.sumo_phases[sumo_phase_id]
                if sumo_idx in phase['sumo_movement_idxs']:
                    movement.phases.add(action_phase_id)
                    phase['movement_ids'].add(move_id)  # phases serve movements

        for mid, movement in self.movements.items():
            movement.max_saturation *= len(movement.in_lanes)
        return in_lanes, out_lanes

    def init_phases(self):
        logic = self.traci.trafficlight.getAllProgramLogics(self.ID)[-1]
        one_hot_movements = {}
        # we have pairs of green and yellow phases
        # assert len(logic.phases) % 2 == 0
        phase_idx = 0
        prev_ohe = ""
        for sumo_idx, sumo_phase in enumerate(logic.phases):
            isYellow = False
            # green phases
            active_connections = [
                m.start() for m in re.finditer(r'[^r]', sumo_phase.state)]
            ohe = [1 * (s == 'r') for s in sumo_phase.state]  # one hot encoded
            ohe_move = one_hot_movements.setdefault(str(ohe), [])
            ohe_move.append(sumo_idx)
            # isYellow = str(ohe)==prev_ohe # True
            isYellow = 'y' in sumo_phase.state
            if sum(ohe) == len(ohe):  # all red phase
                raise NotImplementedError('All red phase not implemented')
                sumo_idx = -1
                continue
            # elif not isYellow and sumo_phase.minDur==sumo_phase.maxDur and sumo_phase.minDur<=3:
            #     continue # simplifies phases loaded in analytic agent.

            self.sumo_phases[sumo_idx] = {'sumo_movement_idxs': active_connections,
                                          'isYellow': isYellow,
                                          'greenPhase': sumo_idx - 1 if isYellow else None,
                                          'yellowPhase': None,
                                          'yellowDuration': sumo_phase.duration if isYellow else None,
                                          'id': sumo_idx,
                                          'movement_ids': set(),
                                          'last_on_time': -2
                                          }
            if isYellow:  # assumes phases are in consecutive order
                self.sumo_phases[sumo_idx - 1]['yellowPhase'] = sumo_idx
                continue  # skip the next line
            self.action_phases[phase_idx] = sumo_idx
            phase_idx += 1
            prev_ohe = str(ohe)


    def set_phase(self, sumo_phase_id: int, duration: float = 1e6):
        """
        Sets the phase of the agent to the indicated phase using the API.

        Parameters
        ----------
        sumo_phase_id : int
            ID of the phase to be activated. Should be a valid SUMO phase index.
        """
        # sumo_idx = self.current_sumo_phase['id']
        self.traci.trafficlight.setPhase(self.ID, sumo_phase_id)
        self.traci.trafficlight.setPhaseDuration(self.ID, duration)  # sets phase to an arbitrarily long duration
        self.current_sumo_phase = self.sumo_phases[sumo_phase_id]

    def update_arr_dep_veh_num(self, lane_data: LaneData):
        """
        Updates the number of vehicles that arrived and departed, aggregated at the phase level.

        Parameters
        ----------
        lane_data : dict[str, dict[int, Union[float, list[str]]]]
            Data output from SUMO aggregated at the lane level.
        """
        for movement in self.movements.values():
            movement.update_arr_dep_veh_num(lane_data)
        # for action_id, sumo_phase_id in self.phases.items():
        #     print(self.phases, self.sumo_phases.keys(), self.env.TLS_DETECTORS[self.ID].keys(), self.ID)
        #     phase = self.sumo_phases[sumo_phase_id]
        #     current_vehs = set()

        #     # Collect vehicles detected by numerical detectors for the current phase
        #     detectors = self.env.TLS_DETECTORS[self.ID][sumo_phase_id]['numerical']
        #     for det in detectors:
        #         current_vehs.update(self.traci.lanearea.getLastStepVehicleIDs(det))

        #     # Calculate arrivals and departures
        #     prev_vehs = phase.get('prev_vehs', set())
        #     dep_vehs = len(prev_vehs - current_vehs)
        #     arr_vehs = len(current_vehs - prev_vehs)

        #     # Update phase-level statistics
        #     phase['arr_vehs_num'] = phase.get('arr_vehs_num', deque([0] * self.interval_length, maxlen=self.interval_length))
        #     phase['arr_vehs_num'].append(arr_vehs)
        #     phase['total_dep'] = phase.get('total_dep', 0) + dep_vehs
        #     phase['total_arr'] = phase.get('total_arr', 0) + arr_vehs
        #     phase['prev_vehs'] = current_vehs

    def update_last_on(self, action: sumoPhase, phase: sumoPhase, time: float):
        """
        Updates last activation time of movement.

        The parameter `self.last_on_time` is necessary to compute the waiting time
        --- elapsed duration from the movement was last active.

        Parameters
        ----------
        action : sumoPhase
            The next queued phase to be activated.
        phase : sumoPhase
            The currently active phase.
        lane_data : LaneData
            Data output from SUMO aggregated at the lane level.
        """
        for movement in self.movements.values():
            movement.update_last_on(action, phase)
        phase["last_on_time"] = time

    def reset(self, traci):
        """
        Resets the set containing the vehicle ids for each movement and the arr/dep vehicles numbers as well as the waiting times
        the set represents the vehicles waiting on incoming lanes of the movement
        """
        self.traci = traci
        self.movements: dict[actionID, Movement] = {}
        self.sumo_phases: dict[int, sumoPhase] = {}
        self.action_phases: dict[actionID, int] = {}

        self.init_phases()
        self.in_lanes, self.out_lanes = self.init_movements()

        self.set_phase(self.traci.trafficlight.getPhase(self.ID))
        for move in self.movements.values():
            move.reset()
        self.total_rewards = []
        self.next_act_time = 0
        self.last_act_time = -1
        self.action_type = 'act'
        self.phase_stats = {action_idx: {'arr_rate': 0.,
                                         'green_time': 0.,
                                         'last_on_time': 0} for action_idx in self.action_phases.keys()}
        self.action_queue: deque[tuple[actionID, int]] = deque([])
        self._sumo2action_phases = {v: k for k, v in self.action_phases.items()}
        self.minimum_green: float = self.env.min_phase_durations[self.ID]
        self.desired_phase = 0

        self.phase_saturations = {}
        for action_id, sumo_phase_id in self.action_phases.items():
            phase = self.sumo_phases[sumo_phase_id]
            saturations = [self.movements[id].max_saturation for id in phase["movement_ids"]]
            self.phase_saturations[action_id] = np.sum(saturations)
        self.sum_Qmax = sum(self.phase_saturations.values())

    def update_priority_idx(self):
        """
        Updates the priority of the movements of the intersection, the higher priority the more the movement needs to get a green lights
        :param time: the time in the simulation, at this moment only integer values are supported
        """
        for move_id, movement in self.movements.items():
            penalty_term = 0
            # incurs a penalty if need to change phase
            if move_id not in self.current_sumo_phase['movement_ids']:
                penalty_term = movement.clearing_time  # naive approximation
            movement.priority = ((movement.green_time * movement.max_saturation) /
                                 (movement.green_time + movement.clearing_time + penalty_term))

    def update_clear_green_time(self, time):
        """
        Updates the green times of the movements of the intersection
        :param time: the time in the simulation, at this moment only integer values are supported
        """
        for movement in self.movements.values():
            green_time = movement.get_green_time(time)
            movement.green_time = green_time
        for action_id, sumo_id in self.action_phases.items():
            phase = self.sumo_phases[sumo_id]
            stats = self.phase_stats[action_id]
            arr_rates = []
            ave_arr_rate = []
            stats['green_time'] = 0
            for move_id in phase['movement_ids']:
                stats['green_time'] = max(stats['green_time'], self.movements[move_id].green_time)
                arr_rates.append(self.movements[move_id].arr_rate)
                ave_arr_rate.append(self.movements[move_id].ave_arr_rate)
            stats['arr_rate'] = sum(arr_rates)
            stats['ave_arr_rate'] = sum(ave_arr_rate)

    # def apply_action(self, api, action: tuple[int, Union[float, None]], lane_data: LaneData):
    #     """Converts the RL action into its implementation in the TRACI api.

    #     Args:
    #         api (_type_): _description_
    #         action (tuple[int, Union[float, None]]): _description_
    #         time (_type_): _description_
    #         lane_data (LaneData): _description_
    #     """
    #     action_idx, green_time = action
    #     self.chosen_phase = self.sumo_phases[self.action_phases[action_idx]]
    #     if self.action_type == "act":
    #         if green_time is None:
    #             green_time = self.action_interval
    #         self.green_time = green_time
    #         assert self.green_time != 0

    #         self.last_act_time = self.env.time
    #         if self.current_sumo_phase['id'] != self.chosen_phase['id']:
    #             self.update_last_on(self.chosen_phase, self.current_sumo_phase, lane_data)
    #             if self.current_sumo_phase['yellowPhase'] is None:
    #                 raise NotImplementedError
    #                 self.set_phase(self.clearing_phase['id'])
    #             else:
    #                 self.set_phase(self.current_sumo_phase['yellowPhase'])
    #             self.next_act_time = self.env.time + self.clearing_time + self.green_time
    #             self.action_type = "update"
    #         else:
    #             self.next_act_time = self.env.time + self.green_time

    def update(self, lane_data):
        self.update_arr_dep_veh_num(lane_data)  # lane data not needed for arrival rates
        # if (self.action_type == 'update' and
        #         isclose(self.env.time,(self.last_act_time+self.clearing_time))):
        #     self.set_phase(self.chosen_phase['id'])
        #     self.action_type = "act"
        #     assert self.current_sumo_phase['id'] == self.chosen_phase['id']

    # def calculate_reward(self, lanes_count):
    #     reward = self.get_reward(lanes_count)
    #     self.total_rewards += [reward]
    #     # self.reward_count += 1
    #     return reward

    # @property
    # def time_to_act(self):
    #     return isclose(self.env.time, self.next_act_time)


class Movement:
    """Helper class to contain attributes of movements
    """

    def __init__(self, id, intersection: Agent, in_lane_length: float, out_lane_length: float,
                 max_speed: float):
        self.ID = id
        # sumo does not group movements, but in principle this can be a set of lane pairs
        self.in_lanes = set()
        self.out_lanes = set()
        self.sumo_movement_idxs = set()
        self.in_length = in_lane_length
        self.out_length = out_lane_length
        self._intersection = intersection
        self._step_length = self._intersection.env.step_length
        self.phases: set[actionID] = set()

        VEH_LENGTH = 5
        MIN_GAP = 2.5
        calc_capacity = lambda x: 3600 / ((VEH_LENGTH + MIN_GAP) / x + self._step_length)
        self.max_speed = max_speed
        self.capacity_vph = calc_capacity(self.max_speed)# * 0.7  # add 10% buffer
        # print(self.capacity_vph) *0.7 == 1636 vph
        self.max_saturation = self.capacity_vph / 3600  # 2000 vehs/hour -> vehicles/sec/lane
        self.clearing_time = intersection.clearing_time[intersection.ID]  # all red time
        self.pass_time = (self.in_length / self.max_speed)

        self.reset()

    def reset(self):
        self.interval_window: float = 150  # seconds
        self.interval_length: int = int(self.interval_window / self._step_length)
        self._min_interval_length: int = ceil(self.pass_time / self._step_length)  # ceil handles cases of short roads
        self.interval_length = max(self.interval_length, self._min_interval_length)
        self.ave_interval_length = self.interval_length * 10

        self.interval_window = self.interval_length * self._step_length
        self.ave_interval_window = self.ave_interval_length * self._step_length
        self.prev_vehs = set()
        # initialize to small arrival rates
        self.arr_vehs_num: deque[int] = deque([0 for i in range(self.interval_length)], maxlen=self.interval_length)  # type: ignore
        self.ave_arr_vehs_num: deque[int] = deque([0 for i in range(self.ave_interval_length)], maxlen=self.ave_interval_length)  # type: ignore
        # self.dep_vehs_num: deque[int] = deque([0 for i in range(len(self.interval_length))], maxlen=self.interval_length) # type: ignore
        self.total_dep = 0
        self.total_delayed_arr = 0  # FIXME: unused for now
        self.total_arr = 0
        self.last_on_time = 0
        # self.waiting_time = 0 # red time
        # self.max_waiting_time = 0
        # self.waiting_time_list = []
        self.arr_rate: float = 0
        self.priority: float = 0
        self.green_time: float = 0

    def update_last_on(self, action: sumoPhase, phase: sumoPhase):
        if self.ID not in action['movement_ids'] and self.ID in phase['movement_ids']:  # switch to new phase
            self.last_on_time = self._intersection.env.time
        elif self.ID in action['movement_ids'] and self.ID not in phase['movement_ids']:  # will be deactivated
            self.last_on_time = -1  # currently active

    def get_green_time(self, time):
        """
        Gets the predicted green time needed to clear the movement
        :param time: the current timestep
        :param current_movements: a list of movements that are currently enabled
        :returns: the predicted green time of the movement

        http://dx.doi.org/10.1088/1742-5468/2008/04/P04019
        """
        if len(self.arr_vehs_num) >= self.interval_length:
            arrivals = self.arr_vehs_num
            interval_window = self.interval_window


        with np.errstate(divide='ignore', invalid='ignore'):
            self.arr_rate = (np.nan_to_num(np.divide(sum(arrivals), interval_window), nan=0))
            self.ave_arr_rate = np.nan_to_num(np.divide(self.total_arr, time), nan=0)
            # self.ave_arr_rate = np.nan_to_num(np.divide(sum(self.ave_arr_vehs_num), self.ave_interval_window), nan=0)
            # len(self.in_lanes))
            assert self.arr_rate < self.max_saturation, (self.arr_rate, self.max_saturation)
        # self.arr_rate = np.nan_to_num(np.divide(self.total_arr,
        #                                         self._intersection.env.time),
        #                               nan=0)/len(self.in_lanes) # FIXME: adapt to surges in Qarr
        # dep = sum(self.dep_vehs_num)/len(self.in_lanes)
        dep = self.total_dep  # /len(self.in_lanes)
        arr = self.total_delayed_arr  # /len(self.in_lanes)
        assert self.total_arr + sum(arrivals) >= dep, (self.total_arr, dep)
        clearing_time = self.clearing_time

        green_time = (self.arr_rate * clearing_time + arr - dep) / (self.max_saturation - self.arr_rate)
        if (self.max_saturation - self.arr_rate) == 0:
            print(f'WARNING: unhandled possible infinite green_time: {green_time, self._intersection.env.sumoCMD}')

        # assert green_time >= 0, f'green time calculated is {green_time}'

        green_time = max(0, green_time - (green_time % self._step_length))  # round down

        return green_time

    def update_arr_dep_veh_num(self, lane_data: LaneData):
        """
        Updates the list containing the number vehicles that arrived and departed
        :param lanes_vehs: a dictionary with lane ids as keys and number of vehicles as values
        """
        current_vehs = set()

        for lane_id in self.in_lanes:
            current_vehs.update(lane_data[lane_id][VEH_LIST])

        # detectors = self._intersection.env.TLS_DETECTORS[self._intersection.ID][self.current_sumo_phase]['numerical']
        # for det in detectors:
        #     current_vehs.update(self.traci.lanearea.getLastStepVehicleIDs(det))

        dep_vehs = len(self.prev_vehs - current_vehs)
        arr_vehs = len(current_vehs - self.prev_vehs)
        self.arr_vehs_num.append(arr_vehs)
        self.ave_arr_vehs_num.append(arr_vehs)
        if len(self.arr_vehs_num) >= self._min_interval_length:
            # self.total_delayed_arr += self.arr_vehs_num[-self._min_interval_length]
            self.total_delayed_arr += self.arr_vehs_num[-self._min_interval_length]
        # self.dep_vehs_num.append(dep_vehs)
        self.prev_vehs = current_vehs
        self.total_dep += dep_vehs
        self.total_arr += arr_vehs

        # b = self.total_delayed_arr + sum(list(self.arr_vehs_num)[-self._min_interval_length:])
        # assert self.total_arr == b, (self.total_arr, b)

