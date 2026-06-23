from __future__ import annotations

import re
from collections import deque
from typing import TypedDict, Optional, Union, TypeAlias
from math import ceil
import traci.constants as tc


from sumo_experiments.strategies import Strategy

DEBUG = False


class AnalyticPlusStrategy(Strategy):
    """
    Implement an Analytic+ agent for all intersections of the Bologna network.

    Lämmer, S., & Helbing, D. (2008). Self-control of traffic lights and vehicle flows in urban road networks. Journal of Statistical Mechanics: Theory and Experiment, 2008(04), P04019.
    """

    def __init__(self, network, min_phase_duration=3, T=150, T_max=180, yellow_time=3, intelligent_intersections=None, stabilization=True):
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
        self.time = {tls_id: 0 for tls_id in self.network.TL_IDS}
        self.next_phase = {tls_id: 0 for tls_id in self.network.TL_IDS}
        self.priority_pile = {tls_id: [] for tls_id in self.network.TL_IDS}
        self.prio = {tls_id: False for tls_id in self.network.TL_IDS}
        self.current_cycle = {tls_id: [] for tls_id in self.network.TL_IDS}
        self.current_yellow_time = {tls_id: 0 for tls_id in self.network.TL_IDS}
        self.is_phase = {tls_id: True for tls_id in self.network.TL_IDS}
        self.nb_switch = {tls_id: 0 for tls_id in self.network.TL_IDS}
        self.nb_phases = {tls_id: 0 for tls_id in self.network.TL_IDS}
        self.phases_occurences = {tls_id: {} for tls_id in self.network.TL_IDS}
        self.agents = {tls_id: AnalyticPlusAgent(self, id_tls_program=tls_id, T=T, T_max=T_max,
                                            yellow_time=self.yellow_time, stabilization=stabilization) for tls_id in self.network.TL_IDS}
        self.phases_durations = {identifier: [] for identifier in network.TLS_DETECTORS}
        self.current_phase_duration = {identifier: 0 for identifier in network.TLS_DETECTORS}
        if intelligent_intersections is None:
            self.intelligent_intersections = network.TL_IDS
        else:
            self.intelligent_intersections = intelligent_intersections
        self.exclude_consumption = 0

    def switch_yellow(self, agent):
        """
        Switch the traffic light tls_id to yellow
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
            return True
        else:
            det_data = self.traci.lanearea.getAllSubscriptionResults()
            self.zeus_monitor.begin_window("all_agents")
            for tls_id in self.intelligent_intersections:
                agent = self.agents[tls_id]
                agent.update(det_data)
                current_phase = self.traci.trafficlight.getPhase(tls_id)
                current_state = self.traci.trafficlight.getRedYellowGreenState(tls_id)
                # if current_phase not in self.TLS_DETECTORS[tls_id]: # Handle the yellow phases
                if 'y' in current_state:
                    if self.yellow_time[tls_id] - self.current_yellow_time[tls_id] <= 0:
                        # maybe there is a second yellow phase
                        self.switch_yellow(agent)
                        assert self.traci.trafficlight.getPhase(tls_id) != current_phase
                        # print(current_phase, current_state, tls_id, self.next_phase[tls_id])
                        if not agent.sumo_phases.get(self.traci.trafficlight.getPhase(tls_id), {'isYellow': False})['isYellow']:  # not a yellow phase, switch to next green
                            self.traci.trafficlight.setPhase(tls_id, self.next_phase[tls_id])
                            agent.current_sumo_phase = agent.sumo_phases[self.next_phase[tls_id]]
                            self.current_cycle[tls_id].append(self.next_phase[tls_id])
                        self.current_yellow_time[tls_id] = 0
                    else:
                        self.current_yellow_time[tls_id] += 1
                else:
                    assert current_phase == self.next_phase[tls_id]
                    if current_phase not in self.phases_occurences[tls_id]:
                        self.phases_occurences[tls_id][current_phase] = 1
                    else:
                        self.phases_occurences[tls_id][current_phase] += 1

                    if self.time[tls_id] >= self.min_phase_durations[tls_id]:
                        # start logic here
                        self.analyticplus_logic(tls_id)
                    self.time[tls_id] += 1
                    self.current_phase_duration[tls_id] += 1
            results = self.zeus_monitor.end_window("all_agents")
            self.energy_consumption += self.get_energy_consumption(results)

    def analyticplus_logic(self, tls_id):
        agent = self.agents[tls_id]
        if self.time[tls_id] >= agent.green_duration:
            self.switch_next_phase(tls_id)

    def switch_next_phase(self, tls_id):
        """
        Switch the traffic light tls_id to the next
        """
        agent = self.agents[tls_id]
        self.nb_switch[tls_id] += 1
        current_phase = self.traci.trafficlight.getPhase(tls_id)
        time = self.traci.simulation.getTime()

        action_id, green_time = agent.choose_action(time)
        next_phase = agent.action_phases[action_id]
        agent.green_duration = green_time

        if next_phase != current_phase: # switch
            self.next_phase[tls_id] = next_phase
            agent.update_last_on(agent.sumo_phases[next_phase], agent.sumo_phases[current_phase], time)
            assert not agent.sumo_phases[next_phase]['isYellow']
            # switch to hopefully the next yellow phase
            # print(self.traci.trafficlight.getRedYellowGreenState(tls_id), tls_id)
            self.switch_yellow(agent)
            assert agent.sumo_phases[self.traci.trafficlight.getPhase(tls_id)]['isYellow']
            self.time[tls_id] = 0
            if self.current_phase_duration[tls_id] > 2:
                self.phases_durations[tls_id].append((current_phase, self.current_phase_duration[tls_id]))
            self.current_phase_duration[tls_id] = 0


    def _start_agents(self):
        """
        Start an agent at the beginning of the simulation.
        """
        for tls_id in self.intelligent_intersections:
            tl_logic = self.traci.trafficlight.getAllProgramLogics(tls_id)[-1]
            nb_phase = 0
            for phase in tl_logic.phases:
                phase.duration = 10000
                phase.maxDur = 10000
                phase.minDur = 10000
                nb_phase += 1
            self.nb_phases[tls_id] = nb_phase
            self.traci.trafficlight.setProgramLogic(tls_id, tl_logic)
            self.traci.trafficlight.setPhase(tls_id, 0)
            self.traci.trafficlight.setPhaseDuration(tls_id, 10000)
            self.agents[tls_id].reset(self.traci)
        # Subscribe to all lane area detectors for batch reading
        sub_vars = [
            tc.LAST_STEP_VEHICLE_NUMBER,  # current vehicles on detector (queue)
            tc.VAR_INTERVAL_NUMBER,        # vehicles passed in current (partial) interval
            tc.VAR_LAST_INTERVAL_NUMBER,   # vehicles passed in last complete interval
        ]
        subscribed = set()
        for tls_id in self.intelligent_intersections:
            tls_dets = self.network.TLS_DETECTORS[tls_id]
            for phase_id, det_groups in tls_dets.items():
                for det in det_groups.get('numerical', []):
                    if det not in subscribed:
                        self.traci.lanearea.subscribe(det, sub_vars)
                        subscribed.add(det)
        self.started = True


actionID: TypeAlias = int


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
                 env: AnalyticPlusStrategy,  # strategy environment
                 id_tls_program,
                 T=150,
                 T_max=180,
                 yellow_time=None,
                 stabilization=True):
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
        self.stabilization = stabilization
        # assert self.action_interval % self.env.step_length == 0

    def choose_action(self, time) -> tuple[actionID, float]:
        """Returns the `action_phase_id` of the next action."""
        self.update_clear_green_time(time)

        if all(stats['green_time'] == 0 for stats in self.phase_stats.values()):
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
                priority, phase_green = self.get_priority(phase, action_phase_id)
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

        if self.stabilization:
            self.stabilise(time, ghat=green_time)  # updates action queue
        if len(self.action_queue):
            action_phase_id, green_time = self.action_queue.popleft()
            assert green_time > 0
            return action_phase_id, green_time

        return action, green_time

    def get_priority(self, phase, action_id):
        phase_green = self.phase_stats[action_id]['green_time']
        phase_saturations = self.phase_saturations[action_id]
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
        sum_Qphase = sum(stats['ave_arr_rate'] / self.phase_saturations[action_idx] for action_idx, stats in self.phase_stats.items())
        T_res = T * (1 - sum_Qphase) - self.clearing_time[self.ID] * len(self.action_phases)

        phase_priority_list: list[tuple[sumoPhase, float]] = []

        for action_id, sumo_phase_id in self.action_phases.items():
            phase = self.sumo_phases[sumo_phase_id]
            Q = self.phase_stats[action_id]['arr_rate']
            Q_ave = self.phase_stats[action_id]['ave_arr_rate']
            Q_max = self.phase_saturations[action_id]

            if phase == self.current_sumo_phase:  # currently active
                # waiting_time = 0
                continue
            else:
                waiting_time = time - phase["last_on_time"]

            phase_green = self.phase_stats[action_id]['green_time']
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
                priority, _ = self.get_priority(phase, action_id)
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
                # raise NotImplementedError('All red phase not implemented')
                # sumo_idx = -1
                pass
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
            if sum(ohe) == len(ohe): continue # do not build action phase for all red
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

    def update_arr_dep_veh_num(self, det_data):
        """
        Updates arrival counts and queue from pre-fetched subscription data.
        Uses SUMO's interval counters instead of Python-side sliding windows.
        """
        VEH_NUM = tc.LAST_STEP_VEHICLE_NUMBER
        CUR_INT = tc.VAR_INTERVAL_NUMBER
        LAST_INT = tc.VAR_LAST_INTERVAL_NUMBER
        tls_detectors = self.env.network.TLS_DETECTORS[self.ID]

        for action_id, sumo_phase_id in self.action_phases.items():
            if sumo_phase_id not in tls_detectors:
                continue

            dets = tls_detectors[sumo_phase_id]
            tracker = self.phase_tracker[action_id]
            num_dets = dets.get('numerical', ())

            # All values come from a single batch dict lookup — no TraCI calls
            queue = 0
            interval_count = 0
            last_interval_count = 0
            for d in num_dets:
                if d in det_data:
                    dd = det_data[d]
                    queue += dd[VEH_NUM]
                    interval_count += dd[CUR_INT]
                    last_interval_count += dd[LAST_INT]

            # Maintain cumulative total via interval-count delta (O(1))
            prev = tracker['prev_interval_count']
            delta = interval_count - prev if interval_count >= prev else interval_count
            tracker['total_arr'] += delta
            tracker['prev_interval_count'] = interval_count

            tracker['queue'] = queue
            tracker['interval_count'] = interval_count
            tracker['last_interval_count'] = last_interval_count

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
            self.phase_saturations[action_id] = sum(self.movements[mid].max_saturation for mid in phase["movement_ids"])
        self.sum_Qmax = sum(self.phase_saturations.values())

        # Phase-level detector tracking (interval-based, no Python-side sliding windows)
        self.det_interval_window = getattr(self.env.network, 'detector_period', self.interval_length)
        self.phase_tracker = {}
        for action_id in self.action_phases:
            self.phase_tracker[action_id] = {
                'prev_interval_count': 0,   # for cumulative delta tracking
                'total_arr': 0,             # cumulative arrivals since sim start
                'queue': 0,                 # vehicles currently on numerical detectors
                'interval_count': 0,        # vehicles passed in current interval
                'last_interval_count': 0,   # vehicles passed in last complete interval
            }

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
        Updates the green times at the phase level using SUMO interval-based
        detector data.  All heavy counting is done in SUMO's C++ engine;
        Python only reads integers and does arithmetic.
        :param time: the time in the simulation
        """
        P = self.det_interval_window  # detector period (seconds)
        time_in_interval = time % P if P > 0 else 0
        step = self.env.step_length

        for action_id in self.action_phases:
            stats = self.phase_stats[action_id]
            tracker = self.phase_tracker[action_id]

            clearing_time = self.clearing_time[self.ID]

            # Arrival rate from last-complete + current-partial interval
            total_count = tracker['last_interval_count'] + tracker['interval_count']
            total_window = min(P + time_in_interval, max(time, step))

            # Plain Python division — total_window is always > 0
            arr_rate = total_count / total_window if total_window > 0 else 0.0
            ave_arr_rate = tracker['total_arr'] / time if time > 0 else 0.0

            self.phase_saturations[action_id] = max(self.phase_saturations[action_id], arr_rate * 1.1)
            Q_max = self.phase_saturations[action_id]
            # Green time (Lämmer & Helbing): g = (λ·tc + queue) / (Q_max − λ)
            queue = tracker['queue']
            denom = Q_max - arr_rate
            if denom > 0:
                green_time = (arr_rate * clearing_time + queue) / denom
                green_time = max(0.0, green_time - (green_time % step))
            else:
                green_time = 0.0

            stats['green_time'] = green_time
            stats['arr_rate'] = arr_rate
            stats['ave_arr_rate'] = ave_arr_rate

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

    def update(self, det_data):
        self.update_arr_dep_veh_num(det_data)
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

    def __init__(self, id, intersection: AnalyticPlusAgent, in_lane_length: float, out_lane_length: float,
                 max_speed: float):
        self.ID = id
        # sumo does not group movements, but in principle this can be a set of lane pairs
        self.in_lanes = set()
        self.out_lanes = set()
        self.sumo_movement_idxs = set()
        self.in_length = in_lane_length
        self.out_length = out_lane_length
        self._intersection = intersection
        self.traci = intersection.env.traci
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
        self.last_on_time = 0
        self.priority: float = 0
        self.green_time: float = 0

    def update_last_on(self, action: sumoPhase, phase: sumoPhase):
        if self.ID not in action['movement_ids'] and self.ID in phase['movement_ids']:  # switch to new phase
            self.last_on_time = self._intersection.env.time
        elif self.ID in action['movement_ids'] and self.ID not in phase['movement_ids']:  # will be deactivated
            self.last_on_time = -1  # currently active


