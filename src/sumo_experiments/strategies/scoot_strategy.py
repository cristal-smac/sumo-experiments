from .rl_util import *
from . import Strategy
import copy
import numpy as np
import math

class ScootScatsStrategy(Strategy):
    """SCOOT/SCATS-style adaptive controller for SUMO experiments."""

    DEFAULT_PARAMS = {
        'adaptation_cycle': 30,
        'adaptation_green': 10,
        'green_thresh': 2,
        'adaptation_offset': 1,
        'offset_thresh': 0.5,
        'min_cycle_length': 50,
        'max_cycle_length': 180,
        'ds_upper_val': 0.925,
        'ds_lower_val': 0.875,
        'measurement_period': 1,
        'yellow_time': 3,
        'min_green': 3,
        'cycle_update_interval': 5,
        'speed_limit_mps': 13.9,
    }

    def __init__(
        self,
        network,
        params=None,
        initial_cycle_length=120,
        yellow_time=3,
        intelligent_intersections=None,
    ):
        super().__init__()
        self.started = False
        self.network = network

        if intelligent_intersections is None:
            self.intelligent_intersections = list(network.TL_IDS)
        else:
            self.intelligent_intersections = list(intelligent_intersections)

        self.params = dict(self.DEFAULT_PARAMS)
        if params:
            self.params.update(params)

        if isinstance(yellow_time, dict):
            self.yellow_time = yellow_time
        else:
            self.yellow_time = {tl_id: int(yellow_time) for tl_id in self.intelligent_intersections}
        self.params['yellow_time'] = int(self.params['yellow_time'])
        self.params['min_green'] = int(self.params['min_green'])
        self.initial_cycle_length = int(initial_cycle_length)

        self.measurement_data = {
            'measurement_counter': -1,
            'control_counter': 0,
            'update_counter': 0,
            'last_cycle_update': 0,
            'cycle_length': int(initial_cycle_length),
            'previous_cycle_length': {tl_id: int(initial_cycle_length) for tl_id in self.intelligent_intersections},
            'greentimes': {},
            'offsets': {tl_id: 0 for tl_id in self.intelligent_intersections},
            'estimated_travel_time': {},
            'history_cycle_lengths': [],
            'history_greentimes': [],
            'history_offsets': [],
        }

        self._phase_order = {}
        self._phase_detectors = {}
        self._det_to_phase_indices = {}
        self._green_states = {}
        self._yellow_states = {}
        self._detector_lane_lengths = {}
        self._all_numerical_detectors = set()

    @staticmethod
    def _is_yellow_or_all_red(state):
        lowered = state.lower()
        return ('y' in lowered) or (set(lowered) == {'r'})

    def _extract_yellow_state(self, phases, phase_id, green_state):
        nb_phases = len(phases)
        for offset in range(1, nb_phases):
            idx = (phase_id + offset) % nb_phases
            candidate = phases[idx].state
            if 'y' in candidate.lower():
                return candidate
        return ''.join('y' if ch in ('g', 'G') else 'r' for ch in green_state)

    def _normalize_phase_greens(self, greens, target_sum):
        if not greens:
            return []

        min_green = int(self.params['min_green'])
        target = max(int(target_sum), len(greens) * min_green)
        normalized = [max(min_green, int(round(g))) for g in greens]
        total = sum(normalized)

        if total < target:
            idx = 0
            while total < target:
                normalized[idx % len(normalized)] += 1
                total += 1
                idx += 1
        elif total > target:
            order = np.argsort(normalized)[::-1]
            excess = total - target
            for idx in order:
                if excess <= 0:
                    break
                removable = max(0, normalized[idx] - min_green)
                delta = min(excess, removable)
                normalized[idx] -= delta
                excess -= delta
            if excess > 0:
                idx = 0
                while excess > 0:
                    if normalized[idx % len(normalized)] > min_green:
                        normalized[idx % len(normalized)] -= 1
                        excess -= 1
                    idx += 1

        return normalized

    def _discover_tls_structure(self, tl_id):
        tls_detectors = self.network.TLS_DETECTORS[tl_id]
        logic = self.traci.trafficlight.getAllProgramLogics(tl_id)[-1]
        phases = logic.phases

        phase_order = sorted(list(tls_detectors.keys()))
        if not phase_order:
            phase_order = [
                idx for idx, phase in enumerate(phases)
                if not self._is_yellow_or_all_red(phase.state)
            ]

        if not phase_order:
            phase_order = [0]

        phase_detectors = {}
        det_to_phase_indices = {}
        green_states = []
        yellow_states = []

        for local_idx, phase_id in enumerate(phase_order):
            detector_groups = tls_detectors[phase_id]
            numerical = list(dict.fromkeys(detector_groups['numerical']))
            phase_detectors[local_idx] = numerical
            for det in numerical:
                det_to_phase_indices.setdefault(det, set()).add(local_idx)

            if phase_id < len(phases):
                green_state = phases[phase_id].state
                yellow_state = self._extract_yellow_state(phases, phase_id, green_state)
            else:
                green_state = phases[0].state
                yellow_state = ''.join('y' if ch in ('g', 'G') else 'r' for ch in green_state)
            green_states.append(green_state)
            yellow_states.append(yellow_state)

        return phase_order, phase_detectors, det_to_phase_indices, green_states, yellow_states

    def _get_detector_lane_length(self, detector_id):
        cached = self._detector_lane_lengths.get(detector_id)
        if cached is not None:
            return cached
        try:
            lane_id = self.traci.lanearea.getLaneID(detector_id)
            lane_length = float(self.traci.lane.getLength(lane_id))
        except Exception:
            lane_length = 10.0
        self._detector_lane_lengths[detector_id] = max(1.0, lane_length)
        return self._detector_lane_lengths[detector_id]

    def _estimate_intersection_travel_time(self, tl_id):
        detectors = set()
        for phase_detectors in self._phase_detectors[tl_id].values():
            detectors.update(phase_detectors)

        lengths = [self._get_detector_lane_length(det) for det in detectors]
        if not lengths:
            return 0.0

        speed_limit = max(float(self.params['speed_limit_mps']), 0.1)
        return float(np.mean(lengths)) / speed_limit

    def _subscribe_detectors(self):
        import traci.constants as tc

        for det in sorted(self._all_numerical_detectors):
            try:
                self.traci.lanearea.subscribe(det, [tc.LAST_STEP_VEHICLE_NUMBER])
            except Exception:
                # Some detector IDs may be invalid for a given scenario; skip safely.
                continue

    def _start_agents(self):
        for tl_id in self.intelligent_intersections:
            (
                phase_order,
                phase_detectors,
                det_to_phase_indices,
                green_states,
                yellow_states,
            ) = self._discover_tls_structure(tl_id)

            self._phase_order[tl_id] = phase_order
            self._phase_detectors[tl_id] = phase_detectors
            self._det_to_phase_indices[tl_id] = det_to_phase_indices
            self._green_states[tl_id] = green_states
            self._yellow_states[tl_id] = yellow_states

            for detector_ids in phase_detectors.values():
                self._all_numerical_detectors.update(detector_ids)

            logic = self.traci.trafficlight.getAllProgramLogics(tl_id)[-1]
            initial = []
            for phase_id in phase_order:
                if phase_id < len(logic.phases):
                    initial.append(int(max(1, round(logic.phases[phase_id].duration))))
                else:
                    initial.append(int(self.params['min_green']))

            cycle_effective = self.initial_cycle_length - len(initial) * int(self.yellow_time[tl_id])
            cycle_effective = max(cycle_effective, len(initial) * int(self.params['min_green']))
            normalized_initial = self._normalize_phase_greens(initial, cycle_effective)

            self.measurement_data['greentimes'][tl_id] = normalized_initial
            self.measurement_data['previous_cycle_length'][tl_id] = int(sum(normalized_initial))

        self._subscribe_detectors()
        self._apply_tl_programme(
            self.measurement_data['greentimes'],
            self.measurement_data['offsets'],
        )
        self.started = True

    def _measure_queues_and_ds(self):
        import traci.constants as tc

        det_data = self.traci.lanearea.getAllSubscriptionResults()
        queue_lengths = {}
        degree_of_sat = {}

        for tl_id in self.intelligent_intersections:
            queue_lengths[tl_id] = {}
            degree_of_sat[tl_id] = {}

            tl_detectors = set()
            for detector_ids in self._phase_detectors[tl_id].values():
                tl_detectors.update(detector_ids)

            for det in tl_detectors:
                if det in det_data and det_data[det] is not None:
                    queue_val = det_data[det][tc.LAST_STEP_VEHICLE_NUMBER]
                else:
                    try:
                        queue_val = self.traci.lanearea.getLastStepVehicleNumber(det)
                    except Exception:
                        queue_val = 0

                queue = int(max(0, queue_val))
                lane_length = self._get_detector_lane_length(det)
                ds = min(1.0, float(queue) / max(1.0, lane_length / 7.0))

                queue_lengths[tl_id][det] = queue
                degree_of_sat[tl_id][det] = ds

        return queue_lengths, degree_of_sat

    def optimize_cycle_length(self, degree_of_sat):
        max_ds_values = [
            max(det_vals.values())
            for det_vals in degree_of_sat.values()
            if det_vals
        ]
        if not max_ds_values:
            return

        max_ds_in_network = max(max_ds_values)
        if max_ds_in_network >= self.params['ds_upper_val']:
            new_length = (
                self.measurement_data['cycle_length']
                + (max_ds_in_network - self.params['ds_upper_val']) * self.params['adaptation_cycle']
            )
            self.measurement_data['cycle_length'] = min(
                int(math.ceil(new_length)),
                int(self.params['max_cycle_length']),
            )
        elif 0 < max_ds_in_network < self.params['ds_lower_val']:
            new_length = (
                self.measurement_data['cycle_length']
                - (self.params['ds_lower_val'] - max_ds_in_network) * self.params['adaptation_cycle']
            )
            self.measurement_data['cycle_length'] = max(
                int(math.floor(new_length)),
                int(self.params['min_cycle_length']),
            )

    def optimize_green_phases(self, queue_lengths, degree_of_sat):
        min_green = int(self.params['min_green'])

        for tl_id, greens in list(self.measurement_data['greentimes'].items()):
            if not greens:
                continue

            yellow_time = int(self.yellow_time[tl_id])
            effective_cycle = self.measurement_data['cycle_length'] - yellow_time * len(greens)
            effective_cycle = max(effective_cycle, len(greens) * min_green)

            det_sat = degree_of_sat[tl_id]
            det_queue = queue_lengths[tl_id]
            if not det_sat:
                prev_cycle = max(1, self.measurement_data['previous_cycle_length'][tl_id])
                scaled = [int(g * effective_cycle / prev_cycle) for g in greens]
                diff = effective_cycle - sum(scaled)
                scaled[0] += diff
                self.measurement_data['greentimes'][tl_id] = self._normalize_phase_greens(scaled, effective_cycle)
                self.measurement_data['previous_cycle_length'][tl_id] = int(
                    sum(self.measurement_data['greentimes'][tl_id])
                )
                continue

            max_det = max(det_sat, key=det_sat.get)
            max_queue = det_queue[max_det]
            max_ds = det_sat[max_det]

            if max_queue > self.params['green_thresh']:
                phases_with_max_lane = sorted(self._det_to_phase_indices[tl_id][max_det])
                if len(phases_with_max_lane) == 1:
                    max_phase = phases_with_max_lane[0]
                else:
                    max_phase = 0

                excluded_detectors = set()
                for phase_idx in phases_with_max_lane:
                    excluded_detectors.update(self._phase_detectors[tl_id][phase_idx])

                phase_ds_list = []
                for phase_idx in range(len(greens)):
                    if phase_idx == max_phase:
                        continue
                    detectors = [
                        det
                        for det in self._phase_detectors[tl_id][phase_idx]
                        if det not in excluded_detectors
                    ]
                    if detectors:
                        ds_val = max(det_sat[det] for det in detectors)
                    else:
                        ds_val = 0.0
                    phase_ds_list.append((phase_idx, ds_val))

                if phase_ds_list:
                    phase_ds_sorted = sorted(phase_ds_list, key=lambda x: x[1], reverse=True)
                    second_phase = phase_ds_sorted[0][0]
                    second_ds = phase_ds_sorted[0][1]
                    third_phase = phase_ds_sorted[1][0] if len(phase_ds_sorted) > 1 else None
                    third_ds = phase_ds_sorted[1][1] if len(phase_ds_sorted) > 1 else second_ds

                    ds_diff_max = max(0.0, max_ds - third_ds)
                    ds_diff_second = max(0.0, second_ds - third_ds)

                    greens[max_phase] = int(min(
                        3 * effective_cycle / 4,
                        greens[max_phase] + ds_diff_max * self.params['adaptation_green'],
                    ))
                    remaining_time = effective_cycle - greens[max_phase]

                    if len(greens) == 2:
                        greens[second_phase] = int(remaining_time)
                    elif len(greens) == 3 and third_phase is not None:
                        greens[second_phase] = int(min(
                            2 * remaining_time / 3,
                            greens[second_phase] + ds_diff_second * self.params['adaptation_green'],
                        ))
                        greens[third_phase] = int(remaining_time - greens[second_phase])
                    else:
                        other_phases = [idx for idx in range(len(greens)) if idx != max_phase]
                        weights = np.asarray([
                            max(1e-3, next((v for p, v in phase_ds_list if p == idx), 1e-3))
                            for idx in other_phases
                        ])
                        weights = weights / np.sum(weights)
                        reassigned = [int(round(remaining_time * w)) for w in weights]
                        if reassigned:
                            reassigned[0] += int(remaining_time - sum(reassigned))
                            for idx, value in zip(other_phases, reassigned):
                                greens[idx] = value
            else:
                previous_cycle = max(1, self.measurement_data['previous_cycle_length'][tl_id])
                scaled = [int(g * effective_cycle / previous_cycle) for g in greens]
                diff = effective_cycle - sum(scaled)
                scaled[0] += diff
                greens = scaled

            greens = self._normalize_phase_greens(greens, effective_cycle)
            self.measurement_data['greentimes'][tl_id] = greens
            self.measurement_data['previous_cycle_length'][tl_id] = int(sum(greens))

    def optimize_offsets(self, queue_lengths):
        intersections = list(self.intelligent_intersections)
        if len(intersections) < 2:
            return

        congestion = {}
        for tl_id in intersections:
            queues = queue_lengths[tl_id]
            if not queues:
                congestion[tl_id] = 0.0
                continue
            total_queue = float(sum(queues.values()))
            total_length = float(sum(self._get_detector_lane_length(det) for det in queues.keys()))
            congestion[tl_id] = (total_queue / max(total_length, 1.0)) * 10.0

        ranked = sorted(congestion.items(), key=lambda kv: kv[1], reverse=True)
        if len(ranked) < 2:
            return

        congestion_gap = abs(ranked[0][1] - ranked[1][1])
        if congestion_gap > self.params['offset_thresh']:
            ordered_intersections = [tl_id for tl_id, _ in ranked]
            offsets = {}
            previous_tl = None
            for tl_id in ordered_intersections:
                if previous_tl is None:
                    offsets[tl_id] = 0.0
                else:
                    offsets[tl_id] = min(
                        offsets[previous_tl]
                        + self._estimate_intersection_travel_time(previous_tl) * self.params['adaptation_offset'],
                        self.measurement_data['cycle_length'],
                    )
                previous_tl = tl_id
            for tl_id in intersections:
                offsets.setdefault(tl_id, 0.0)
            self.measurement_data['offsets'] = offsets
        elif congestion_gap < self.params['offset_thresh'] - 0.1:
            self.measurement_data['offsets'] = {tl_id: 0.0 for tl_id in intersections}

    def _apply_phase_offset(self, tl_id, greens, shift_seconds):
        if not greens:
            return
        phase_durations = []
        yellow_time = int(self.yellow_time[tl_id])
        for green in greens:
            phase_durations.extend([int(max(1, green)), max(1, yellow_time)])

        cycle_duration = int(sum(phase_durations))
        if cycle_duration <= 0:
            self.traci.trafficlight.setPhase(tl_id, 0)
            return

        shift = int(shift_seconds) % cycle_duration
        cumulative = 0
        target_phase_idx = 0
        remaining = phase_durations[0]

        for idx, duration in enumerate(phase_durations):
            if shift < cumulative + duration:
                target_phase_idx = idx
                remaining = duration - (shift - cumulative)
                break
            cumulative += duration

        self.traci.trafficlight.setPhase(tl_id, int(target_phase_idx))
        self.traci.trafficlight.setPhaseDuration(tl_id, max(1, int(remaining)))

    def _apply_tl_programme(self, greentimes, offsets):
        import traci.constants as tc

        tl_type_static = getattr(tc, 'TRAFFICLIGHT_TYPE_STATIC', 0)

        for tl_id, greens in greentimes.items():
            if not greens:
                continue

            phases = []
            yellow_time = int(self.yellow_time[tl_id])
            for idx, green in enumerate(greens):
                green_state = self._green_states[tl_id][idx]
                yellow_state = self._yellow_states[tl_id][idx]
                phases.append(self.traci.trafficlight.Phase(int(max(1, green)), green_state))
                phases.append(self.traci.trafficlight.Phase(max(1, yellow_time), yellow_state))

            logic = self.traci.trafficlight.Logic(
                programID=f'program_scootscats_{tl_id}',
                type=tl_type_static,
                currentPhaseIndex=0,
                phases=phases,
            )
            self.traci.trafficlight.setProgramLogic(tl_id, logic)
            self._apply_phase_offset(tl_id, greens, offsets[tl_id])

    def _get_sim_time(self):
        if hasattr(self.traci.simulation, 'getCurrentTime'):
            return self.traci.simulation.getCurrentTime()
        return int(float(self.traci.simulation.getTime()) * 1000.0)

    def run_all_agents(self, traci):
        if not self.started:
            self.traci = traci
            self._start_agents()
            return True

        self.zeus_monitor.begin_window('all_agents')
        try:
            self.measurement_data['measurement_counter'] += 1
            if self.measurement_data['measurement_counter'] == int(self.params['measurement_period']):
                self.measurement_data['measurement_counter'] = 0
                self.measurement_data['control_counter'] += 1

                if self.measurement_data['control_counter'] >= (
                    self.measurement_data['last_cycle_update'] + self.measurement_data['cycle_length']
                ):
                    self.measurement_data['last_cycle_update'] = self.measurement_data['control_counter']

                    queue_lengths, degree_of_sat = self._measure_queues_and_ds()
                    cycle_interval = int(self.params['cycle_update_interval'])

                    if (
                        self.measurement_data['update_counter'] % cycle_interval == 0
                        and self.measurement_data['update_counter'] != 0
                    ):
                        self.optimize_cycle_length(degree_of_sat)

                    if self.measurement_data['update_counter'] != 0:
                        self.optimize_green_phases(queue_lengths, degree_of_sat)

                    if (
                        self.measurement_data['update_counter'] % cycle_interval == 0
                        and self.measurement_data['update_counter'] != 0
                    ):
                        self.optimize_offsets(queue_lengths)

                    self.measurement_data['update_counter'] += 1
                    self._apply_tl_programme(
                        self.measurement_data['greentimes'],
                        self.measurement_data['offsets'],
                    )

                    current_time = self._get_sim_time()
                    self.measurement_data['history_greentimes'].append([
                        current_time,
                        copy.deepcopy(self.measurement_data['greentimes']),
                    ])
                    self.measurement_data['history_offsets'].append([
                        current_time,
                        copy.deepcopy(self.measurement_data['offsets']),
                    ])
                    self.measurement_data['history_cycle_lengths'].append([
                        current_time,
                        int(self.measurement_data['cycle_length']),
                    ])
        finally:
            results = self.zeus_monitor.end_window('all_agents')
            self.energy_consumption += self.get_energy_consumption(results)
