from .DQN_strategy import DQNStrategy
from .rl_networks import *

class TransformerDQNStrategy(DQNStrategy):
    """
    Transformer-based variant of Intellilight/DQN.

    Reuses Intellilight control flow and replay mechanics while replacing the
    Q-network with a shared Transformer communication module over all intersections.
    """

    def __init__(
        self,
        *args,
        sequence_length=8,
        transformer_heads=4,
        transformer_layers=2,
        transformer_dropout=0.1,
        transformer_ff_multiplier=4,
        transformer_beta=2,
        learning_start=1000,
        epsilon_min=0.01,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.__name__ = "TransformerDQNStrategy"

        self.sequence_length = self._to_tls_dict(sequence_length, int)
        self.transformer_heads = self._to_tls_dict(transformer_heads, int)
        self.transformer_layers = self._to_tls_dict(transformer_layers, int)
        self.transformer_dropout = self._to_tls_dict(transformer_dropout, float)
        self.transformer_ff_multiplier = self._to_tls_dict(transformer_ff_multiplier, int)
        self.transformer_beta = int(transformer_beta)
        self.learning_start = self._to_tls_dict(learning_start, int)
        self.epsilon_min = self._to_tls_dict(epsilon_min, float)
        self.connectivity_buckets = 4

        self.tls_ids = list(self.network.TLS_DETECTORS.keys())
        self.tls_index = {tl_id: idx for idx, tl_id in enumerate(self.tls_ids)}

        self.state_dims = {}
        self.global_state_dim = None
        self.relation_index = None

        self._joint_cache_time = None
        self._joint_cache_state = None
        self._joint_cache_actions = {}

        self.last_global_state = {tl_id: None for tl_id in self.network.TLS_DETECTORS}
        self._padding_buffer = None

    def _to_tls_dict(self, value, cast_fn):
        if isinstance(value, dict):
            return {tl_id: cast_fn(value[tl_id]) for tl_id in self.network.TLS_DETECTORS}
        return {tl_id: cast_fn(value) for tl_id in self.network.TLS_DETECTORS}

    def _estimate_axis_scale(self, values):
        # A degenerate axis (all TLS share one coordinate, e.g. a collinear line
        # network on the y axis) has no spread to infer a grid step from. That is
        # fine: every delta on that axis is 0, so 0 / scale == 0 regardless of the
        # scale value. Return a harmless unit scale instead of raising, so line
        # networks work while grid networks still get a real per-axis scale.
        uniq = sorted({round(float(v), 6) for v in values})
        if len(uniq) < 2:
            return 1.0
        diffs = [abs(uniq[i + 1] - uniq[i]) for i in range(len(uniq) - 1)]
        diffs = [d for d in diffs if d > 1e-6]
        if not diffs:
            return 1.0
        return float(np.median(diffs))

    def _resolve_network_path(self):
        file_names = getattr(self.network, "file_names", None)
        if isinstance(file_names, dict):
            candidate = file_names.get("network")
            if candidate:
                return os.fspath(candidate)

        candidate = getattr(self.network, "NET_FILE", None)
        if candidate:
            return os.fspath(candidate)

        raise ValueError(
            "Transformer paper implementation requires a SUMO network path "
            "(network.file_names['network'] or network.NET_FILE)."
        )

    def _build_relative_position_index(self):
        n = len(self.tls_ids)
        beta = self.transformer_beta
        span = 2 * beta + 1

        network_path = self._resolve_network_path()
        if not os.path.exists(network_path):
            raise FileNotFoundError(f"SUMO network file not found: {network_path}")

        coords = {}
        adjacency = np.zeros((n, n), dtype=np.int64)
        root = ET.parse(network_path).getroot()

        junction_coords = {}
        for junction in root.findall("junction"):
            junction_id = junction.attrib.get("id")
            if not junction_id:
                continue
            if "x" not in junction.attrib or "y" not in junction.attrib:
                continue
            junction_coords[junction_id] = (float(junction.attrib["x"]), float(junction.attrib["y"]))

        edge_endpoints = {}
        for edge in root.findall("edge"):
            edge_id = edge.attrib.get("id")
            src = edge.attrib.get("from")
            dst = edge.attrib.get("to")
            if edge_id and src and dst:
                edge_endpoints[edge_id] = (src, dst)

        # Build TLS -> controlled junctions map from connection tl attributes.
        tls_to_junctions = {tl_id: set() for tl_id in self.tls_ids}
        for tl_id in self.tls_ids:
            if tl_id in junction_coords:
                tls_to_junctions[tl_id].add(tl_id)

        for connection in root.findall("connection"):
            tl_id = connection.attrib.get("tl")
            if tl_id not in tls_to_junctions:
                continue
            from_edge = connection.attrib.get("from")
            to_edge = connection.attrib.get("to")
            for edge_id in (from_edge, to_edge):
                endpoints = edge_endpoints.get(edge_id)
                if endpoints is None:
                    continue
                src, dst = endpoints
                if src in junction_coords:
                    tls_to_junctions[tl_id].add(src)
                if dst in junction_coords:
                    tls_to_junctions[tl_id].add(dst)

        missing_coords = []
        for tl_id in self.tls_ids:
            candidate_junctions = [j for j in tls_to_junctions[tl_id] if j in junction_coords]
            if not candidate_junctions:
                missing_coords.append(tl_id)
                continue

            if tl_id in junction_coords:
                coords[tl_id] = junction_coords[tl_id]
            else:
                points = np.asarray([junction_coords[j] for j in sorted(candidate_junctions)], dtype=np.float32)
                coords[tl_id] = (float(np.mean(points[:, 0])), float(np.mean(points[:, 1])))

        if missing_coords:
            raise ValueError(
                "Missing junction coordinates for TLS ids: " + ", ".join(missing_coords)
            )

        junction_to_tls = {}
        for tl_id, junctions in tls_to_junctions.items():
            for junction_id in junctions:
                if junction_id not in junction_coords:
                    continue
                junction_to_tls.setdefault(junction_id, set()).add(tl_id)

        for src, dst in edge_endpoints.values():
            src_tls = junction_to_tls.get(src)
            dst_tls = junction_to_tls.get(dst)
            if not src_tls or not dst_tls:
                continue
            for src_tl in src_tls:
                for dst_tl in dst_tls:
                    if src_tl == dst_tl:
                        continue
                    adjacency[self.tls_index[src_tl], self.tls_index[dst_tl]] = 1

        xs = [coords[tl_id][0] for tl_id in self.tls_ids]
        ys = [coords[tl_id][1] for tl_id in self.tls_ids]
        scale_x = self._estimate_axis_scale(xs)
        scale_y = self._estimate_axis_scale(ys)

        relation_index = np.zeros((n, n), dtype=np.int64)
        for i, src_tl in enumerate(self.tls_ids):
            sx, sy = coords[src_tl]
            for j, dst_tl in enumerate(self.tls_ids):
                dx = (coords[dst_tl][0] - sx) / scale_x
                dy = (coords[dst_tl][1] - sy) / scale_y

                r_x = int(np.clip(np.round(dx), -beta, beta))
                r_y = int(np.clip(np.round(dy), -beta, beta))
                distance_idx = (r_x + beta) * span + (r_y + beta)

                if i == j:
                    conn_idx = 0
                elif adjacency[j, i] and adjacency[i, j]:
                    conn_idx = 3
                elif adjacency[j, i]:
                    conn_idx = 1
                elif adjacency[i, j]:
                    conn_idx = 2
                else:
                    conn_idx = 3

                relation_index[i, j] = distance_idx * self.connectivity_buckets + conn_idx

        relation_bucket_count = span * span * self.connectivity_buckets
        return torch.tensor(relation_index, dtype=torch.long, device=self.device), relation_bucket_count

    def _validate_and_cast_state(self, state, tl_id, out_row):
        arr = np.asarray(state, dtype=np.float32).reshape(-1)
        expected_dim = self.state_dims[tl_id]
        if len(arr) != expected_dim:
            raise ValueError(
                f"State size mismatch for TLS '{tl_id}': expected {expected_dim}, got {len(arr)}"
            )
        
        # Directly slice out target memory space from the pre-allocated batch view
        out_row[:len(arr)] = arr

    def _collect_global_state(self):
        self._padding_buffer.fill(0.0)
        for idx, tl_id in enumerate(self.tls_ids):
            state = self.get_state(tl_id)
            self._validate_and_cast_state(state, tl_id, self._padding_buffer[idx])
        return self._padding_buffer

    def _compute_joint_actions(self, train=True):
        sim_time = int(self.traci.simulation.getTime())
        if self._joint_cache_time == sim_time and self._joint_cache_state is not None:
            return

        global_state = self._collect_global_state()
        state_tensor = torch.as_tensor(global_state, dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.no_grad():
            q_values = self.model[self.tls_ids[0]](state_tensor, self.relation_index).squeeze(0)

        actions = {}
        for idx, tl_id in enumerate(self.tls_ids):
            epsilon = max(float(self.exploration_prob[tl_id]), float(self.epsilon_min[tl_id]))
            if train and random.random() < epsilon:
                action = random.choice([0, 1])
            else:
                action = int(torch.argmax(q_values[idx]).item())
            actions[tl_id] = action

            if train:
                decayed = epsilon * (1.0 - float(self.cooling_rate[tl_id]))
                self.exploration_prob[tl_id] = max(float(self.epsilon_min[tl_id]), decayed)

        self._joint_cache_time = sim_time
        self._joint_cache_state = global_state.copy()  # Clone state array safely
        self._joint_cache_actions = actions

    def _start_agent(self, tl_id):
        self.nb_phases[tl_id] = len(self.traci.trafficlight.getAllProgramLogics(tl_id)[0].phases)
        tl_logic = self.traci.trafficlight.getAllProgramLogics(tl_id)[0]
        for phase in tl_logic.phases:
            phase.duration = 10000
            phase.maxDur = 10000
            phase.minDur = 10000

        self.traci.trafficlight.setProgramLogic(tl_id, tl_logic)
        self.traci.trafficlight.setPhase(tl_id, 0)
        self.traci.trafficlight.setPhaseDuration(tl_id, 10000)
        self.started = True

        if self.global_state_dim is None:
            self.state_dims = {tls_id: len(self.get_state(tls_id)) for tls_id in self.tls_ids}
            self.global_state_dim = int(max(self.state_dims.values()))
            self._padding_buffer = np.zeros((len(self.tls_ids), self.global_state_dim), dtype=np.float32)

        if self.relation_index is None:
            self.relation_index, relation_bucket_count = self._build_relative_position_index()
            self.relation_bucket_count = int(relation_bucket_count)

        if self.model[tl_id] is None:
            shared_model = TCMQNetwork(
                input_dim=self.global_state_dim,
                hidden_dim=self.hidden_layer_size[tl_id],
                output_dim=2,
                nhead=self.transformer_heads[tl_id],
                num_layers=self.transformer_layers[tl_id],
                dropout=self.transformer_dropout[tl_id],
                ff_multiplier=self.transformer_ff_multiplier[tl_id],
                relation_bucket_count=self.relation_bucket_count,
            ).to(self.device)
            shared_target = copy.deepcopy(shared_model).to(self.device)
            shared_optimizer = optim.Adam(shared_model.parameters(), lr=self.learning_rate[tl_id])

            for tls_id in self.tls_ids:
                self.model[tls_id] = shared_model
                self.target_model[tls_id] = shared_target
                self.optimizer[tls_id] = shared_optimizer

        self.last_global_state[tl_id] = None
        self._joint_cache_time = None
        self._joint_cache_state = None
        self._joint_cache_actions = {}

    def get_next_action(self, tl_id, train=True):
        self._compute_joint_actions(train=train)

        global_state = self._joint_cache_state
        action = int(self._joint_cache_actions[tl_id])

        reward = self.get_reward(tl_id, change_phase=self.last_action[tl_id])
        done = (self.traci.simulation.getTime() % self.episode_duration[tl_id] == 0)
        done = done or bool(getattr(self.traci, '_sumo_experiments_episode_reset', False))

        if self.last_global_state[tl_id] is not None and self.last_action[tl_id] is not None:
            self.replay_buffer[tl_id].append(
                (
                    self.last_global_state[tl_id],
                    self.last_action[tl_id],
                    reward,
                    global_state,
                    self.tls_index[tl_id],
                    done,
                )
            )
            if tl_id == self.network.TL_IDS[0]:
                self.rewards.append(reward)
                self.times.append(self.traci.simulation.getTime())
            self.scores = [
                self.get_score(tls_id, self.last_action[tls_id])
                for tls_id in self.network.TLS_DETECTORS
                if self.last_action[tls_id] is not None
            ]

        if done:
            self.last_state[tl_id] = None
            self.last_global_state[tl_id] = None
            self.last_action[tl_id] = None
        else:
            self.last_state[tl_id] = global_state[self.tls_index[tl_id]]
            self.last_global_state[tl_id] = global_state.copy()
            self.last_action[tl_id] = action
        return action

    def _sample_global_memory(self):
        non_empty_buffers = []
        sizes = []
        for tls_id in self.tls_ids:
            buf = self.replay_buffer[tls_id]
            buf_len = len(buf)
            if buf_len > 0:
                non_empty_buffers.append(buf)
                sizes.append(buf_len)

        if not non_empty_buffers:
            return []

        total_samples = sum(sizes)
        max_batch = self.batch_size[self.tls_ids[0]]
        batch_size = min(total_samples, max_batch)

        probs = np.asarray(sizes, dtype=np.float64) / total_samples
        counts = np.random.multinomial(batch_size, probs)

        batch = []
        for buf, count in zip(non_empty_buffers, counts):
            if count <= 0:
                continue
            sampled_elements = random.sample(buf, count)
            batch.extend(sampled_elements)

        random.shuffle(batch)
        return batch

    def train(self, tl_id):
        total_samples = sum(len(self.replay_buffer[tls_id]) for tls_id in self.tls_ids)
        if total_samples < self.learning_start[tl_id]:
            return

        if self.shared_network:
            model_id = id(self.model[tl_id])
            if model_id in self._trained_this_step:
                return
            self._trained_this_step.add(model_id)

        batch = self._sample_global_memory()
        if not batch:
            return

        self.model[tl_id].train()
        self.target_model[tl_id].eval()

        states, actions, rewards, next_states, agent_idx, dones = zip(*batch)

        states = torch.as_tensor(np.asarray(states, dtype=np.float32), device=self.device)
        next_states = torch.as_tensor(np.asarray(next_states, dtype=np.float32), device=self.device)
        actions = torch.as_tensor(np.asarray(actions, dtype=np.int64), device=self.device).unsqueeze(1)
        rewards = torch.as_tensor(np.asarray(rewards, dtype=np.float32), device=self.device).unsqueeze(1)
        agent_idx = torch.as_tensor(np.asarray(agent_idx, dtype=np.int64), device=self.device)
        dones = torch.as_tensor(np.asarray(dones, dtype=np.float32), device=self.device).unsqueeze(1)
        batch_idx = torch.arange(next_states.size(0), device=self.device)

        with torch.no_grad():
            next_q_values = self.target_model[tl_id](next_states, self.relation_index)
            next_agent_q = next_q_values[batch_idx, agent_idx]
            max_next_q_values = next_agent_q.max(dim=1, keepdim=True)[0]
            targets = rewards + self.gamma[tl_id] * max_next_q_values * (1 - dones)

        current_q_values = self.model[tl_id](states, self.relation_index)
        current_agent_q = current_q_values[batch_idx, agent_idx]
        current_q_values = current_agent_q.gather(1, actions)
        loss = nn.functional.smooth_l1_loss(current_q_values, targets)

        self.optimizer[tl_id].zero_grad()
        loss.backward()
        self.optimizer[tl_id].step()

        self.loss_history[tl_id].append(loss.item())
        self.number_of_trainings[tl_id] += 1
        if self.number_of_trainings[tl_id] == self.update_target_frequency[tl_id]:
            self.number_of_trainings[tl_id] = 0
            self.update_target_model(tl_id)

    def load_model(self, filepath):
        torch.serialization.add_safe_globals([
            TCMQNetwork,
            RelativePositionCommunicationLayer,
            nn.Linear,
            nn.LayerNorm,
            nn.ReLU,
            nn.GELU,
            nn.Sequential,
            nn.ModuleList,
            nn.Embedding,
            nn.Dropout,
            optim.Adam,
            dict,
        ])
        super().load_model(filepath)

        first_model = next((self.model[tls_id] for tls_id in self.tls_ids if self.model[tls_id] is not None), None)
        first_target = next((self.target_model[tls_id] for tls_id in self.tls_ids if self.target_model[tls_id] is not None), None)
        if first_model is not None and first_target is not None:
            shared_optimizer = optim.Adam(first_model.parameters(), lr=self.learning_rate[self.tls_ids[0]])
            for tls_id in self.tls_ids:
                self.model[tls_id] = first_model
                self.target_model[tls_id] = first_target
                self.optimizer[tls_id] = shared_optimizer
