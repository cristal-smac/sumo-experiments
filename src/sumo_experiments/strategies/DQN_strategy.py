from .rl_util import episode_reward_scale
from . import IntellilightStrategy, MaxPressureStrategy
import copy
import torch
import numpy as np

class DQNStrategy(IntellilightStrategy):

    REWARD_REFERENCE_EPISODE_DURATION = 1000.0

    def _remap_loaded_models(self, loaded_models):
        """Remap loaded model keys to current network TLS ids when they differ."""
        expected_ids = list(self.network.TLS_DETECTORS.keys())
        remapped = {tl_id: None for tl_id in expected_ids}

        # Backward compatibility: some checkpoints may store a single module.
        if not isinstance(loaded_models, dict):
            for tl_id in expected_ids:
                remapped[tl_id] = copy.deepcopy(loaded_models)
            return remapped

        overlap_ids = [tl_id for tl_id in expected_ids if tl_id in loaded_models]
        if overlap_ids:
            for tl_id in overlap_ids:
                remapped[tl_id] = loaded_models[tl_id]

            missing = [tl_id for tl_id in expected_ids if tl_id not in loaded_models]
            if missing:
                print(
                    f"Warning: missing model keys for {len(missing)} intersections; "
                    "those agents will start from random initialization."
                )
            return remapped

        # No exact key overlap: try best-effort remapping by cardinality.
        if len(loaded_models) == 1:
            template = next(iter(loaded_models.values()))
            for tl_id in expected_ids:
                remapped[tl_id] = copy.deepcopy(template)
            print("Warning: checkpoint had a single model; duplicated to all intersections.")
            return remapped

        if len(loaded_models) == len(expected_ids):
            for target_id, source_id in zip(expected_ids, sorted(loaded_models.keys())):
                remapped[target_id] = loaded_models[source_id]
            print("Warning: checkpoint keys did not match TLS ids; remapped by sorted order.")
            return remapped

        print(
            "Warning: checkpoint keys do not match network TLS ids and cannot be safely remapped; "
            "agents will use random initialization."
        )
        return remapped

    def load_model(self, filepath):
        """Load checkpoint while preserving expected per-intersection dictionary keys."""
        loaded_model = torch.load(filepath, map_location=self.device, weights_only=False)
        loaded_target_model = torch.load(filepath, map_location=self.device, weights_only=False)

        self.model = self._remap_loaded_models(loaded_model)
        self.target_model = self._remap_loaded_models(loaded_target_model)

        for tl_id in self.model:
            if self.model[tl_id] is not None:
                self.model[tl_id] = self.model[tl_id].to(self.device)
        for tl_id in self.target_model:
            if self.target_model[tl_id] is not None:
                self.target_model[tl_id] = self.target_model[tl_id].to(self.device)

    def _get_homogeneous_memory(self, tl_id, max_len):
        """Sample uniformly from a single replay buffer (no phase/action gating)."""
        replay_memory = self.replay_buffer[tl_id]
        replay_len = len(replay_memory)
        if replay_len == 0 or max_len <= 0:
            return []

        sample_size = min(int(max_len), replay_len)
        if sample_size == replay_len:
            batch = list(replay_memory) if not isinstance(replay_memory, list) else replay_memory.copy()
            np.random.shuffle(batch)
            return batch

        idx = np.random.choice(replay_len, size=sample_size, replace=False)
        if isinstance(replay_memory, list):
            return [replay_memory[i] for i in idx]

        replay_list = list(replay_memory)
        return [replay_list[i] for i in idx]

    def get_reward(self, tl_id, change_phase=None):
        pressure = MaxPressureStrategy._compute_pressure(self, self.network.TLS_DETECTORS[tl_id])
        base_reward = -np.nanmean(list(pressure.values())) / 2000
        scale = episode_reward_scale(
            self.episode_duration,
            tl_id=tl_id,
            reference_duration=self.REWARD_REFERENCE_EPISODE_DURATION,
        )
        return base_reward * scale