def episode_reward_scale(episode_duration, tl_id=None, reference_duration=1000.0):
    """Scale rewards so 1000-step episodes keep historical magnitude."""
    if isinstance(episode_duration, dict):
        duration = episode_duration[tl_id]
    else:
        duration = episode_duration

    return float(reference_duration) / duration