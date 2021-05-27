from gym.envs.registration import register

register(
    id='carpark_arena-v0',
    entry_point='carpark_arena.envs:SimpleDrivingEnv',
)