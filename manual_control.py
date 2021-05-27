import gym
import carpark_arena
import pybullet as p

env = gym.make(
    "carpark_arena-v0",
    render=True
)

action = [0.0, 0.0]
env.reset(
    shouldParkedCarsBeLoaded = True,
)
done = False

while True:
    next_state = [0, 0]
    if done:
        print("Episode Doneeeeeeeeeeeeeeeeeeeeeeeeeee\n\n\n")
        break
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        if k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN):
            action[1] -= 0.005
            next_state, reward, done,_ = env.step(action)
        if k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN):
            action[1] += 0.005
            next_state, reward, done,_ = env.step(action)
        if (k == p.B3G_LEFT_ARROW or k == p.B3G_RIGHT_ARROW) and (v & p.KEY_WAS_RELEASED):
            action[1] = 0.0
            next_state, reward, done,_ = env.step(action)
        if k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN):
            action[0] += 0.001
            next_state, reward, done,_ = env.step(action)
        if k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED):
            action[0] = 0
            next_state, reward, done,_ = env.step(action)
        if k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN):
            action[0] -= 0.001
            next_state, reward, done,_ = env.step(action)
        if k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED):
            action[0] = 0
            next_state, reward, done,_ = env.step(action)
    if next_state[0] == 0:
        continue
    else:
        print('coords:', (next_state[0], next_state[1]))
        

