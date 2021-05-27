import gym 
import pybullet as p
import pybullet_data
import carpark_arena
import time

if __name__=="__main__":
    env = gym.make(
        "carpark_arena-v0",
        render = False
    )
    print(100 * "-")

    print('gym.make() function')
    print(env.__init__.__doc__)
    print(100 * "-")
    
    print("render function--")
    print(env.render.__doc__)
    print(100 * "-")

    print("reset_function--")
    print(env.reset.__doc__)
    print(100 * "-")

    print("step function--")
    print(env.step.__doc__)
    print(100 * "-")

    print("seed function--")
    print(env.seed.__doc__)
    print(100 * "-")
    
    print("close function--")
    print(env.close.__doc__)
    print(100 * "-")
