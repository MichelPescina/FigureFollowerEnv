import gymnasium as gym
import FigFollowerEnv
import numpy as np
import cv2
import pybullet as p

if __name__ == "__main__":
    # Wheels
    FR = FL = BR = BL = 0
    env = gym.make('FigFollowerEnv-v1', width = 320, height = 240)
    obs, info = env.reset()
    step = 0
    while True:
        #print([FR, FL, BR, BL])
        if step % 300 == 0:
            print(info)
            step = 0
        step += 1
        cv2.imshow('Test', cv2.cvtColor(obs, cv2.COLOR_RGB2BGR))
        actions = np.array([FR, FL, BR, BL])
        obs, reward, terminated, truncated, info = env.step(actions)
        if terminated or truncated:
            obs, info = env.reset()
            FR = FL = BR = BL = 0
        key = cv2.waitKey(1)
        # Exit: escape
        if key == 27:
            break
        # Back left Wheel: a
        elif key == 97:
            BL = (BL + 1) % 2
        # Front left wheel: s
        elif key == 115:
            FL = (FL + 1) % 2
        # Front right wheel: d
        elif key == 100:
            FR = (FR + 1) % 2
        # Back roght wheel: f
        elif key == 102:
            BR = (BR + 1) % 2
    env.close()
