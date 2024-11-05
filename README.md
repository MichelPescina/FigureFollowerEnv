# Figure Follower Environment

This is a simple environment designed to be resolved by an an agent embodien in an omnidirectional robot using camera images as observations from the environment. The robot has four mecanum wheels actioned through independent motors. The agent must learn how to move through a path where each signal it encounters tells it where to go next. Each one of these signals have the form of geometric figures. The movements that the agent should learn to do once it sees the signals are the next:
* Triangle: go left.
* Square: go right.
* Pentagon: turn around.
* Circle: stop.

As the robot approaches the coming signal in the path the agent receives some reward, to be precise, it's one point of reward per meter. If the robot crashes with the signals the agent receives a penalization of 10 points.

## Usage
There are two ways to use this environment. The first one involves importing the ```gymnasiun``` package aswell as this one and the using the traditional ```gymnasium.make()``` function to create and initialize the environment as shown in the following code snippet.

```python
import gymnasium
import FigFollowerEnv

env = gym.make('FigFollowerEnv/FigFollower', render_mode="rgb_array")
```

But you can also create the environment this way.

```python
import gymnasium
import FigFollowerEnv

env = FigFollowerEnv.FigFollower()
```

There a couple of options you can use to customize this environment. These are all of them:
* width : int
  - The width of the robot's camera field of view.
* height : int
  - The height of the robot's camera field of view.
* fps : int
  - The refresh rate at which the camera sends images. Useful for simulating lag.
* max_speed : float
  - Maximum speed for the wheels to spin.
* max_time : int
  - Maximum number of simulation seconds allowed.
* nodes : int
  - How many signals are in the path.
* render_mode : str
  - Indicates Gymansium how to render this environment.

## Installation
Due to this package not being finished as of now I didn't upload it to PyPi (I'll keep updating it), also I felt the laziness creeping up my back so yeah, for now we will have to install it manually, to do that go to the project's folder and use this command.

```pip install .```

If you don't want to install the package you can also copy the FigFollowerEnv into your project and use it normally.

## Extra
If you need to use the URDF and XACRO files for the four-wheeled omnidirectional robot and the other assets, go ahead, it's in the assets folder. It was incredibly cumbersome making those models and if I can leverage some of that work to someone else I'm happy to have been of help.
