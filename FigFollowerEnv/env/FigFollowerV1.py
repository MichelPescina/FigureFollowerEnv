import numpy as np
import pybullet as p
import pybullet_data
from pybullet_utils import bullet_client
import gymnasium as gym
from FigFollowerEnv.env.MapGenerator import MapGenerator
from FigFollowerEnv.get_assets_path import get_assets_path
from importlib.util import find_spec
from typing import Optional
import math
import os

class FigFollowerV1(gym.Env):
    """
    The FigFollower class implements a gymnasium evironment for the task involving an
    omnidirectional robot (with 4 mecanum wheels) following several geometric signals that show
    the direction of the next signal. There are four different shapes and each type has a different
    meaning: triangle, turn left; square, turn right; pentagon, turn around; and circle, stop.
    The agent should learn what to do for each case given the image it receives from the simulation.
    
    Reward is given as the robot approximates the target signal.
    There is not a reward limit as the maps can contain different number of signals.
    A penalization is given each time the robot collides with something as well for activating the motors
    but in a lesser extent.
    """

    metadata = {"render_modes": ["rgb_array"], "render_fps":30}
    def __init__(self, width=320, height=240, fps=30, max_speed = 10.0, max_time = 180, nodes = 10, render_mode = None):
        """
        Initializes an instance of the class FigFollower

        Parameters
        ----------
        width : int
            The width of the robot's camera field of view.
        height : int
            The height of the robot's camera field of view.
        fps : int
            The refresh rate at which the camera sends images. Useful for simulating lag.
        max_speed : float
            Maximum speed for the wheels to spin.
        max_time : int
            Maximum number of simulation seconds allowed.
        nodes : int
            How many signals are in the path.
        render_mode : str
            Indicates Gymansium how to render this environment.
        """

        self.resources_path = get_assets_path()
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(4,))
        self.observation_space = gym.spaces.Box(low=0, high=255, shape=(height, width, 3), dtype=np.uint8)
        self._width = width
        self._height = height
        self._proj_matrix = p.computeProjectionMatrixFOV(70, (width/height), 0.0001, 30.0)
        self._p = bullet_client.BulletClient(p.DIRECT)
        self._p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.phy_client_id = self._p._client
        self.render_mode = render_mode
        try:
            if os.environ["PYBULLET_EGL"]:
                egl = find_spec('eglRenderer')
                if egl:
                    self._p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
                else:
                    self._p.loadPlugin("eglRendererPlugin")
        except:
            pass
        self.fps = fps
        self._map_gen = MapGenerator(20, nodes, 10, 3)
        self._skip_frames = 240 // fps
        self._max_timestep = int(max_time * fps)
        self._max_speed = max_speed
        self._tile_size = 0.2 # The map generator creates a route using tiles, the tile size is in meters
        self.timestep = 0
        self._robot_id = None
        self._cam_joint = None
        self._cam_up_joint = None
        self._cam_front_joint = None
        self._FR_joint = None
        self._FL_joint = None
        self._BR_joint = None
        self._BL_joint = None
        self._floor_id = None
        self.nodes = None
        self.target_node = 0
        self._prev_dist = 0
        self._last_pos = None
        self._end_step = self._max_timestep


    def reset(self, seed: Optional[int] = None, options = None):
        """
        Resets the environment to an initial state

        Parameters
        ----------
        seed : int
            Optional. Useful for creating a deterministic environment.
        
        Returns
        -------
        obs : numpy.array
            The first observation of the environment after reset. It's an image!
        info : dict
            An empty info, added for conformity to the Gymnasium standard.
        """

        super().reset(seed=seed)
        self._p.resetSimulation()
        self._p.setGravity(0, 0, -9.8)
        self._floor_id = self._p.loadURDF("plane.urdf", [0, 0, 0])
        self.timestep = 0
        self._end_step = self._max_timestep
        self._prev_dist = 0
        self.target_node = 0
        self.nodes, _ = self._map_gen.generate()
        self._last_pos = None
        self._load_nodes(self.nodes)
        self._identify_joints()
        obs = self.render()
        info = self._get_info()
        #print(self.nodes)
        return obs, info


    def step(self, action : np.array):
        """
        Steps the simulation given the fps at initialization. E.g. if fps = 30
        then the simulation steps the physics server 8 times to simulate a refresh rate
        of this magnitude. The physics server runs needs to update 240 times to simulate
        a real second, thus the formula for computing the steps skipped is 240/fps.

        Parameters
        ----------
        action : numpy.array
            A numpy array containing the an activation value for each motor in the following
            order : [Front_Right, Front_Left, Back_Right, Back_Left].
        """
        joints = [self._FR_joint,
                  self._FL_joint,
                  self._BR_joint,
                  self._BL_joint
                  ]
        mode = p.VELOCITY_CONTROL
        action = np.clip(action, -1.0, 1.0)
        motor_usage = np.mean(np.absolute(action)) / (self.fps * 10)
        self._p.setJointMotorControlArray(self._robot_id,
                                          joints,
                                          controlMode = mode,
                                          targetVelocities = action * self._max_speed
                                          )
        for i in range(self._skip_frames):
            self._p.stepSimulation()
        self.timestep += 1
        terminated = False if self.timestep < self._end_step or self.target_node < len(self.nodes) else True
        truncated = True if self.timestep >= self._max_timestep else False
        reward = self._compute_reward(motor_usage)
        observation = self.render()
        info = self._get_info()
        return observation, reward, terminated, truncated, info
    

    def _compute_reward(self, motor_usage):
        pos_a = self._get_robot_pos()
        pos_b = self._get_target_pos(self.target_node)
        dist = math.sqrt(np.power((pos_b - pos_a), 2).sum())
        # Check if the robot is close to the target node
        if self.target_node < len(self.nodes) and dist < self._tile_size * 2:
            if self.target_node == len(self.nodes)-1:
                self._end_step = min(self.timestep + self.fps * 5, self._max_timestep)
                self._last_pos = pos_a
            self.target_node += 1
            pos_b = self._get_target_pos(self.target_node)
            dist = math.sqrt(np.power((pos_b - pos_a), 2).sum())
            self._prev_dist = dist
        # Compute improvement
        if self.target_node >= len(self.nodes):
            self._last_pos = pos_a
            improvement = 0.0
        else:
            improvement = max(self._prev_dist - dist, 0.0)
            self._prev_dist = dist if improvement > 0.0 else self._prev_dist
        # Check for collisions
        collisions = self._is_colliding()
        return (improvement / self._tile_size) - collisions - motor_usage


    def _is_colliding(self):
        vorbo = p.getContactPoints(bodyA = self._robot_id)
        flago = 0.0
        for v in vorbo:
            if v[2] != self._floor_id:
                flago = 1.0
        return flago


    def _get_target_pos(self, id):
        if id < len(self.nodes):
            data_b = self.nodes[id]
            x = data_b.x * self._tile_size
            y = data_b.y * self._tile_size
        else:
            x = self._last_pos[0]
            y = self._last_pos[1]
        return np.array([x, y])
    

    def _get_robot_pos(self):
        data_a = self._p.getLinkState(self._robot_id, self._cam_joint)
        x = data_a[0][0]
        y = data_a[0][1]
        return np.array([x, y])


    def _load_nodes(self, nodes):
        for node in nodes:
            pos = [node.x * self._tile_size, node.y * self._tile_size, 0.07]
            ori = self._p.getQuaternionFromEuler([0, 0, math.radians(node.angle - 180)])
            if node.type == 'Origin':
                ori = self._p.getQuaternionFromEuler([0, 0, 0])
                self._robot_id = self._loadURDF("omni_robot/omni_robot.urdf", [pos[0], pos[1], 0.1], ori)
            elif node.type == 'Triangle':
                self._loadURDF("triangle/triangle.urdf", pos, ori, 0, 1)
            elif node.type == 'Square':
                self._loadURDF("square/square.urdf", pos, ori, 0, 1)
            elif node.type == 'Pentagon':
                self._loadURDF("pentagon/pentagon.urdf", pos, ori, 0, 1)
            elif node.type == 'Circle':
                self._loadURDF("circle/circle.urdf", pos, ori, 0, 1)
    

    def _loadURDF(self, path : str, pos, ori, max_coords = 0, fixed_base = 0):
        real_path = self.resources_path + "/" + path
        id = self._p.loadURDF(real_path, pos, ori, useMaximalCoordinates=max_coords, useFixedBase=fixed_base)
        return id
    

    def _identify_joints(self):
        total_joints = self._p.getNumJoints(self._robot_id)
        for i in range(total_joints):
            joint = p.getJointInfo(self._robot_id, i)
            if joint[1] == b'base_link_to_FR_wheel':
                self._FR_joint = joint[0]
            elif joint[1] == b'base_link_to_FL_wheel':
                self._FL_joint = joint[0]
            elif joint[1] == b'base_link_to_BR_wheel':
                self._BR_joint = joint[0]
            elif joint[1] == b'base_link_to_BL_wheel':
                self._BL_joint = joint[0]
            elif joint[1] == b'base_link_to_cam':
                self._cam_joint = joint[0]
            elif joint[1] == b'cam_to_cam_dir':
                self._cam_front_joint = joint[0]
            elif joint[1] == b'cam_to_cam_up':
                self._cam_up_joint = joint[0]


    def _get_info(self):
        return {'target_node': self.target_node,
                'total_nodes': len(self.nodes)
                }


    def render(self):
        data_a = self._p.getLinkState(self._robot_id, self._cam_joint)
        data_b = self._p.getLinkState(self._robot_id, self._cam_front_joint)
        data_c = self._p.getLinkState(self._robot_id, self._cam_up_joint)
        pos_a = np.array(data_a[0])
        pos_b = np.array(data_b[0])
        pos_c = np.array(data_c[0])
        pos_c = pos_c - pos_a
        up = pos_c/np.sqrt((pos_c*pos_c).sum())
        view_matrix = p.computeViewMatrix(pos_a, pos_b, up)
        frame = self._p.getCameraImage(
            self._width,
            self._height,
            viewMatrix=view_matrix,
            projectionMatrix=self._proj_matrix,
            shadow=1,
            lightDirection=[8, 8, 2]
        )
        img = frame[2][:,:,:3]
        return img
    

    def close(self):
        if (self.phy_client_id >= 0):
            self._p.disconnect()
            self.phy_client_id = -1
    

