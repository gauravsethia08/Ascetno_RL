# Importing Libraries
import math
import random
import numpy as np
from gym import utils, spaces
from scipy import ndimage
from gym.envs.mujoco import mujoco_env
from squaternion import Quaternion

# Function to apply force on the robot
def get_force(force_limit):
	# print("Small force applied") 
	sf = round(random.uniform(-force_limit, force_limit),2)
	new_xfrc = [[0,0,0,0,0,0],
				[sf*1.1,sf,0,0,0,0],
				[0,0,0,0,0,0],
				[0,0,0,0,0,0],
				[0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0]]
	return new_xfrc	
	

# Function to get quaternions from euler angles
def get_quaternion_from_euler(roll, pitch, yaw):
	"""
	Convert an Euler angle to a quaternion.

	Input
		:param roll: The roll (rotation around x-axis) angle in radians.
		:param pitch: The pitch (rotation around y-axis) angle in radians.
		:param yaw: The yaw (rotation around z-axis) angle in radians.

	Output
		:return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
	"""
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

	return [qx, qy, qz, qw]


# GYM Class for the agent
class Ascento(mujoco_env.MujocoEnv, utils.EzPickle):

	# Init Function
	# Initializing all the required parameters and classes
	def __init__(self):

		# Initial values of robot parameters
		self.x = 0 
		self.y = 0 
		self.z = 0 
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.vx = 0
		self.vy = 0
		self.vz = 0
		self.vroll = 0
		self.vpitch = 0
		self.vyaw = 0

		self.roll_pitch = []
		self.pitch_pitch = []
		self.z_list = []

		# Gains for rewards
		self.reward_scale = 1e-2
		self.reward_for_staying_alive = 5
		self.bonus_to_reach_goal = 15
		self.position_reward_constant = 5.0
		self.orientation_reward_constant = 0.02
		self.linear_velocity_reward_constant = 0.1
		self.angular_velocity_reward_constant = 0.01
		self.action_reward_constant = 0.25

		# Defining the desired set point
		self.desired_position = [-2, 0, 0.4]

		# Initial actions
		self.left_hip_cmd = 0.0
		self.right_hip_cmd = 0.0
		self.left_wheel_cmd = 0.0
		self.right_wheel_cmd = 0.0
		
		# Parameters for each episode
		self.ep_len = 0
		self.prev_reward = 0

		# Domain Randomization Params
		self.apply_force = False

		# Initializing the Mujoco GYM ENV
		utils.EzPickle.__init__(self)
		mujoco_env.MujocoEnv.__init__(self, "/home/gaurav/final_year_project/Ascetno_RL_Joint_Limited/Demo/Envs/Bump_RL.xml", 2)

		self.action_space = spaces.Box(np.array([-1, -1, -1, -1]), np.array([1, 1, 1, 1]))

	# STEP Function -> Corresponds to each step of simulation
	def step(self, a):

		# Updating the step counter
		self.ep_len += 1

		a_ = [0, 0, 0, 0]
		a_[0] = np.clip(a[0], -0.2, 0.3)
		a_[1] = np.clip(a[1], -0.2, 0.3)
		a_[2] = a[2] #* 0.2
		a_[3] = a[3] #* 0.2

		# Changing the reference value based on number of steps
		# To avoid very high initial reference, which causes the robot to be unstable
		# if self.ep_len <= 100:
		# 	ref = self.ep_len/100
		# else:
		# 	ref = 1
		#random.randint(2, 5)
		
		# Applying force randomly
		if self.apply_force:
			rand_xfrc = get_force(random.randint(10, 30))
			self.sim.data.xfrc_applied[:] = rand_xfrc

		# Doing simulation
		self.do_simulation(a_, self.frame_skip)

		# print(self.roll, self.pitch, self.yaw)
		def bonus_reward_to_achieve_goal(self, error_xyz):
			bonus = 0.0
			if self.norm(error_xyz) < 0.3:
				bonus += self.bonus_to_reach_goal
			return bonus


		# Defining the reward
		ref = -0.5
		# reward = 0. -3*(0.4-self.z)**2 - 3*(ref - self.x)**2 - 3*(self.y)**2 - 50*(self.pitch)**2 - 50*(self.yaw)**2 -50*self.roll**2#- 0.01*(self.left_wheel_cmd**2 + self.right_wheel_cmd**2) - 0.1*(self.left_hip_cmd**2 + self.right_hip_cmd**2)
		reward = 25 - 3*np.linalg.norm(ref - self.vx) - 50*(self.pitch)**2 - 50*(self.yaw)**2 - 10*np.linalg.norm(a[0:2]) - 0.1*np.linalg.norm(a[2:]) 
		# - 3*np.linalg.norm(self.y) - 3*np.linalg.norm(self.z)
		# if abs(ref - self.x) < 0.2:
		# 	reward+=15
		
		# alive_bonus = self.reward_for_staying_alive
		# reward_position = self.position_reward_constant * np.linalg.norm([self.desired_position[0] -self.x, self.desired_position[1] - self.y, 0])
		# reward_orientation = self.orientation_reward_constant * np.linalg.norm([self.roll, self.pitch, self.yaw])
		# reward_linear_velocity = self.linear_velocity_reward_constant * np.linalg.norm([self.vx, self.vy, self.vz])
		# reward_angular_velocity = self.angular_velocity_reward_constant * np.linalg.norm([self.vroll, self.vpitch, self.vyaw])
		# reward_action = self.action_reward_constant * np.linalg.norm(a)

		# rewards = (reward_position, reward_orientation, reward_linear_velocity, reward_angular_velocity, alive_bonus, reward_action)#, motor_efficiency) #reward_action)
		# #print("Motor efficiency",motor_efficiency)
		# reward = sum(rewards) * self.reward_scale
		
		# if np.abs(self.x - ref) < 0.2:
		# 	reward += 15
		# self.prev_reward = reward

		# Getting Observation
		ob = self._get_obs()

		# Extracting commands and updating pervious commands
		self.left_hip_cmd = a[0]
		self.right_hip_cmd = a[1]
		self.left_wheel_cmd = a[2]
		self.right_wheel_cmd = a[3]

		# print(self.pitch)

		# Defining the terminating conditions
		done = not (
            np.isfinite(ob).all() # -> True
            and (abs(self.pitch) < 1.2) # -> True
			and (abs(self.roll) < 1.2) # -> True
			and (self.ep_len < 2500)
        )

		

		if done:
			self.ep_len = 0
			# reward-=50


		return ob, reward, done, {}

	# Fucntion to resetting the model and simulation 
	def reset_model(self):
		# Generating Hilly Terrain
		# Constants related to terrain generation.
		# _HEIGHTFIELD_ID = 0
		# _TERRAIN_SMOOTHNESS = 0.65  # 0.0: maximally bumpy; 1.0: completely smooth.
		# _TERRAIN_BUMP_SCALE = 5  # Spatial scale of terrain bumps (in meters).

		# # Get heightfield resolution, assert that it is square.
		# res = self.sim.model.hfield_nrow[_HEIGHTFIELD_ID]
		# assert res == self.sim.model.hfield_ncol[_HEIGHTFIELD_ID]
		# # Sinusoidal bowl shape.
		# row_grid, col_grid = np.ogrid[-1:1:res*1j, -1:1:res*1j]
		# radius = np.clip(np.sqrt(col_grid**2 + row_grid**2), .04, 1)
		# bowl_shape = .5 - np.cos(2*np.pi*radius)/2
		# # Random smooth bumps.
		# terrain_size = 2 * self.sim.model.hfield_size[_HEIGHTFIELD_ID, 0]
		# bump_res = int(terrain_size / _TERRAIN_BUMP_SCALE)
		# bumps = np.random.uniform(_TERRAIN_SMOOTHNESS, 1, (bump_res, bump_res))
		# smooth_bumps = ndimage.zoom(bumps, res / float(bump_res))
		# # Terrain is elementwise product.
		# terrain = bowl_shape * smooth_bumps
		# start_idx = self.sim.model.hfield_adr[_HEIGHTFIELD_ID]
		# self.sim.model.hfield_data[start_idx:start_idx+res**2] = terrain.ravel()

		# Randomly choosing angle of the ramp
		# angle = -5#random.randint(15, 35)

		# if -angle > 25:
		# 	th = 0.2
		# 	dth = 0.15
		# else:
		# 	th = 0.1
		# 	dth = 0

		# # Changing Ramp Slope
		# quat_angle = get_quaternion_from_euler(0, math.radians(angle), 0)[::-1]
		# size_y = self.sim.model.geom_size[self.sim.model.geom_name2id('ramp_up')][1]
		# required_z = (size_y)*np.sin(math.radians(angle)) + 0.05
		# self.sim.model.geom_quat[self.sim.model.geom_name2id('ramp_up')] = quat_angle
		# self.sim.model.geom_pos[self.sim.model.geom_name2id('ramp_up')][2] = np.abs(required_z)

		# quat_angle = get_quaternion_from_euler(0, math.radians(-angle), 0)[::-1]
		# size_y = self.sim.model.geom_size[self.sim.model.geom_name2id('ramp_up_back')][1]
		# required_z = (size_y)*np.sin(math.radians(-angle)) - 0.05
		# self.sim.model.geom_quat[self.sim.model.geom_name2id('ramp_up_back')] = quat_angle
		# self.sim.model.geom_pos[self.sim.model.geom_name2id('ramp_up_back')][2] = np.abs(required_z)


		# # # For Straight Ramp
		# required_x = (size_y*2)*np.cos(math.radians(-angle))
		# required_z = (size_y*2)*np.sin(math.radians(-angle))
		# org_x = -2#self.sim.model.geom_pos[self.sim.model.geom_name2id('ramp_straight_back')][0]
		# # print(org_x + th + np.abs(required_x))
		# self.sim.model.geom_pos[self.sim.model.geom_name2id('ramp_straight_back')][0] = org_x - np.abs(required_x) + 0.5
		# self.sim.model.geom_pos[self.sim.model.geom_name2id('ramp_straight_back')][2] = np.abs(required_z) - 0.05

		# # For Down Ramp
		# quat_angle = get_quaternion_from_euler(0, math.radians(-angle), 0)[::-1]
		# size_y = self.sim.model.geom_size[self.sim.model.geom_name2id('ramp_down')][1]
		# required_z = (size_y)*np.sin(math.radians(-angle))
		# self.sim.model.geom_quat[self.sim.model.geom_name2id('ramp_down')] = quat_angle
		# size_straight = self.sim.model.geom_size[self.sim.model.geom_name2id('ramp_straight')][1]
		# current_straight = self.sim.model.geom_pos[self.sim.model.geom_name2id('ramp_straight')][0]
		# self.sim.model.geom_pos[self.sim.model.geom_name2id('ramp_down')][0] = current_straight - dth + size_straight*2
		# self.sim.model.geom_pos[self.sim.model.geom_name2id('ramp_down')][2] = np.abs(required_z)


		# Resetting Model Pos and Vel
		qpos = self.init_qpos + self.np_random.uniform(
			size=self.model.nq, low=-0.01, high=0.01
		)
		qvel = self.init_qvel + self.np_random.uniform(
			size=self.model.nv, low=-0.01, high=0.01
		)
		self.set_state(qpos, qvel)
		return self._get_obs()


	# Function to get observations
	def _get_obs(self):
		
		self.x = self.sim.data.qpos[0] # x
		self.vx = self.sim.data.qvel[0] # x'
		self.y = self.sim.data.qpos[1] # y
		self.vy = self.sim.data.qvel[1] # y'
		self.z = self.sim.data.qpos[2] # z
		self.vz = self.sim.data.qvel[2] #z'
		
		# Orientation in Quaterion 
		# q = [1,0,0,0]
		# q[0] = self.sim.data.sensordata[0]
		# q[1] = self.sim.data.sensordata[1]
		# q[2] = self.sim.data.sensordata[2]
		# q[3] = self.sim.data.sensordata[3]

		# Converting Quat to euler
		quat = Quaternion(self.sim.data.qpos[3], self.sim.data.qpos[4], self.sim.data.qpos[5], self.sim.data.qpos[6])
		eu = quat.to_euler(degrees=False)

		# Getting Joint Pos and Joint Vel
		# Left Hip 
		# self.left_hip_pos = self.sim.data.sensordata[4]
		# self.left_hip_vel = self.sim.data.sensordata[5]
		# # Right Hip
		# self.right_hip_pos = self.sim.data.sensordata[6]
		# self.right_hip_vel = self.sim.data.sensordata[7]
		# # Left Knee 
		# self.left_knee_pos = self.sim.data.sensordata[8]
		# self.left_knee_vel = self.sim.data.sensordata[9]
		# # Right Knee
		# self.right_knee_pos = self.sim.data.sensordata[10]
		# self.right_knee_vel = self.sim.data.sensordata[11]

		self.roll = eu[0]
		self.pitch = eu[1]
		self.yaw = eu[2] - 3.14
		self.vroll = self.sim.data.qvel[3] 
		self.vpitch = self.sim.data.qvel[4] # pitch'
		self.vyaw = self.sim.data.qvel[5]

		self.roll_pitch.append(math.degrees(eu[0]))
		self.pitch_pitch.append(math.degrees(eu[1]))
		self.z_list.append(self.z)
		# self.prev_cmd = [self.left_wheel_cmd, self.right_wheel_cmd, self.left_hip_cmd, self.right_hip_cmd] # prev cmds

		return np.concatenate(([self.x, self.y, self.z], 
								[self.roll, self.pitch, self.yaw], [self.vx, self.vy, self.vz],  
								[self.vroll, self.vpitch, self.vyaw])).ravel()


	def viewer_setup(self):
		v = self.viewer
		v.cam.trackbodyid = 0
		v.cam.distance = self.model.stat.extent
