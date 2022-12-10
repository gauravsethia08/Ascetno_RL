# Importing Libraries
from cProfile import label
import os
import math
import random 
import matplotlib.pyplot as plt
import mujoco_py as mjp
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
	

# Defining path and constructing simulation
xml_path = '/home/gaurav/final_year_project/Ascetno_RL_Joint_Limited/Demo/Envs/Flat.xml'
model = mjp.load_model_from_path(xml_path)
sim = mjp.MjSim(model)
viewer = mjp.MjViewer(sim)

apply_force = True



# Looping over the simulation
sim_state = sim.get_state()

# Defining variables
sum_of_error = 0
perv_error = 0
ref = 0

re = 0
kp = 6
ki = 0#1e5
kd = 0

pitch_list = []
roll_list = []
z_list = []
a2 = []
a3 = []
a4 = []

for _ in range(5000):
    # Setting state of the robot
    sim.set_state(sim_state)
    # Getting IMU data
    quat = Quaternion(sim.data.qpos[3], sim.data.qpos[4], sim.data.qpos[5], sim.data.qpos[6])
    eu = quat.to_euler(degrees=False)

    roll_list.append(math.degrees(eu[0]))
    pitch_list.append(math.degrees(eu[1]))
    z_list.append(sim.data.qpos[2])

    if apply_force == True:
        rand_xfrc = get_force(random.randint(10, 20))
        sim.data.xfrc_applied[:] = rand_xfrc

    #Computing error and other terms
    error = ref - eu[1] #+ 0 - sim.data.qpos[0]

    output = kp * error + kd * perv_error + ki * sum_of_error

    e2 = re - sim.data.qpos[2]
    re = re + 0.001
    o2 = kp*e2

    perv_error = error
    sum_of_error += error

    if abs(eu[1]) < 0.7:
        # Setting Speed of wheel motors
        sim.data.ctrl[0] = o2
        sim.data.ctrl[1] = o2
        sim.data.ctrl[2] = output
        sim.data.ctrl[3] = output
    else:
        # Setting Speed of wheel motors
        sim.data.ctrl[0] = 0
        sim.data.ctrl[1] = 0
        sim.data.ctrl[2] = 0
        sim.data.ctrl[3] = 0


    # print(output, sim.data.get_sensor('left_wheel_vel'), sim.data.get_sensor('right_wheel_vel'))
    # Taking action and updating the state of the robot
    sim.step()
    sim_state = sim.get_state()
    viewer.render()

# plt.subplot(2, 1, 1)
# plt.plot(roll_list)
# # plt.xlim(0, 5)
# plt.ylim(-30, 30)
# plt.xlabel("Time in mili-seconds")
# plt.ylabel("Roll in Degrees")
# plt.title("Roll")

# plt.subplot(2, 1, 2)
# plt.plot(pitch_list)
# # plt.xlim(0, 5)
# plt.ylim(-30, 30)
# plt.xlabel("Time in mili-seconds")
# plt.ylabel("Pitch in Degrees")
# plt.title("Pitch")
plt.rcParams.update({'font.size': 20})
plt.subplot_tool()
plt.tight_layout()
plt.subplot(2, 1, 1)
plt.plot(roll_list, label="Roll")
plt.plot(pitch_list, label="Pitch")
# plt.ylim(-90, 90)
plt.title("Orientation")
plt.xlabel("Time in mili-seconds")
plt.ylabel("Degrees")
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(z_list, label="Z Position")
plt.title("Position")
plt.xlabel("Time in mili-seconds")
plt.ylabel("Meter")
plt.legend()

if apply_force == True:
    plt.suptitle("PID on Bump Terrain with External Distrubance")
else:
    plt.suptitle("PID on Bump Terrain Terrain with No External Distrubance")

plt.show()