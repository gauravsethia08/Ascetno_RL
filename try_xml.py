# Importing Libraries
import os
import time
import numpy as np
import pandas as pd
import mujoco_py as mjp
from squaternion import Quaternion

# Defining path and constructing simulation
xml_path = "/home/bmsit/Ascento/jointed_limited/ascento_joint_limited.xml"

# Making the simulation
model = mjp.load_model_from_path(xml_path)
sim = mjp.MjSim(model)
viewer = mjp.MjViewer(sim)


# Looping over the simulation
sim_state = sim.get_state()
# print(len(sim_state.qpos))
# print(sim_state.qpos)
# print(len(sim_state.qvel))
# print(sim_state.qvel)

q = [1,0,0,0]
q[0] = sim.data.sensordata[0]
q[1] = sim.data.sensordata[1]
q[2] = sim.data.sensordata[2]
q[3] = sim.data.sensordata[3]

# Converting Quat to euler
quat = Quaternion(q[0],q[1],q[2],q[3])
eu = quat.to_euler(degrees=True)
print(eu)

# print(sim_state.qpos[3:7])
quat = Quaternion(sim_state.qpos[3], sim_state.qpos[4], sim_state.qpos[5], sim_state.qpos[6])
eu = quat.to_euler(degrees=True)
print(eu)

# while True:
#     sim.set_state(sim_state)

#     action_ = [0, 0, 0, 0]
#     action_[0] = 0
#     action_[1] = 0 
#     action_[2] = 0 
#     action_[3] = 1

#     sim.data.ctrl[0] = action_[0]#0.375
#     sim.data.ctrl[1] = action_[1]#0.375
#     sim.data.ctrl[2] = action_[2]#0.375
#     sim.data.ctrl[3] = action_[3]#0.375

#     wheel_qvel = sim.data.get_joint_qvel("Stoupento__wheel_link1R")
#     print(wheel_qvel)
    
#     sim.step()
#     sim_state = sim.get_state()
#     viewer.render()
#     time.sleep(0.1)
#     # b=time.time()-a
#     #print("Loop time ",b)

