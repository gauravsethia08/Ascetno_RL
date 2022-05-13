# Importing Libraries
import os
import cv2
import copy
import math
import random
import numpy as np
import mujoco_py as mjp


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
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]


# Defining path and constructing simulation
xml_path = "/home/gaurav/final_year_project/Ramp/ascento_v2_mujoco_4.xml"
model = mjp.load_model_from_path(xml_path)
sim = mjp.MjSim(model)
viewer = mjp.MjViewer(sim)

# Randomly choosing angle of the ramp
angle = -random.randint(15, 35)

if -angle > 25:
    th = 0.2
    dth = 0.15
else:
    th = 0.1
    dth = 0

# Changing Ramp Slope
quat_angle = get_quaternion_from_euler(0, math.radians(angle), 0)[::-1]
size_y = sim.model.geom_size[sim.model.geom_name2id("ramp_up")][1]
required_z = (size_y) * np.sin(math.radians(angle))
sim.model.geom_quat[sim.model.geom_name2id("ramp_up")] = quat_angle
sim.model.geom_pos[sim.model.geom_name2id("ramp_up")][2] = np.abs(required_z)

# For Straight Ramp
required_x = (size_y * 2) * np.cos(math.radians(angle))
required_z = (size_y * 2) * np.sin(math.radians(angle))
org_x = sim.model.geom_pos[sim.model.geom_name2id("ramp_straight")][0]
sim.model.geom_pos[sim.model.geom_name2id("ramp_straight")][0] = (
    org_x + th + np.abs(required_x)
)
sim.model.geom_pos[sim.model.geom_name2id("ramp_straight")][2] = np.abs(required_z)

# For Down Ramp
quat_angle = get_quaternion_from_euler(0, math.radians(-angle), 0)[::-1]
size_y = sim.model.geom_size[sim.model.geom_name2id("ramp_down")][1]
required_z = (size_y) * np.sin(math.radians(-angle))
sim.model.geom_quat[sim.model.geom_name2id("ramp_down")] = quat_angle
size_straight = sim.model.geom_size[sim.model.geom_name2id("ramp_straight")][1]
current_straight = sim.model.geom_pos[sim.model.geom_name2id("ramp_straight")][0]
sim.model.geom_pos[sim.model.geom_name2id("ramp_down")][0] = (
    current_straight - dth + size_straight * 2
)
sim.model.geom_pos[sim.model.geom_name2id("ramp_down")][2] = np.abs(required_z)


# Looping over the simulation
sim_state = sim.get_state()
while True:
    # Setting state of the robot
    sim.set_state(sim_state)
    sim.step()
    sim_state = sim.get_state()
    image, depth = copy.deepcopy(
        sim.render(width=1080, height=720, camera_name="camera_1", depth=True)
    )
    extent = sim.model.stat.extent
    near = sim.model.vis.map.znear * extent
    far = sim.model.vis.map.zfar * extent
    # print(extent, near, far)
    image = near / (1 - depth * (1 - near / far))
    cv2.imwrite("camera_d.png", image)
    print(np.unique(image))
    viewer.render()
# print(sim.model.geom_quat[sim.model.geom_name2id('ramp')])
