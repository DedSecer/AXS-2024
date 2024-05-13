from robot_tools.coordinate import CoordinateTools
import numpy as np

""" pose（position+orientation）坐标系转换（translation+rotation） """
robot_in_world = (np.zeros(3), np.array([np.pi / 2, np.pi / 2, 0]))
target_in_robot = (np.zeros(3), np.array([0, np.pi / 4, 0]))
target_in_world = CoordinateTools.to_world_coordinate(target_in_robot, robot_in_world)
print(target_in_world)

""" orientation坐标系转换（仅rotation） """
current_euler = [np.pi / 2, np.pi / 2, 0]
rela_euler = [0, np.pi / 4, 0]
target_euler = CoordinateTools.to_target_orientation(rela_euler, current_euler)
print(target_euler)

assert np.allclose(target_in_world[1], target_euler)