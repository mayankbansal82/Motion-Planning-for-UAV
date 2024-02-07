from map import Map, Cuboid
from rrt_star import RRT_star
from map_reader import MapReader
from trajGen import *
from trajutils import *
import numpy as np
import time

map_file = "Tello_maps/custom.txt"  # Replace with the path to your map file
map_reader = MapReader(map_file)
map = map_reader.getMap()
map.set_buffer(0.5)
# map_visualizer = MapVisualization(map)
# rrt_visualizer = RRTVisualization()
rrt_planner = RRT_star(map)

start = [0,0,1.0]
goal = [-0.4, 2.0, 1.0]

rrt_planner.reset_search_reslut()
rrt_planner.set_start_and_goal(start,goal)
rrt_planner.set_max_samples(500)
rrt_planner.search()
waypoints,_,_ = rrt_planner.retrieve_path()

waypoints = np.array(waypoints)

traj = trajGenerator(waypoints, max_vel = 1.0, gamma = 100)

Tmax = traj.TS[-1]

t = 0.0
t_step = 1.0
tello_vel = 50


state = traj.get_des_state(t)
while t < Tmax:

    t += t_step
    next_state = traj.get_des_state(t)
    pos = state[0]
    # vel = state[1]
    next_pos = next_state[0]
    # next_vel = next_state[1]
    
    state = next_state

    pos_vector = 100*(next_pos - pos)
    # avg_vel = pos_vector/t_step
    # avg_vel_mag = np.linalg.norm(avg_vel)
    vel = np.linalg.norm(pos_vector)/0.5
    
    print("pos vector = ", pos_vector)
    # tello.go_xyz_speed(int(pos_vector[0]), int(pos_vector[1]), int(pos_vector[2]), int(tello_vel))
    
    print(np.linalg.norm(pos_vector)/tello_vel)
    time.sleep(np.linalg.norm(pos_vector)/tello_vel)
