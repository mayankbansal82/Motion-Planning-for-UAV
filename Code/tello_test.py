from map import Map, Cuboid
from rrt_star import RRT_star
from map_reader import MapReader
from trajGen import *
from trajutils import *
import numpy as np
import time

map_file = "/home/blacksnow/drones/TrainSetP2b/maps/map1.txt"  # Replace with the path to your map file
map_reader = MapReader(map_file)
map = map_reader.getMap()
map.set_buffer(0.5)
# map_visualizer = MapVisualization(map)
# rrt_visualizer = RRTVisualization()
rrt_planner = RRT_star(map)

start = [0,0,1.0]
goal = [-0.59, 6.81, 1.0]

rrt_planner.reset_search_reslut()
rrt_planner.set_start_and_goal(start,goal)
rrt_planner.set_max_samples(2000)
rrt_planner.search()
waypoints,_,_ = rrt_planner.retrieve_path()

waypoints = np.array(waypoints)

traj = trajGenerator(waypoints, max_vel = 1, gamma = 100)

Tmax = traj.TS[-1]

t = 0.0
t_step = 0.1
state = traj.get_des_state(t)
while t < Tmax:
    start_time = time.time()
    t += t_step
    next_state = traj.get_des_state(t)
    pos = state[0]
    vel = state[1]
    next_pos = next_state[0]
    next_vel = next_state[1]
    
    state = next_state

    pos_vector = next_pos - pos
    avg_vel = (vel + next_vel)/2
    avg_vel_mag = np.linalg.norm(avg_vel)
    
    # tello.go_xyz_speed(pos_vector[0], pos_vector[1], pos_vector[2],avg_vel_mag)
    time.sleep(0.02)
    command_time = time.time()
    
    time_to_sleep = t_step - (command_time-start_time)
    time.sleep(time_to_sleep)
    print(time_to_sleep)
    