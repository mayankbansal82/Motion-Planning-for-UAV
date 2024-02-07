# IMPORTS
import sys
import os
import cv2
import time

# sys.path.append('./DJITelloPy')
from djitellopy import Tello

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
goal = [2.0,0.0, 1.0]

# rrt_planner.reset_search_reslut()
# rrt_planner.set_start_and_goal(start,goal)
# rrt_planner.set_max_samples(500)
# rrt_planner.search()
# waypoints,_,_ = rrt_planner.retrieve_path()

# waypoints = np.array(waypoints)
# # print(waypoints)

# traj = trajGenerator(waypoints, max_vel = 1.0, gamma = 100)

# Tmax = traj.TS[-1]

# INIT TELLO CLASS
tello = Tello(retry_count=1)

# HELPERS
def takeoffHelper(tobj):
    attempt = 0
    TAKEOFF_TIMEOUT = 5
    MAX_ATTEMPT = 2
    takeoffdone = False
    while True:
        attempt += 1

        tobj.send_command_without_return("takeoff")
        start_time = time.time()

        while True:
            el_time = time.time() - start_time
            if tobj.get_distance_tof() > 80.0:
                takeoffdone = True
                print('Takeoff complete in seconds = ', el_time)
                print('Altitude ', tello.get_distance_tof())
                break
            elif el_time > TAKEOFF_TIMEOUT:
                takeoffdone = False
                break
            else:
                # sleep for 1 second and check again
                time.sleep(0.05)
        
        if takeoffdone:
            break
        elif attempt>=MAX_ATTEMPT:
            break
            
    return takeoffdone

# THE MAIN PROGRAM
try:
    # ESTABLISH CONNECTION ------------------------------------------------------------------
    attempt = 0
    while True:
        try:
            # ENTER COMMAND MODE AND TRY CONNECTING OVER UDP
            attempt += 1
            print("Takeoff attempt ", attempt)
            tello.connect()
        except:
            print('Failed to connect or it connected but "ok" not received. Retrying...')
            if attempt > 1:
                print('Failed to connect after multiple attempts')
                exit(-1)
        else:
            # No exception 
            break

    # CHECK SENSOR READINGS------------------------------------------------------------------
    print('Altitude ', tello.get_distance_tof())
    print('Battery, ', tello.get_battery())


    # if takeoffHelper(tello) == False:
    #     print('takeoff failed after multiple attempts')
    #     exit(-1)

    # time.sleep(5.0)
    des_height = 100
    while tello.get_distance_tof() != des_height:
        # current_height = tello.get_distance_tof()
        print("current height: ", tello.get_distance_tof())
        height_change = des_height-tello.get_distance_tof()
        if(height_change >=0):
            tello.move_up(height_change)
        else:
            tello.move_down(height_change)

except KeyboardInterrupt:
    # HANDLE KEYBOARD INTERRUPT AND STOP THE DRONE COMMANDS
    print('keyboard interrupt')
    tello.emergency()
    tello.emergency()
    tello.end()