# Sample DJI Tello EDU code

# Illustrates the following:
#    1. states access
#    2. video feed access
#    3. control commands

# Install the following library (or download and add to sys.path)
# https://github.com/damiafuentes/DJITelloPy#djitellopy

# Example codes:
# https://github.com/damiafuentes/DJITelloPy/tree/master/examples

# DJI Tello Py library documentation:
# https://djitellopy.readthedocs.io/en/latest/tello/

# Tello UDP COMMAND documentation (used by djitellopy library):
#     [1.3](https://dl-cdn.ryzerobotics.com/downloads/tello/20180910/Tello%20SDK%20Documentation%20EN_1.3.pdf),
#     [2.0 with EDU-only commands](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf)
#     https://djitellopy.readthedocs.io/en/latest/

#  Please note that the commands are transmitted over UDP and responses are received over UDP.
#   Sometimes command or response (ok) may not be received!
#   This code is written to handle most of such corner cases, but you may still face them!

# Ignore video decoding error!
    # non-existing PPS 0 referenced
    # non-existing PPS 0 referenced
    # decode_slice_header error
    # no frame!

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

map_file = "/home/pear/drones/rbe595_p2b/msdiwan_p2a/TestSetP2b/maps/map1.txt"  # Replace with the path to your map file
map_reader = MapReader(map_file)
map = map_reader.getMap()
map.set_buffer(0.2)
# map_visualizer = MapVisualization(map)
# rrt_visualizer = RRTVisualization()
rrt_planner = RRT_star(map)

start = [0,0,0]
goal = [-0.59, 6.81, 0]
waypoints = None
# waypoints = [[ 0.,          0.,          0.        ],
#             [ 0.96711965,  1.93329184,  0.36769118],
#             [ 0.69651409,  5.9185722,   0.17516881],
#             [-0.59,        6.81,        0.        ]]
# waypoints =[[ 0.,          0.,          0.        ],
#             [ 1.21887354,  1.84322906,  0.17326733],
#             [ 1.05321643,  5.54555131,  0.02804601],
#             [ 0.61296649,  6.22138917,  0.19379183],
#             [-0.59,        6.81,        0.        ]]
waypoints = [[ 0.,          0.,          0.        ],
 [-0.78109729,  1.83422813,  0.40643553],
 [-0.97414203,  3.72119254,  0.36649994],
 [-0.59,        6.81,        0.        ]]
if(not waypoints):
    rrt_planner.reset_search_reslut()
    rrt_planner.set_start_and_goal(start,goal)
    rrt_planner.set_max_samples(500)
    rrt_planner.search()
    waypoints,_,_ = rrt_planner.retrieve_path()

waypoints = np.array(waypoints)
print(waypoints)

traj = trajGenerator(waypoints, max_vel = 1.0, gamma = 100)

Tmax = traj.TS[-1]

# INIT TELLO CLASS
tello = Tello(retry_count=1)

# HELPERS
def takeoffHelper(tobj):
    attempt = 0
    TAKEOFF_TIMEOUT = 10
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

# CHECK VIDEO FEED ----------------------------------------------------------------------
# tello.streamon()
# time.sleep(2)
# frame_read = tello.get_frame_read()
# time.sleep(2)
# cv2.imwrite("picture.png", frame_read.frame)

# TAKEOFF--------------------------------------------------------------------------------

    #   technique 1 - Call tello.takeoff() - 
    #       in case the takeoff works but ok is not received, it will think that takeoff is incomplete
    #      tello.takeoff()

# technique 2 - send takeoff command and dont expect a response. see if altitude crosses some preset value
# if takeoffHelper(tello) == False:
#     print('takeoff failed after multiple attempts')
#     exit(-1)



time.sleep(3.0)
tello.takeoff()

#time.sleep(6.0)
#current_height = tello.get_distance_tof()
#print("current height: ", current_height)
#des_height = 120
#height_change = des_height-current_height
#if(height_change >=0):
#    tello.move_up(height_change)
#else:
#    tello.move_down(-height_change)
    
t = 0.0
t_step = 0.01
# tello_vel = 30


state = traj.get_des_state(t)
des_pos = state[0]
#pos_actual = des_pos
#pos_error = des_pos - pos_actual
vel = state[1]
vel = 100 * vel

try:
	while t < Tmax:

		# vel_norm = np.linalg.norm(vel)
		# index = np.argmax(vel)
		# vel_actual = np.round(vel)
		# vel_actual_norm = np.linalg.norm(vel_actual)
		# diff = vel_actual_norm - vel_norm
		# vel_actual[index] += round(diff)
		#pos_error[2] = 0
		# vel_actual = np.round(vel + 0.01*pos_error)
		vel_actual = np.round(vel)
		cmd = f"rc {-vel_actual[1]} {vel_actual[0]} {vel_actual[2]} 0"
		tello.send_command_without_return(cmd)
		start_time = time.time()

		t += t_step
		next_state = traj.get_des_state(t)
		# print('t=', t)
		state = next_state

		des_pos = state[0]
		vel = state[1]
		vel = 100 * vel

		# vel_norm = np.linalg.norm(vel)
		# if(vel_norm >= 1.0):
		    
		#pos_actual += vel_actual*t_step
		#pos_error = des_pos - pos_actual

		# tello.go_xyz_speed(int(pos_vector[0]), int(pos_vector[1]), int(pos_vector[2]), int(tello_vel))

		end_time = time.time()
		time_to_sleep = t_step - (end_time-start_time)


		# print(np.linalg.norm(pos_vector)/tello_vel)
		# print("pos: ", 100*pos)
		# print("vel: ", vel)
		# print("Time to sleep: ", time_to_sleep)

		time.sleep(time_to_sleep)
		# time.sleep(t_step)


	tello.land()
    

except KeyboardInterrupt:
    # HANDLE KEYBOARD INTERRUPT AND STOP THE DRONE COMMANDS
    print('keyboard interrupt')
    tello.emergency()
    tello.emergency()
    # tello.land()
    tello.end()
