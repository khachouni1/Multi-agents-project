# -*- coding: utf-8 -*-
"""
author: Sylvain Bertrand, 2023

   All variables are in SI units
    
   
   Variables used by the functions of this script
    - t: time instant (s)
    - robotNo: no of the current robot for which control is coputed (0 .. nbRobots-1)
    - poses:  size (3 x nbRobots)
        eg. of use: the pose of robot 'robotNo' can be obtained by: poses[:,robotNo]
            poses[robotNo,0]: x-coordinate of robot position (in m)
            poses[robotNo,1]: y-coordinate of robot position (in m)
            poses[robotNo,2]: orientation angle of robot (in rad)   (in case of unicycle dynamics only)
"""


import numpy as np
import math



# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ==============
# all variables declared here will be known by functions below
# use keyword "global" inside a function if the variable needs to be modified by the function


# global toto

global firstCall   # this variable can be used to check the first call ever of a function
firstCall = True



# =============================================================================




# =============================================================================
def formation(t, robotNo, robots_poses):
# =============================================================================  

    # --- example of modification of global variables ---
    # ---(updated values of global variables will be known at next call of this funtion) ---
    # global toto
    # toto = toto +1
    global firstCall
    
    
    
    # number of robots (short notation)
    N = robots_poses.shape[0]
    
    # get index of current robot  (short notation)
    i = robotNo
   
    # get positions of all robots
    x = robots_poses[:,0:2]
     
    # control law
    vx = 0.
    vy = 0.


    # adjacencdy matrix of communication graph
    # -----------------------------------------
    A= np.ones((N, N)) - np.eye(N)
    
    if (firstCall):  # print information (only once)
        print(A)
        
        
        firstCall = False
    

    
    
    # initialize control input vector
    ui = np.zeros(2)
    
    
    # ===================== COMPUTATION OF ui =================================
        
    k_L = 1
    k_F = 5
    x_star_0_dot = 0
    r_dot = 0

    if robots_poses[0, 0] < -2.5:
        x_ref = np.array([-2.5, 0])
        r = np.array([[0,0],[1, 0],[2, 0],[-1, 0]])
    if robots_poses[0, 0] < 2.5 and robots_poses[0, 0] > -2.5:
        x_ref = np.array([7.5, 0])
        r = np.array([[0,0],[1, 0],[2, 0],[-1, 0]])
    else:
        x_ref = np.array([7.5, -1])
        r = np.array([[0,0],[0, 2],[1, 1],[- 1, 1]])


    #r = np.array([[-0.5*(j+1),-0.5*(j+1)] for j in range(10)])
    u0 = -k_L*(x[i]-x_ref) + x_star_0_dot
    if i == 0:
        # leader
        ui = u0
    else:
        # follower
        ui = -k_F*((x[i]-x[0]) - r[i]) + r_dot + u0
    
    # =========================================================================
    
    
    # retrieve values from control input vector
    vx = ui[0]
    vy = ui[1]
    
    
    return vx, vy
# =============================================================================







# general template of a function defining a control law
# =============================================================================
def my_control_law(t, robotNo, robots_poses):
# =============================================================================  

    # --- example of modification of global variables ---
    # ---(updated values of global variables will be known at next call of this funtion) ---
    # global toto
    # toto = toto +1

    # number of robots
    nbOfRobots= robots_poses.shape[0]
    
    
    # control law
    vx = 0.
    vy = 0.

    # .................  TO BE COMPLETED HERE .............................
    
    return vx, vy
# =============================================================================

