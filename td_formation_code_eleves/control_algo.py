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

global a_visiter   # Variable qui indice les points à visiter par le camion
a_visiter = 0

global Plein       # Si le drone est plein, il rejoint le camion, s'il est vide, il rejoint le point de ravitaillement
Plein = True

global nbr_aller_retour # Compte le nombre d'aller retour que réalise le drone
nbr_aller_retour = 0


# =============================================================================




# =============================================================================
def formation(t, robotNo, robots_poses):
# =============================================================================  

    # --- example of modification of global variables ---
    # ---(updated values of global variables will be known at next call of this funtion) ---
    # global toto
    # toto = toto +1
    global firstCall
    global a_visiter
    global Plein
    global nbr_aller_retour

    
    
    
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

    A[0, 1] = 0
    
    if (firstCall):  # print information (only once)
        print(A)
        
        firstCall = False
    

    
    
    # initialize control input vector
    ui = np.zeros(2)
    kp = 0.1
    kp2 = 0.2
    
    # Points à visiter pour le camion

    visit = np.array([[- 6, 6], [6, 6], [6, - 6], [- 6, - 6]])  # Points que le camion doit visiter
    origin_drone = [- 10, - 10]                                 # Points de ravitaillement du drone



    # ===================== COMPUTATION OF ui =================================
        

    # Camion
    if i == 0:
        if math.dist(x[i], visit[a_visiter]) < 0.05:    # Si la distance entre le point fcitf et le camion assez faible, on change de point
            a_visiter = (a_visiter + 1) % 4
        ui = ui - kp*(x[i] - visit[a_visiter])
    
    # Drone
    else:
        if nbr_aller_retour < 3:
            if Plein:
                if math.dist(x[i], x[0]) < 0.1:         # Si la distance entre le drone et le camion est assez faible, il se vide
                    Plein = False
            else:
                if math.dist(x[i], origin_drone) < 0.1: # Si la distance entre le drone et le point de ravitaillement est assez faible, il se remplit
                    Plein = True
                    nbr_aller_retour += 1    
            if Plein:
                ui = ui - kp2*(x[i] - x[0]) + repulsion(x[i], [0, 0])             # Si le drone est plein, il rejoint le camion
            else:
                ui = ui - kp2*(x[i] - origin_drone) + repulsion(x[i], [0, 0])     # Si le drone est vide, il rejoint le point de ravitaillement

        
    
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

def repulsion(pos_self, pos_other, threshold=3.0, gain=10.0):
    direction = pos_self - pos_other
    distance = np.linalg.norm(direction)
    
    if distance < threshold and distance > 1e-6:
        force = gain * (1.0 / distance - 1.0 / threshold) * (direction / distance)
        return force
    else:
        return np.zeros_like(pos_self)
