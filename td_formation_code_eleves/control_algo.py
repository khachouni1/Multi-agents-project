#contrôle algo_ 2drones et camions 

# -- coding: utf-8 --
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
# =============================================================================
# =============================================================================
def formation(t, robotNo, robots_poses):
    global firstCall, a_visiter, Plein, nbr_aller_retour

    N = robots_poses.shape[0]  # nombre total de robots
    i = robotNo
    x = robots_poses[:, :2]

    vx = 0.
    vy = 0.
    ui = np.zeros(2)

    kp = 0.1
    kp2 = 0.2

    visit = np.array([[-6, 6], [6, 6], [6, -6], [-6, -6]])  # points du camion
    origin_drone = np.array([[-10, -10], [10, -10], [0, 10]])  # 3 points de ravitaillement

    if firstCall:
        print("Première exécution. Adjacency matrix non utilisée ici.")
        firstCall = False

    # ---------------- CAMION ----------------
    if i == 0:
        if np.linalg.norm(x[i] - visit[a_visiter]) < 0.05:
            a_visiter = (a_visiter + 1) % 4
        ui = -kp * (x[i] - visit[a_visiter])

    # ---------------- DRONES ----------------
    else:
        # Calculer la distance du drone courant au camion
        distance_to_camion = np.linalg.norm(x[i] - x[0])

        # Identifier le drone le plus proche du camion
        drones_positions = x[1:]  # sans le camion
        distances = np.linalg.norm(drones_positions - x[0], axis=1)
        closest_drone_index = np.argmin(distances) + 1  # ajouter 1 car indexé à partir de 1

        # Calcul du point de ravitaillement le plus proche
        closest_supply_index = np.argmin(np.linalg.norm(x[i] - origin_drone, axis=1))
        supply_target = origin_drone[closest_supply_index]

        if i == closest_drone_index:
            # Drone actif pour livraison
            if nbr_aller_retour < 3:
                if Plein:
                    if distance_to_camion < 0.1:
                        Plein = False
                else:
                    if np.linalg.norm(x[i] - supply_target) < 0.1:
                        Plein = True
                        nbr_aller_retour += 1

                if Plein:
                    ui = -kp2 * (x[i] - x[0])
                else:
                    ui = -kp2 * (x[i] - supply_target)
        else:
            # Drone inactif : reste au point de ravitaillement
            ui = -kp2 * (x[i] - supply_target)

    return ui[0], ui[1]



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