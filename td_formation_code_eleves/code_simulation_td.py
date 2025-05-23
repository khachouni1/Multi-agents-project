# -*- coding: utf-8 -*-
"""
Simulation : 2 camions + 4 drones avec répulsion
"""

import numpy as np
from lib.simulation import FleetSimulation
from lib.robot import Fleet, si_to_uni
import control_algo
import matplotlib.pyplot as plt

# === Paramètres ===
nbOfRobots = 6
robotDynamics = 'singleIntegrator2D'

initPositions = np.array([
    [-6, -6],     # Camion 1
    [6, 6],       # Camion 2
    [-10, -10],   # Drone 1
    [-10, -6],    # Drone 2
    [10, -10],    # Drone 3
    [10, -6]      # Drone 4
])

if robotDynamics == 'singleIntegrator2D':
    fleet = Fleet(nbOfRobots, dynamics=robotDynamics, initStates=initPositions)
else:
    initAngles = np.zeros((nbOfRobots, 1))
    initPoses = np.concatenate((initPositions, initAngles), axis=1)
    fleet = Fleet(nbOfRobots, dynamics=robotDynamics, initStates=initPoses)

Ts = 0.01
simulation = FleetSimulation(fleet, t0=0.0, tf=400.0, dt=Ts)

for t in simulation.t:
    robots_poses = fleet.getPosesArray()
    for robotNo in range(fleet.nbOfRobots):
        vx, vy = control_algo.formation(t, robotNo, robots_poses)
        fleet.robot[robotNo].ctrl = np.array([vx, vy])
    simulation.addDataFromFleet(fleet)
    fleet.integrateMotion(Ts)

# === Visualisation simple (compatibilité assurée)
simulation.plotXY(figNo=2)
simulation.plotState(figNo=3)
simulation.plotCtrl(figNo=4)

plt.show()
