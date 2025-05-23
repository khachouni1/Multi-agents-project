# -*- coding: utf-8 -*-
"""
Contrôle multi-agent : 2 camions, 4 drones
- Chaque drone dessert un camion
- Seul le plus proche de chaque camion agit
- Répulsion entre drones activée
"""

import numpy as np
import math

# === Fonction de répulsion entre drones ===
def repulsion(pos_self, pos_other, threshold=3.0, gain=10.0):
    direction = pos_self - pos_other
    distance = np.linalg.norm(direction)
    if distance < threshold and distance > 1e-6:
        force = gain * (1.0 / distance - 1.0 / threshold) * (direction / distance)
        return force
    else:
        return np.zeros_like(pos_self)

# === Variables globales ===
firstCall = True
a_visiter = [0, 0]  # index de point à visiter par chaque camion
Plein = [True] * 4
nbr_aller_retour = [0] * 4


def formation(t, robotNo, robots_poses):
    global firstCall, a_visiter, Plein, nbr_aller_retour

    N = robots_poses.shape[0]
    i = robotNo
    x = robots_poses[:, :2]
    ui = np.zeros(2)

    kp = 0.1
    kp2 = 0.2

    visits = [
        np.array([[-6, 6], [6, 6], [6, -6], [-6, -6]]),
        np.array([[4, 4], [4, -4], [-4, -4], [-4, 4]])
    ]
    origin_drones = np.array([[-10, -10], [-10, -6], [10, -10], [10, -6]])

    if firstCall:
        print("=== Contrôle multi-agent : 2 camions, 4 drones + répulsion ===")
        firstCall = False

    # Camions
    if i in [0, 1]:
        if np.linalg.norm(x[i] - visits[i][a_visiter[i]]) < 0.2:
            a_visiter[i] = (a_visiter[i] + 1) % 4
        ui = -kp * (x[i] - visits[i][a_visiter[i]])

    # Drones
    elif i >= 2:
        drone_id = i - 2
        if nbr_aller_retour[drone_id] >= 3:
            return 0.0, 0.0

        camion_id = drone_id % 2
        drone_indices = [j for j in range(4) if j % 2 == camion_id]
        global_indices = [j + 2 for j in drone_indices]
        distances = [np.linalg.norm(x[j] - x[camion_id]) for j in global_indices]
        closest = global_indices[np.argmin(distances)]
        closest_base = origin_drones[drone_id]

        if i == closest:
            if Plein[drone_id]:
                if np.linalg.norm(x[i] - x[camion_id]) < 0.2:
                    Plein[drone_id] = False
            else:
                if np.linalg.norm(x[i] - closest_base) < 0.2:
                    Plein[drone_id] = True
                    nbr_aller_retour[drone_id] += 1

            if Plein[drone_id]:
                ui = -kp2 * (x[i] - x[camion_id])
            else:
                ui = -kp2 * (x[i] - closest_base)
        else:
            ui = -kp2 * (x[i] - closest_base)

        # Ajout répulsion avec autres drones
        for j in range(2, N):
            if j != i:
                ui += repulsion(x[i], x[j])

    return ui[0], ui[1]
