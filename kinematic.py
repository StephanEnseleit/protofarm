
# TCP - Tool centere point, = input coordinates
#-->

# TOP . Tool origin point, TCP + Tool Offset
#-->

# MD - Maschine Definition, 
#--> 

# UVW - Calculation (in mm)
#-->

# Action Code (Values for stepps.)

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

### Maschinenparameter ###
# Höhe der unteren Rolle vom Ursprung 
h = 300 # [mm]
# Abstand des Seil Drehpunktes vom Ursprung
a = 400 

def inverse_kinematics(x, y, z):

    # Berechnung der Verbindungspunkte zwischen Seilpunkten und TCP UT(X,Y,Z), VT(X,Y,Z), WT(X,Y,Z)
    
    
    # Berechnung der Gelenkwinkel für die gewünschte Endeffektorposition (x, y, z)
    r = np.sqrt(x**2 + y**2)
    theta1 = np.arctan2(y, x)
    D = (r**2 + (z - l1)**2 - l2**2 - l3 **2) / (2 * l2 * l3)
    
    # Begrenzung für D
    D = np.clip(D, -1, 1)
    
    theta3 = np.arccos(D)
    theta2 = np.arctan2(z - l1, r) - np.arctan2(l3 * np.sin(theta3), l2 + l3 * np.cos(theta3))
    
    return np.degrees(theta1), np.degrees(theta2), np.degrees(theta3)

def update(frame):
    ax.cla()

    end_effector_pos = np.array([2 * np.cos(np.radians(frame)), 2 * np.sin(np.radians(frame)), np.sin(np.radians(frame)) + 1])

    # Längen der Seile
    l1, l2, l3 = 1, 1, 1

    # Inverse Kinematik
    theta1, theta2, theta3 = inverse_kinematics(end_effector_pos[0], end_effector_pos[1], end_effector_pos[2], l1, l2, l3)

    # Darstellung des 3D-Seilroboters
    ax.plot([0, l1 * np.cos(np.radians(theta1))], [0, l1 * np.sin(np.radians(theta1))], [0, l1], color='b', linewidth=2, marker='o')
    ax.plot([l1 * np.cos(np.radians(theta1)), l1 * np.cos(np.radians(theta1)) + l2 * np.sin(np.radians(theta2))],
            [l1 * np.sin(np.radians(theta1)), l1 * np.sin(np.radians(theta1)) - l2 * np.cos(np.radians(theta2))],
            [l1, l1 + l2 * np.sin(np.radians(theta2))], color='r', linewidth=2, marker='o')
    ax.plot([l1 * np.cos(np.radians(theta1)) + l2 * np.sin(np.radians(theta2)), end_effector_pos[0]],
            [l1 * np.sin(np.radians(theta1)) - l2 * np.cos(np.radians(theta2)), end_effector_pos[1]],
            [l1 + l2 * np.sin(np.radians(theta2)), end_effector_pos[2]], color='g', linewidth=2, marker='o')

    ax.scatter(*end_effector_pos, c='r', marker='o', s=100)

    ax.set_xlim([-3, 3])
    ax.set_ylim([-3, 3])
    ax.set_zlim([0, 3])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Cable-Driven Robot')

# Erstellung der 3D-Figur und Achsen
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Erstellung der Animation
animation = FuncAnimation(fig, update, frames=np.arange(0, 361, 1), interval=50)

# Anzeige der Animation
plt.show()
