#!/usr/bin/python3
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

x = []
y = []
vmax = 1000
plt.xlim(-vmax-1, vmax+1)
plt.ylim(-vmax-1, vmax+1)
plt.gca().set_aspect('equal', adjustable='box')
plt.axhline()
plt.axvline()
ax = plt.gca()
for v in np.arange(0, vmax, 100):
    for w in np.arange(0, np.pi, 0.2):
        if np.fabs(w) > 0.01:
            r = v/w
            xc = r
            yc = 0
            x.append(r - r * np.cos(w))
            y.append(r * np.sin(w))
            pac = mpatches.Arc([r, 0], 2*r, 2*r, 0, theta1=180-np.rad2deg(w), theta2=180)
            ax.add_patch(pac)
        else:
            x.append(0)
            y.append(v)

fx = np.array(x)
fy = np.array(y)
plt.scatter(fx, fy)
plt.show()
