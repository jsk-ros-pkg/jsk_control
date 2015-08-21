#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import csv
import sys
from itertools import chain
from math import pi

csv_file = sys.argv[1]
methods = ['none']
dx = 3
dy = 3

def plot(theta, one_data, ax):
    global im
    xs = [data[0] for data in one_data]
    ys = [data[1] for data in one_data]
    zs = {}
    for x, y, i in zip(xs, ys, range(len(xs))):
        zs[(x, y)] = one_data[i][2]
    xs.sort()
    ys.sort()

    minz = 0
    maxz = 0.2

    img = np.arange(0, len(xs) * len(ys), 1.0).reshape((len(xs), len(ys)))
    for x, i in zip(xs, range(len(xs))):
        for y, j in zip(ys, range(len(ys))):
            z = zs[(x, y)]
            img[j][i] = z
    #im = ax.pcolor(img, vmin=minz, vmax=maxz, cmap="gnuplot")
    im = ax.imshow(img, vmin=minz, vmax=maxz, cmap="gnuplot")
    ax.set_xticklabels(np.arange(-dx - 1, dx+1))
    ax.set_yticklabels(np.arange(-dy - 1, dy+1))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title('$\\theta$ = %f deg' % (theta / pi * 180.0))


reader = csv.reader(open(csv_file))
fields = reader.next()

initialized = False

data = {}
# theta -> x, y, time

for row in reader:
    # Initialization
    if not initialized:
        figure_num = int(row[fields.index("n_theta")])
        if figure_num % 3 == 0:
            xnum = figure_num / 3
        else:
            xnum = figure_num / 3 + 1
        fig, axes = plt.subplots(xnum, 3)
        initialized = True
    theta_str = row[fields.index("theta")]
    theta = float(theta_str)
    if not data.has_key(theta_str):
        data[theta_str] = []
    data[theta_str].append((float(row[fields.index("x")]), float(row[fields.index("y")]), float(row[fields.index("one_time")])))

counter = 0
for theta, one_data in data.items():
    print "Plotting theta=", theta
    plot(float(theta), one_data, axes.flat[counter])
    counter = counter + 1
    
for a in axes.flat[counter:]:
    fig.delaxes(a)
plt.tight_layout()
fig.subplots_adjust(right = 0.8)
cbar_ax = fig.add_axes([0.85, 0.15, 0.05, 0.7])
cb = fig.colorbar(im, cax=cbar_ax)
cb.set_label("Time [sec]")
plt.interactive(True)
plt.show()
while True:
    plt.pause(1)
