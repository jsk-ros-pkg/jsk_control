#!/usr/bin/env python
from __future__ import print_function

import matplotlib.pyplot as plt
import numpy as np
import csv
import sys
import os
from itertools import chain
from math import pi
import argparse

parser = argparse.ArgumentParser(description="Plot benchmark result csv")
parser.add_argument("csv_file", help="csv file")
parser.add_argument("--image-suffix", default="eps", help="suffix to save image")
parser.add_argument("--only-save-image", action="store_true", help="die right after saving image")
parser.add_argument("--min", default=0, type=float, help="minimum value")
parser.add_argument("--max", default=0.2, type=float, help="maximum value")
args = parser.parse_args()
csv_file = args.csv_file
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

    minz = args.min
    maxz = args.max

    img = np.arange(0, len(xs) * len(ys), 1.0).reshape((len(xs), len(ys)))
    for x, i in zip(xs, range(len(xs))):
        for y, j in zip(ys, range(len(ys))):
            try:
                z = zs[(x, y)]
            except:
                print("Failed to find (%f, %f)" % (x, y), file=sys.stderr)
            img[j][i] = z
    #im = ax.pcolor(img, vmin=minz, vmax=maxz, cmap="gnuplot")
    im = ax.imshow(img, vmin=minz, vmax=maxz, cmap="gnuplot")
    ax.set_xticklabels(np.arange(-dx - 1, dx+1))
    ax.set_yticklabels(np.arange(-dy - 1, dy+1))
    ax.set_xlabel("$x$ [m]")
    ax.set_ylabel("$y$ [m]")
    ax.set_title('$\\theta = %.0f$ deg' % (theta / pi * 180.0))


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
    if theta_str not in data:
        data[theta_str] = []
    data[theta_str].append((float(row[fields.index("x")]), float(row[fields.index("y")]), float(row[fields.index("one_time")])))

counter = 0
#for theta, one_data in data.items():
for theta in sorted(data.keys()):
    one_data = data[theta]
    print("Plotting theta=", theta)
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
# save figure
eps_file = os.path.basename(csv_file) + "." + args.image_suffix
print("Saving to %s file: " % (args.image_suffix), eps_file)
plt.savefig(eps_file)
if args.only_save_image:
    sys.exit(0)
while True:
    plt.pause(1)
