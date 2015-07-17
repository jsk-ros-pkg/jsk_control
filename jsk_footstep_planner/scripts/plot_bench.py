#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import csv
import sys
from itertools import chain
from math import pi
# from the docs:

# If interpolation is None, default to rc image.interpolation. See also
# the filternorm and filterrad parameters. If interpolation is 'none', then
# no interpolation is performed on the Agg, ps and pdf backends. Other
# backends will fall back to 'nearest'.
#
# http://matplotlib.org/api/pyplot_api.html#matplotlib.pyplot.imshow
csv_files = sys.argv[1:]
# methods = [None, 'none', 'nearest', 'bilinear', 'bicubic', 'spline16',
#            'spline36', 'hanning', 'hamming', 'hermite', 'kaiser', 'quadric',
#            'catrom', 'gaussian', 'bessel', 'mitchell', 'sinc', 'lanczos']
methods = ['none']
# grid = np.random.rand(4, 4)

# fig, axes = plt.subplots(3, 6, figsize=(12, 6),
#                          subplot_kw={'xticks': [], 'yticks': []})

dx = 3
dy = 3

def plot(csv_file, ax):
    global im
    reader = csv.reader(open(csv_file))
    xs = []
    ys = []
    ts = []
    zs = dict()
    
    for row in reader:
        if row:
            x = float(row[0])
            y = float(row[1])
            theta = float(row[2])
            z = float(row[3])
            if x not in xs:
                xs.append(x)
            if y not in ys:
                ys.append(y)
            zs[(x, y)] = z
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


if len(csv_files) % 3 == 0:
    xnum = len(csv_files) / 3
else:
    xnum = len(csv_files) / 3 + 1

fig, axes = plt.subplots(xnum, 3)
for csv_file, ax in zip(csv_files, axes.flat):
    plot(csv_file, ax)
for a in axes.flat[len(csv_files):]:
    fig.delaxes(a)
plt.tight_layout()
fig.subplots_adjust(right = 0.8)
cbar_ax = fig.add_axes([0.85, 0.15, 0.05, 0.7])
cb = fig.colorbar(im, cax=cbar_ax)
cb.set_label("Time [sec]")
plt.show()
