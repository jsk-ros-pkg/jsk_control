#!/usr/bin/env python

"""
Visualize csv file of ik-grid in 2d heatmap
"""
import matplotlib.pyplot as plt
import sys
import csv
from itertools import chain
import numpy as np
import math

csvfile = sys.argv[1]
if len(sys.argv) == 3:
    image_file = sys.argv[2]
else:
    image_file = None


data_by_k = dict()
# read data from csv file
with open(csvfile) as f:
    reader = csv.reader(f)
    index = reader.next()
    # x, y, z, i, j, k. value
    x_index = index.index("x")
    y_index = index.index("y")
    z_index = index.index("z")
    i_index = index.index("i")
    j_index = index.index("j")
    k_index = index.index("k")
    value_index = index.index("value")
    for row in reader:
        (x, y, z, i, j, k, value) = [float(row[x_index]),
                                     float(row[y_index]),
                                     float(row[z_index]),
                                     int(row[i_index]),
                                     int(row[j_index]),
                                     int(row[k_index]),
                                     float(row[value_index])]
        data = {"x": x,
                "y": y,
                "z": z,
                "i": i,
                "j": j,
                "k": k,
                "value": value}
        if k in data_by_k:
            data_by_k[k].append(data)
        else:
            data_by_k[k] = [data]
k_num = len(data_by_k.keys())

# print "{0} k values".format(k_num)

# Estimating grid size
z_values = np.array(list(chain.from_iterable([[data["z"] for data in data_array]
                                              for data_array in data_by_k.values()])))
min_z = np.amin(z_values)
max_z = np.amax(z_values)
# print "z value from {0} to {1}".format(min_z, max_z)
step = (max_z - min_z) / (k_num - 1)
# print "estimated step is", step

fig, axes = plt.subplots(int(math.ceil(k_num / 3.0)), 3)
images = dict()
non_zero_counter = 0
for k, counter in zip(data_by_k, range(k_num)):             # need to sort by k?
    data_array = data_by_k[k]
    i_s = sorted(set([data["i"] for data in data_array]))
    j_s = sorted(set([data["j"] for data in data_array]))
    xnum = len(i_s)
    ynum = len(j_s)
    min_i = min([data["i"] for data in data_array])
    min_j = min([data["j"] for data in data_array])
    min_value = min([data["value"] for data in data_array])
    max_value = max([data["value"] for data in data_array])
    min_x = min([data["x"] for data in data_array])
    max_x = max([data["x"] for data in data_array])
    min_y = min([data["y"] for data in data_array])
    max_y = max([data["y"] for data in data_array])
    # print "{0}-{1}".format(xnum, ynum)
    image = np.arange(0, xnum * ynum, 1.0).reshape((xnum, ynum))
    for data in data_array:
        i = data["i"]
        j = data["j"]
        image[i - min_i][j - min_j] = data["value"]
        if data["value"] != 0:
            non_zero_counter = non_zero_counter + 1
    images[k] = image
    ax = axes.flat[counter]
    # print "x: {0} - {1}".format(min_x, max_x)
    # print "y: {0} - {1}".format(min_y, max_y)
    # ax.set_xticklabels([i  * istep + min_x for i in range(xnum-1)])
    ax.set_title("$z = {0}$mm".format(data_array[0]["z"]))
    ax.set_xlabel("$x$ [mm]")
    ax.set_ylabel("$y$ [mm]")
    ax.tick_params(labelsize=8)
    im = ax.imshow(image, vmin=min_value, vmax=max_value, cmap="gnuplot", interpolation="none",
                   aspect="auto",
                   extent=[min_x-step/2, max_x+step/2, min_y-step/2, max_y+step/2])
# cbar_ax = fig.add_axes([0.95, 0.15, 0.01, 0.7])
# cb = fig.colorbar(im, cax=cbar_ax)
print("{0} reached regions".format(non_zero_counter))
fig.suptitle("{0} reached regions".format(non_zero_counter), fontsize=8)
plt.tight_layout()

if image_file:
    plt.savefig(image_file)
else:
    plt.show()
