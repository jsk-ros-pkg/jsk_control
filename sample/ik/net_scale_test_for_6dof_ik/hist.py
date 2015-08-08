#!/usr/bin/env python

import matplotlib.pyplot as plt
from pylab import *
import numpy
import random
import os
import scipy.stats.distributions as dis
import functools

def gen_time_buf(path):
    f = open(path, 'r')
    read_flag=False
    time_buf=[]
    line = f.readline()
    while line:
        if ":raw" in line:
            read_flag=True
        elif ":" in line:
            read_flag=False
        elif read_flag:
        ## print line
            time_buf.append( (1e6 * float(line.split(" ")[0])) )
        line = f.readline()
    f.close()
    return time_buf

def gen_pos_buf(x, path):
    f = open(path, 'r')
    read_flag=False
    time_buf=[]
    line = f.readline()
    while line:
        if ":raw" in line:
            read_flag=True
        elif ":" in line:
            read_flag=False
        elif read_flag:
        ## print line
            time_buf.append( float(line.split(" ")[1+x]) )
        line = f.readline()
    f.close()
    return time_buf

def gen_rot_buf(x, path):
    f = open(path, 'r')
    read_flag=False
    time_buf=[]
    line = f.readline()
    while line:
        if ":raw" in line:
            read_flag=True
        elif ":" in line:
            read_flag=False
        elif read_flag:
        ## print line
            time_buf.append( 180 / 3.14 * float(line.split(" ")[4+x]) )
        line = f.readline()
    f.close()
    return time_buf


def gen_graph(ls, rng, gen_data, title="histgram", xlabel="dif [m]", labels=None, logpath="log.eps"):
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel("histgram")
    plt.grid(True)
    if not labels:
        labels = ls
## fig = plt.figure()
## ax = fig.add_subplot(111)
## ls = os.listdir(".")
## ls.sort()
## ls = ["analysis_5x400.log.train", "analysis_5x200.log.train", "analysis_5x100.log.train"]
    ## rng=[0,0.5]
    markers=[',', '+', '.', 'o', '*']
    i = 0
    for p in ls:
        if "analysis" in p and "log" in p and not "x50" in p:
            time_buf = gen_data(p)
            ## time_buf_fit = dis.norm.fit(time_buf)
            weights = np.ones_like(time_buf) ##/len(time_buf)
            plt.hist(time_buf, weights=weights, bins=100, label=labels[i], range=rng, normed=True, alpha=0.2)
            ##
            loc=float(np.mean(time_buf))
            scale=float(np.std(time_buf))
            x = np.arange(rng[0], rng[1], (rng[1] - rng[0])/1000.0)
            plt.plot(x, dis.norm.pdf(x, loc=loc, scale=scale), label=(labels[i]+".nd (" + (r'''$\mu=%.1f$, $\sigma=%.2f$''' % (loc, scale)) + ")"), marker=markers[(i % len(markers))])
            i = i+1
    plt.legend()
    plt.savefig(logpath)
    plt.show()

params = {'backend': 'ps',
          'axes.labelsize': 20,
          'text.fontsize': 20,
          'legend.fontsize': 20,
          'xtick.labelsize': 15,
          'ytick.labelsize': 15,
          ## "figure.subplot.right": 0.8,
          ## "figure.subplot.top": 0.95,
          ## "figure.subplot.left": 0.05,
          ## "figure.subplot.bottom": 0.05,
          'figure.figsize': [20, 10],
          'ps.useafm': True,
          'pdf.use14corefonts': True
          }
plt.rcParams.update(params)

ls=["analysis_5x400.log.test", "analysis_5x200.log.test", "analysis_5x100.log.test"]
lb=["mid100", "mid200", "mid400"]
gen_graph(ls, [50, 500], gen_time_buf, xlabel='time [micro sec]', title="_", labels=lb, logpath="hist_time_wide.eps")
for i in [0, 1, 2]:
    print "diff pos["+str(i)+"] histgram"
    gen_graph(ls, [-300, 300], functools.partial(gen_pos_buf, i), xlabel='dif [mm]', title="_", labels=lb, logpath=("hist_dpos" + str(i) + "_wide.eps"))
for i in [0, 1, 2]:
    print "diff rpy[" + str(i) + "] histgram"
    gen_graph(ls, [-100, 100], functools.partial(gen_rot_buf, i), xlabel='dif [rad]', title="_", labels=lb, logpath=("hist_drpy" + str(i) + "_wide.eps"))

gen_graph(["analysis_5x200.log.test", "analysis_4x200.log.test", "analysis_3x200.log.test"], [50, 500], gen_time_buf, xlabel='time [micro sec]', title="_", labels=["5layer", "4layer", "3layer"], logpath="hist_time_depth.eps")
for i in [0, 1, 2]:
    print "diff pos["+str(i)+"] histgram"
    gen_graph(["analysis_5x200.log.test", "analysis_4x200.log.test", "analysis_3x200.log.test"], [-300, 300], functools.partial(gen_pos_buf, i), xlabel='dif [mm]', title="_", labels=["5layer", "4layer", "3layer"], logpath=("hist_dpos" + str(i) + "_depth.eps"))
for i in [0, 1, 2]:
    print "diff rpy[" + str(i) + "] histgram"
    gen_graph(["analysis_5x200.log.test", "analysis_4x200.log.test", "analysis_3x200.log.test"], [-100, 100], functools.partial(gen_rot_buf, i), xlabel='dif [rad]', title="_", labels=["5layer", "4layer", "3layer"], logpath=("hist_drpy" + str(i) + "_depth.eps"))
