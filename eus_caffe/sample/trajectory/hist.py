#!/usr/bin/env python

import matplotlib.pyplot as plt
from pylab import *
import numpy
import random
import os
import scipy.stats.distributions as dis
import functools

def gen_scaled_data_buf(x, scale, path):
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
            time_buf.append( (scale * float(line.split(" ")[x])) )
        line = f.readline()
    f.close()
    return time_buf

def calc_pdf(xl, loc=0, scale=1):
    ret=[]
    for _x in xl:
        x = (_x - loc) / scale
        y = exp(-x**2/2)/sqrt(2*3.14)/scale
        ret.append(y)
    return ret;

def calc_logistic(xl, loc=0, scale=1):
    ret=[]
    ## s = sqrt(3*scale/3.14/3.14)
    s = sqrt(3/3.14/3.14) * scale
    ## s = scale
    for _x in xl:
        x = (_x - loc) / s
        ex = exp(-x)
        y = ex/s/(1+ex)/(1+ex)
        ret.append(y)
    return ret;

def calc_lap(xl, loc=0, scale=1):
    ret=[]
    ## phi = sqrt(scale/2)
    phi = scale/2
    for _x in xl:
        x = abs( (_x - loc) / phi )
        y = exp(-x)/2/phi
        ret.append(y)
    return ret;

def calc_cosh(xl, loc=0, scale=1):
    ret=[]
    for _x in xl:
        x = (_x - loc) / scale
        y = 1.0/(exp(3.14/2*x) + exp(-3.14/2*x))/scale
        ret.append(y)
    return ret;

def gen_graph(ls, rng, gen_data, title="histgram", xlabel="dif [m]", labels=None, logpath="log.eps", draw_np=True):
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(".")
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
    ## time_buf_buf=[]
    for p in ls:
        if True: ## "analysis" in p and "log" in p and not "x50" in p:
            if isinstance(gen_data, list) or isinstance(gen_data, tuple):
                time_buf = (gen_data[(i % 2)])(p)
            else:
                time_buf = gen_data(p)
            ## time_buf_fit = dis.norm.fit(time_buf)
            weights = np.ones_like(time_buf) ##/len(time_buf)
            plt.hist(time_buf, weights=weights, bins=100, label=labels[i], range=rng, normed=True, alpha=0.3 , histtype="bar")
            ## time_buf_buf.append(time_buf)
            ##
            loc=float(np.mean(time_buf))
            scale=float(np.std(time_buf))
            phi = scale/2
            x = np.arange(rng[0], rng[1], (rng[1] - rng[0])/1000.0)
            if draw_np:
                plt.plot(x, calc_lap(x, loc=loc, scale=scale), label=("Laplace" + "\n" + ('     $\mu=%.1f$\n     $\phi=%.2f$' % (loc, phi))), marker=markers[(i % len(markers))])
            i = i+1
            ##plt.hist(time_buf_buf, bins=100, label=labels, range=rng, normed=True)
    plt.legend()
    plt.savefig(logpath, transparent=True)
    plt.show()

params = {'backend': 'ps',
          'axes.labelsize': 20,
          'axes.unicode_minus': False,
          'text.fontsize': 20,
          'legend.fontsize': 20,
          'xtick.labelsize': 15,
          'ytick.labelsize': 15,
          "figure.subplot.right": 0.96,
          "figure.subplot.top": 0.96,
          "figure.subplot.left": 0.12,
          "figure.subplot.bottom": 0.15,
          'figure.figsize': [7, 4.5],
          'ps.useafm': True,
          'pdf.use14corefonts': True,
          'text.usetex': True
          }
plt.rcParams.update(params)


ls=["redundancy_traj_search_analysis.log"]
gen_graph(ls, [50, 150], functools.partial(gen_scaled_data_buf, 0, 1000.0), xlabel='time [micro sec]', title=".", labels=["Time [sec]"], logpath="hist_time.pdf", draw_np=False)
for i in [0, 1, 2]:
    print "diff pos["+str(i)+"] histgram"
    gen_graph(ls, [-40, 40], functools.partial(gen_scaled_data_buf, (1+i), 1.0), xlabel='dif [mm]', title="_", labels=[("diff pos["+str(i)+"]")], logpath=("hist_dpos" + str(i) + ".pdf"))
for i in [0, 1, 2, 3]:
    print "diff q[" + str(i) + "] histgram"
    gen_graph(ls, [-40, 40], functools.partial(gen_scaled_data_buf, (4+i), (180.0/3.14)), xlabel='dif [deg]', title="_", labels=[("diff q[" + str(i) + "]")], logpath=("hist_dq" + str(i) + ".pdf"))

