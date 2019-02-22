#!/usr/bin/env python

from argparse import ArgumentParser
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


def plot_data(ax,
              title,
              ylabel,
              data_filename=None,
              time_list=None):
    if data_filename is None:
        data_filename =title+'.dat'
    data = np.genfromtxt(args.data_dirname+'/'+data_filename,
                         dtype=np.float32, delimiter=' ', names=True)
    t = data['time']
    for name in data.dtype.names[1:]:
        d = data[name]
        ax.plot(t, d, linewidth=2, label=name)
        ax.set_ylabel(ylabel)

    ax.set_title(title, fontsize=18)
    ax.set_xlabel('time')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=4)
    ax.set_xlim(t[0], t[-1])
    ax.grid(True)

    if time_list is not None:
        for t in time_list:
            ax.axvline(x=t, linewidth=0.8, color='k', linestyle='--')


if __name__ == '__main__':
    global args

    parser = ArgumentParser()

    parser.add_argument('--data-dirname', type=str, dest="data_dirname", required=True)
    parser.add_argument('--output-filename', type=str, dest="output_filename", required=True)

    parser.add_argument('--kin-task-time-list', nargs="*", type=float, dest="kin_task_time_list", default=None)
    parser.add_argument('--eom-task-time-list', nargs="*", type=float, dest="eom_task_time_list", default=None)
    parser.add_argument('--centroid-task-time-list', nargs="*", type=float, dest="centroid_task_time_list", default=None)
    parser.add_argument('--posture-task-time-list', nargs="*", type=float, dest="posture_task_time_list", default=None)

    args = parser.parse_args()

    fig,axes = plt.subplots(nrows=10,ncols=1,figsize=(10,100))

    plot_data(axes[0], title='theta', ylabel='joint position [rad] [m]')
    plot_data(axes[1], title='cog', ylabel='position [m]')
    plot_data(axes[2], title='angular_momentum', ylabel='angular momentum [kg m^2 / s]')
    plot_data(axes[3], title='wrench', ylabel='force [N] / moment [Nm]')

    plot_data(axes[4], title="kinematics_task", ylabel='position [m] / orientation [rad]', time_list=args.kin_task_time_list)
    plot_data(axes[5], title="translational_eom_task", ylabel='force [N]', time_list=args.eom_task_time_list)
    plot_data(axes[6], title="rotational_eom_task", ylabel='moment [Nm]', time_list=args.eom_task_time_list)
    plot_data(axes[7], title="cog_task", ylabel='position [m]', time_list=args.centroid_task_time_list)
    plot_data(axes[8], title="angular_momentum_task", ylabel='angular momentum [kg m^2 / s]', time_list=args.centroid_task_time_list)
    plot_data(axes[9], title="posture_task", ylabel='joint position [rad] [m]', time_list=args.posture_task_time_list)

    plt.tight_layout()
    fig.show()

    pdf = PdfPages(args.output_filename)
    pdf.savefig()
    pdf.close()
