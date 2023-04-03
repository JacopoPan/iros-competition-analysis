"""Parser.

Notes:
    Main executable script.

Run as:

    $ python3 parser.py --test_arg <test_arg>

"""
import io
import argparse
import numpy as np
import pandas as pd
from scipy.interpolate import interp1d   
import traces
import datetime
from datetime import timedelta
import bagpy
from bagpy import bagreader
import pickle
import csv
import os
import re
import matplotlib.pyplot as plt


def run(
        test_arg,
    ):
    for sol in ['arg', 'eku', 'h2']:
        fig, axs = plt.subplots(3, 4)

        real_ref_time = [[] for i in range(10)]
        real_ref_x = [[] for i in range(10)]
        real_ref_y = [[] for i in range(10)]
        real_ref_z = [[] for i in range(10)]

        real_time = [[] for i in range(10)]
        real_x = [[] for i in range(10)]
        real_y = [[] for i in range(10)]
        real_z = [[] for i in range(10)]
        
        sim_time = [[] for i in range(10)]

        sim_ref_x = [[] for i in range(10)]
        sim_ref_y = [[] for i in range(10)]
        sim_ref_z = [[] for i in range(10)]

        sim_x = [[] for i in range(10)]
        sim_y = [[] for i in range(10)]
        sim_z = [[] for i in range(10)]

        for num in range(1,11):
            idx = num-1

            # real cmd
            first_cmd_time = None
            last_cmd_time = None
            with open('./exp/cmd_test_' + sol + str(num) + '.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for i, row in enumerate(csv_reader):
                    if i == 0: continue
                    if i == 1:
                        real_ref_init_time = float(row[0])
                        first_cmd_time = real_ref_init_time
                    real_ref_time[idx].append(float(row[0])-real_ref_init_time)
                    if row[1] == 'takeoff' or row[1] == 'none':
                        pass
                    real_ref_x[idx].append(float(row[2]))
                    real_ref_y[idx].append(float(row[3]))
                    real_ref_z[idx].append(float(row[4]))
                last_cmd_time = real_ref_time[idx][-1]+real_ref_init_time

            # real vicon
            with open('./exp/' + sol + str(num) + '.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for i, row in enumerate(csv_reader):
                    if i == 0: continue
                    t = float(row[0])
                    if t >= first_cmd_time and t <= last_cmd_time:
                        real_time[idx].append(t-real_ref_init_time)
                        real_x[idx].append(float(row[5]))
                        real_y[idx].append(float(row[6]))
                        real_z[idx].append(float(row[7]))

            # sim cmd
            sim_ref_time = []
            with open('./sim/' + sol + '/episode' + str(num) + '.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for i, row in enumerate(csv_reader):
                    if i == 0: continue
                    sim_ref_time.append(float(row[0]))
                    if row[1] == 'takeoff' or row[1] == 'none':
                        pass
                    sim_ref_x[idx].append(float(row[2]))
                    sim_ref_y[idx].append(float(row[3]))
                    sim_ref_z[idx].append(float(row[4]))

            # sim pos
            folder = './sim/' + sol + '/ep' + str(num)
            sim_x_time = []
            sim_y_time = []
            sim_z_time = []
            with open(folder + '/x0.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for i, row in enumerate(csv_reader):
                    sim_x_time.append(float(row[0]))
                    sim_x[idx].append(float(row[1]))
            with open(folder + '/y0.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for i, row in enumerate(csv_reader):
                    sim_y_time.append(float(row[0]))
                    sim_y[idx].append(float(row[1]))
            with open(folder + '/z0.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for i, row in enumerate(csv_reader):
                    sim_z_time.append(float(row[0]))
                    sim_z[idx].append(float(row[1]))
            if sim_x_time != sim_y_time or sim_x_time != sim_z_time or sim_x_time != sim_ref_time:
                print('Error: time mismatch')
                exit()
            sim_time[idx] = sim_x_time

            # math magic
            time = np.array(real_time[idx])
            values = np.array(real_x[idx])
            func = interp1d(time, values, kind='linear')
            t_new = np.linspace(
                max( min(sim_time[idx]), min(real_time[idx]) ), # REDO: min val in sim_time that is greater than min val in real_time
                min( max(sim_time[idx]), max(real_time[idx]) ), # REDO: max val in sim_time that is less than max val in real_time
                len(sim_time[idx])
                )
            v_new = func(t_new)
            print(t_new, v_new)
            exit()

            # plot one-by-one
            axs[0, 0].plot(real_ref_time[idx], real_ref_x[idx], label='ref')
            axs[1, 0].plot(real_ref_time[idx], real_ref_y[idx], label='ref')
            axs[2, 0].plot(real_ref_time[idx], real_ref_z[idx], label='ref')

            axs[0, 1].plot(real_time[idx], real_x[idx], label='exp')
            axs[1, 1].plot(real_time[idx], real_y[idx], label='exp')
            axs[2, 1].plot(real_time[idx], real_z[idx], label='exp')

            axs[0, 2].plot(sim_time[idx], sim_ref_x[idx], label='ref')
            axs[1, 2].plot(sim_time[idx], sim_ref_y[idx], label='ref')
            axs[2, 2].plot(sim_time[idx], sim_ref_z[idx], label='ref')  

            axs[0, 3].plot(sim_time[idx], sim_x[idx], label='sim')
            axs[1, 3].plot(sim_time[idx], sim_y[idx], label='sim')
            axs[2, 3].plot(sim_time[idx], sim_z[idx], label='sim')

            # comparing time sampling
            print(sim_time[idx][0], sim_time[idx][-1], len(sim_time[idx]))
            print(real_ref_time[idx][0], real_ref_time[idx][-1], len(real_ref_time[idx]))
            print(real_time[idx][0], real_time[idx][-1], len(real_time[idx]))

        # labels
        fig.suptitle(sol)
        axs[2, 0].set_xlabel('time')
        axs[2, 1].set_xlabel('time')
        axs[2, 2].set_xlabel('time')
        axs[2, 3].set_xlabel('time')

        axs[0, 0].set_ylabel('x')
        axs[1, 0].set_ylabel('y')
        axs[2, 0].set_ylabel('z')

        axs[0, 0].set_title('real_cmd')
        axs[0, 1].set_title('real')
        axs[0, 2].set_title('sim_cmd')
        axs[0, 3].set_title('sim')

        # axes limits
        axs[0, 0].set_ylim(-3.5,3.5)
        axs[0, 1].set_ylim(-3.5,3.5)
        axs[0, 2].set_ylim(-3.5,3.5)
        axs[0, 3].set_ylim(-3.5,3.5)

        axs[1, 0].set_ylim(-3.5,3.5)
        axs[1, 1].set_ylim(-3.5,3.5)
        axs[1, 2].set_ylim(-3.5,3.5)
        axs[1, 3].set_ylim(-3.5,3.5)

        axs[2, 0].set_ylim(-3.5,3.5)
        axs[2, 1].set_ylim(-3.5,3.5)
        axs[2, 2].set_ylim(-3.5,3.5)
        axs[2, 3].set_ylim(-3.5,3.5)
        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Parsing script')
    parser.add_argument('--test_arg',
                        default='',
                        type=str,
                        help='Test argument', metavar='')
    ARGS = parser.parse_args()
    run(**vars(ARGS))
