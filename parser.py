"""Parser.

Notes:
    Main executable script.

Run as:

    $ python3 parser.py --test_arg <test_arg>

"""
import argparse
import numpy as np
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

        for num in range(1,11):

            # real cmd
            first_cmd_time = None
            last_cmd_time = None
            with open('./exp/cmd_test_' + sol + str(num) + '.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                real_ref_time = []
                real_ref_x = []
                real_ref_y = []
                real_ref_z = []
                for i, row in enumerate(csv_reader):
                    if i == 0:
                        continue
                    elif i == 1:
                        real_ref_init_time = float(row[0])
                        first_cmd_time = real_ref_init_time
                    real_ref_time.append(float(row[0])-real_ref_init_time)
                    real_ref_x.append(float(row[2]))
                    real_ref_y.append(float(row[3]))
                    real_ref_z.append(float(row[4]))
                last_cmd_time = real_ref_time[-1]+real_ref_init_time
                axs[0, 0].plot(real_ref_time, real_ref_x, label='ref')
                axs[1, 0].plot(real_ref_time, real_ref_y, label='ref')
                axs[2, 0].plot(real_ref_time, real_ref_z, label='ref')

            # real vicon
            with open('./exp/' + sol + str(num) + '.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                real_time = []
                real_x = []
                real_y = []
                real_z = []
                for i, row in enumerate(csv_reader):
                    if i == 0:
                        continue
                    else:
                        t = float(row[0])
                        # if i == 1:
                        #     init_time = t
                        if t >= first_cmd_time and t <= last_cmd_time:
                            real_time.append(t-real_ref_init_time)
                            real_x.append(float(row[5]))
                            real_y.append(float(row[6]))
                            real_z.append(float(row[7]))
                axs[0, 1].plot(real_time, real_x, label='exp')
                axs[1, 1].plot(real_time, real_y, label='exp')
                axs[2, 1].plot(real_time, real_z, label='exp')

            # sim cmd
            with open('./sim/' + sol + '/episode' + str(num) + '.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                sim_ref_time = []
                sim_ref_x = []
                sim_ref_y = []
                sim_ref_z = []
                for i, row in enumerate(csv_reader):
                    if i == 0:
                        continue
                    sim_ref_time.append(float(row[0]))
                    sim_ref_x.append(float(row[2]))
                    sim_ref_y.append(float(row[3]))
                    sim_ref_z.append(float(row[4]))
                axs[0, 2].plot(sim_ref_time, sim_ref_x, label='ref')
                axs[1, 2].plot(sim_ref_time, sim_ref_y, label='ref')
                axs[2, 2].plot(sim_ref_time, sim_ref_z, label='ref')

            # sim pos
            folder = './sim/' + sol + '/ep' + str(num)
            with open(folder + '/x0.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                sim_x_time = []
                sim_x = []
                for i, row in enumerate(csv_reader):
                    sim_x_time.append(float(row[0]))
                    sim_x.append(float(row[1]))
            with open(folder + '/y0.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                sim_y_time = []
                sim_y = []
                for i, row in enumerate(csv_reader):
                    sim_y_time.append(float(row[0]))
                    sim_y.append(float(row[1]))
            with open(folder + '/z0.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                sim_z_time = []
                sim_z = []
                for i, row in enumerate(csv_reader):
                    sim_z_time.append(float(row[0]))
                    sim_z.append(float(row[1]))
            axs[0, 3].plot(sim_x_time, sim_x, label='sim')
            axs[1, 3].plot(sim_y_time, sim_y, label='sim')
            axs[2, 3].plot(sim_z_time, sim_z, label='sim')

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
