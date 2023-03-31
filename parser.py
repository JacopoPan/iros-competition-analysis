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
    # files = os.listdir('./exp-data/')
    # print(files)
    # for file in files:
    #     if re.search('.bag', file): 
    #         print(file)
    #         b = bagreader('./exp-data/' + file)
    #         print(b.topic_table) # get the list of topics
    #         csvfiles = []
    #         for t in b.topics:
    #             data = b.message_by_topic(t)
    #             csvfiles.append(data)
    #         print(csvfiles[0])

    for sol in ['arg', 'eku', 'h2']:
        fig, axs = plt.subplots(3, 2)

        for num in range(1,11):
            with open('./exp/' + sol + str(num) + '.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                time = []
                x = []
                y = []
                z = []
                for i, row in enumerate(csv_reader):
                    if i == 0:
                        continue
                    elif i == 1:
                        init_time = float(row[0])
                    time.append(float(row[0])-init_time)
                    x.append(float(row[5]))
                    y.append(float(row[6]))
                    z.append(float(row[7]))
                axs[0, 0].plot(time, x, label='exp')
                axs[1, 0].plot(time, y, label='exp')
                axs[2, 0].plot(time, z, label='exp')


            #################

            folder = './sim/' + sol + '/ep' + str(num)
            with open(folder + '/x0.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                time = []
                x = []
                for i, row in enumerate(csv_reader):
                    time.append(float(row[0]))
                    x.append(float(row[1]))
            with open(folder + '/y0.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                time = []
                y = []
                for i, row in enumerate(csv_reader):
                    time.append(float(row[0]))
                    y.append(float(row[1]))
            with open(folder + '/z0.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                time = []
                z = []
                for i, row in enumerate(csv_reader):
                    time.append(float(row[0]))
                    z.append(float(row[1]))
            axs[0, 1].plot(time, x, label='sim')
            axs[1, 1].plot(time, y, label='sim')
            axs[2, 1].plot(time, z, label='sim')

            with open('./sim/' + sol + '/episode' + str(num) + '.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                time = []
                ref_x = []
                ref_y = []
                ref_z = []
                for i, row in enumerate(csv_reader):
                    if i == 0:
                        continue
                    time.append(float(row[0]))
                    ref_x.append(float(row[2]))
                    ref_y.append(float(row[3]))
                    ref_z.append(float(row[4]))
                axs[0, 1].plot(time, ref_x, label='ref')
                axs[1, 1].plot(time, ref_y, label='ref')
                axs[2, 1].plot(time, ref_z, label='ref')

        fig.suptitle(sol)
        axs[2, 0].set_xlabel('time')
        axs[2, 1].set_xlabel('time')

        axs[0, 0].set_ylabel('x')
        axs[1, 0].set_ylabel('y')
        axs[2, 0].set_ylabel('z')

        axs[0, 0].set_title('real')
        axs[0, 1].set_title('sim')
        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Parsing script')
    parser.add_argument('--test_arg',
                        default='',
                        type=str,
                        help='Test argument', metavar='')
    ARGS = parser.parse_args()
    run(**vars(ARGS))
