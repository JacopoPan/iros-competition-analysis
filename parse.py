"""Parser.

Notes:
    Main executable script.

Run as:

    $ python3 parse.py --paper

"""
import argparse
import numpy as np
from scipy.interpolate import interp1d   
import csv
import matplotlib.pyplot as plt
import pybullet as p
import tikzplotlib
# import bagpy
# from bagpy import bagreader
# import pickle


def run(
        paper=False,
    ):
    sol_num = -1
    if paper:
        fig, axs = plt.subplots(3, 9)
    for sol in ['arg', 'eku', 'h2']:
        sol_num += 1
        if not paper:
            fig, axs = plt.subplots(6, 6)
        num_files = 10

        real_ref_time = [[] for i in range(num_files)]
        real_ref_x = [[] for i in range(num_files)]
        real_ref_y = [[] for i in range(num_files)]
        real_ref_z = [[] for i in range(num_files)]
        real_time = [[] for i in range(num_files)]
        real_x = [[] for i in range(num_files)]
        real_y = [[] for i in range(num_files)]
        real_z = [[] for i in range(num_files)]
        real_q1 = [[] for i in range(num_files)]
        real_q2 = [[] for i in range(num_files)]
        real_q3 = [[] for i in range(num_files)]
        real_q4 = [[] for i in range(num_files)]
        
        sim_time = [[] for i in range(num_files)]
        sim_ref_x = [[] for i in range(num_files)]
        sim_ref_y = [[] for i in range(num_files)]
        sim_ref_z = [[] for i in range(num_files)]
        sim_x = [[] for i in range(num_files)]
        sim_y = [[] for i in range(num_files)]
        sim_z = [[] for i in range(num_files)]
        sim_r = [[] for i in range(num_files)]
        sim_p = [[] for i in range(num_files)]
        sim_j = [[] for i in range(num_files)]

        avg_real_cmd_timestep = 0

        t_new = [[] for i in range(num_files)]
        initial_skip = [0 for i in range(num_files)]
        resampled_real_ref_x = [[] for i in range(num_files)]
        resampled_real_ref_y = [[] for i in range(num_files)]
        resampled_real_ref_z = [[] for i in range(num_files)]
        resampled_real_x = [[] for i in range(num_files)]
        resampled_real_y = [[] for i in range(num_files)]
        resampled_real_z = [[] for i in range(num_files)]
        resampled_real_r = [[] for i in range(num_files)]
        resampled_real_p = [[] for i in range(num_files)]
        resampled_real_j = [[] for i in range(num_files)]

        se_x = 0
        se_y = 0
        se_z = 0
        se_count = 0
        se_ref_x = 0
        se_ref_y = 0
        se_ref_z = 0
        se_ref_count = 0

        for num in range(1,num_files+1):
            idx = num-1

            ############################
            ############################
            ############################

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
                        real_q1[idx].append(float(row[8]))
                        real_q2[idx].append(float(row[9]))
                        real_q3[idx].append(float(row[10]))
                        real_q4[idx].append(float(row[11]))

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

            with open(folder + '/r0.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for i, row in enumerate(csv_reader):
                    sim_r[idx].append(float(row[1]))
            with open(folder + '/p0.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for i, row in enumerate(csv_reader):
                    sim_p[idx].append(float(row[1]))
            with open(folder + '/ya0.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for i, row in enumerate(csv_reader):
                    sim_j[idx].append(float(row[1]))

            ############################
            ############################
            ############################

            # math magic
            initial_skip[idx] = 0
            for val in sim_time[idx]:
                if val >= real_ref_time[idx][0] and val >= real_time[idx][0]:
                    t_new[idx].append(val)
                else:
                    initial_skip[idx] += 1
                if val >= real_ref_time[idx][-1] or val >= real_time[idx][-1]:
                    print('Something is not right')
                    exit()
            t_new[idx] = np.array(t_new[idx])

            real_ref_x_func = interp1d(np.array(real_ref_time[idx]), np.array(real_ref_x[idx]), kind='linear')
            resampled_real_ref_x[idx] = real_ref_x_func(t_new[idx])
            real_ref_y_func = interp1d(np.array(real_ref_time[idx]), np.array(real_ref_y[idx]), kind='linear')
            resampled_real_ref_y[idx] = real_ref_y_func(t_new[idx])
            real_ref_z_func = interp1d(np.array(real_ref_time[idx]), np.array(real_ref_z[idx]), kind='linear')
            resampled_real_ref_z[idx] = real_ref_z_func(t_new[idx])

            real_x_func = interp1d(np.array(real_time[idx]), np.array(real_x[idx]), kind='linear')
            resampled_real_x[idx] = real_x_func(t_new[idx])
            real_y_func = interp1d(np.array(real_time[idx]), np.array(real_y[idx]), kind='linear')
            resampled_real_y[idx] = real_y_func(t_new[idx])
            real_z_func = interp1d(np.array(real_time[idx]), np.array(real_z[idx]), kind='linear')
            resampled_real_z[idx] = real_z_func(t_new[idx])

            ############################
            ############################
            ############################

            # plot one-by-one
            if not paper:
                axs[0, 0].plot(real_ref_time[idx], real_ref_x[idx], label='ref')
                axs[1, 0].plot(real_ref_time[idx], real_ref_y[idx], label='ref')
                axs[2, 0].plot(real_ref_time[idx], real_ref_z[idx], label='ref')
            #
                axs[0, 0].plot(t_new[idx], resampled_real_ref_x[idx], label='ref')
                axs[1, 0].plot(t_new[idx], resampled_real_ref_y[idx], label='ref')
                axs[2, 0].plot(t_new[idx], resampled_real_ref_z[idx], label='ref')

            avg_time = (real_ref_time[idx][-1] - real_ref_time[idx][0]) / (len(real_ref_time[idx])-1)
            avg_real_cmd_timestep += avg_time

            if paper:
                axs[0, 0+3*sol_num].plot(real_time[idx], real_x[idx], label='exp')
                axs[1, 0+3*sol_num].plot(real_time[idx], real_y[idx], label='exp')
                axs[2, 0+3*sol_num].plot(real_time[idx], real_z[idx], label='exp')
            else:
                axs[0, 1].plot(real_time[idx], real_x[idx], label='exp')
                axs[1, 1].plot(real_time[idx], real_y[idx], label='exp')
                axs[2, 1].plot(real_time[idx], real_z[idx], label='exp')
                #
                axs[0, 1].plot(t_new[idx], resampled_real_x[idx], label='exp')
                axs[1, 1].plot(t_new[idx], resampled_real_y[idx], label='exp')
                axs[2, 1].plot(t_new[idx], resampled_real_z[idx], label='exp')
            #
            real_euler1 = []
            real_euler2 = []
            real_euler3 = []
            for t in range(len(real_q1[idx])):
                val = p.getEulerFromQuaternion([real_q1[idx][t], real_q2[idx][t], real_q3[idx][t], real_q4[idx][t]])
                real_euler1.append(val[0])
                real_euler2.append(val[1])
                real_euler3.append(val[2])
                # getEulerFromQuaternion returns a list of 3 floating point values, a vec3. 
                # The rotation order is first roll around X, then pitch around Y and finally yaw around Z, 
                # as in the ROS URDF rpy convention.
            if not paper:
                axs[3, 1].plot(real_time[idx], real_euler1, label='exp')
                axs[4, 1].plot(real_time[idx], real_euler2, label='exp')
                axs[5, 1].plot(real_time[idx], real_euler3, label='exp')
            #
            real_r_func = interp1d(np.array(real_time[idx]), np.array(real_euler1), kind='linear')
            resampled_real_r[idx] = real_r_func(t_new[idx])
            real_p_func = interp1d(np.array(real_time[idx]), np.array(real_euler2), kind='linear')
            resampled_real_p[idx] = real_p_func(t_new[idx])
            real_j_func = interp1d(np.array(real_time[idx]), np.array(real_euler3), kind='linear')
            resampled_real_j[idx] = real_j_func(t_new[idx])
            #
            if not paper:
                axs[3, 1].plot(t_new[idx], resampled_real_r[idx], label='exp')
                axs[4, 1].plot(t_new[idx], resampled_real_p[idx], label='exp')
                axs[5, 1].plot(t_new[idx], resampled_real_j[idx], label='exp')

                axs[0, 2].plot(sim_time[idx], sim_ref_x[idx], label='ref')
                axs[1, 2].plot(sim_time[idx], sim_ref_y[idx], label='ref')
                axs[2, 2].plot(sim_time[idx], sim_ref_z[idx], label='ref')  

            if paper:
                axs[0, 1+3*sol_num].plot(sim_time[idx], sim_x[idx], label='sim')
                axs[1, 1+3*sol_num].plot(sim_time[idx], sim_y[idx], label='sim')
                axs[2, 1+3*sol_num].plot(sim_time[idx], sim_z[idx], label='sim')
            else:
                axs[0, 3].plot(sim_time[idx], sim_x[idx], label='sim')
                axs[1, 3].plot(sim_time[idx], sim_y[idx], label='sim')
                axs[2, 3].plot(sim_time[idx], sim_z[idx], label='sim')
                #
                axs[3, 3].plot(sim_time[idx], sim_r[idx], label='sim')
                axs[4, 3].plot(sim_time[idx], sim_p[idx], label='sim')
                axs[5, 3].plot(sim_time[idx], sim_j[idx], label='sim')

            se_x += np.sum((resampled_real_x[idx] - np.array(sim_x[idx][initial_skip[idx]:]))**2)
            se_y += np.sum((resampled_real_y[idx] - np.array(sim_y[idx][initial_skip[idx]:]))**2)
            se_z += np.sum((resampled_real_z[idx] - np.array(sim_z[idx][initial_skip[idx]:]))**2)
            se_count += len(resampled_real_x[idx])

            se_ref_x += np.sum((resampled_real_ref_x[idx] - np.array(sim_ref_x[idx][initial_skip[idx]:]))**2)
            se_ref_y += np.sum((resampled_real_ref_y[idx] - np.array(sim_ref_y[idx][initial_skip[idx]:]))**2)
            se_ref_y += np.sum((resampled_real_ref_z[idx] - np.array(sim_ref_z[idx][initial_skip[idx]:]))**2)
            se_ref_count += len(resampled_real_ref_x[idx])

            if not paper:
                axs[0, 4].plot(t_new[idx], (sim_x[idx][initial_skip[idx]:]-resampled_real_x[idx])**2, label='diff')
                axs[1, 4].plot(t_new[idx], (sim_y[idx][initial_skip[idx]:]-resampled_real_y[idx])**2, label='diff')
                axs[2, 4].plot(t_new[idx], (sim_z[idx][initial_skip[idx]:]-resampled_real_z[idx])**2, label='diff')

                axs[3, 4].plot(t_new[idx], (sim_r[idx][initial_skip[idx]:]-resampled_real_r[idx])**2, label='diff')
                axs[4, 4].plot(t_new[idx], (sim_p[idx][initial_skip[idx]:]-resampled_real_p[idx])**2, label='diff')
                axs[5, 4].plot(t_new[idx], (sim_j[idx][initial_skip[idx]:]-resampled_real_j[idx])**2, label='diff')

                axs[0, 5].plot(t_new[idx], (sim_ref_x[idx][initial_skip[idx]:]-resampled_real_ref_x[idx])**2, label='diff')
                axs[1, 5].plot(t_new[idx], (sim_ref_y[idx][initial_skip[idx]:]-resampled_real_ref_y[idx])**2, label='diff')
                axs[2, 5].plot(t_new[idx], (sim_ref_z[idx][initial_skip[idx]:]-resampled_real_ref_z[idx])**2, label='diff')

            ############################
            ############################
            ############################

        latest_start = -1
        earliest_end = 9000
        for i in range(num_files):
            if sol == 'eku' and i == 1:
                continue
            latest_start = max(latest_start, t_new[i][0])
            earliest_end = min(earliest_end, t_new[i][-1])
            # print(t_new[i][0], t_new[i][-1])
        print(latest_start, earliest_end)
        for i in range(num_files):
            pass
            # print(f'{initial_skip[i]} {t_new[i][0]:.3f}', t_new[i].shape, np.array(sim_x[i][initial_skip[i]:]).shape, np.array(sim_ref_x[i][initial_skip[i]:]).shape, np.array(resampled_real_x[i]).shape, np.array(resampled_real_ref_x[i]).shape)
        
        cropped_real_x = np.zeros((num_files, int((earliest_end-latest_start)*30)))
        cropped_sim_x = np.zeros((num_files, int((earliest_end-latest_start)*30)))
        cropped_real_y = np.zeros((num_files, int((earliest_end-latest_start)*30)))
        cropped_sim_y = np.zeros((num_files, int((earliest_end-latest_start)*30)))
        cropped_real_z = np.zeros((num_files, int((earliest_end-latest_start)*30)))
        cropped_sim_z = np.zeros((num_files, int((earliest_end-latest_start)*30)))
        #
        cropped_real_r = np.zeros((num_files, int((earliest_end-latest_start)*30)))
        cropped_sim_r = np.zeros((num_files, int((earliest_end-latest_start)*30)))
        cropped_real_p = np.zeros((num_files, int((earliest_end-latest_start)*30)))
        cropped_sim_p = np.zeros((num_files, int((earliest_end-latest_start)*30)))
        cropped_real_j = np.zeros((num_files, int((earliest_end-latest_start)*30)))
        cropped_sim_j = np.zeros((num_files, int((earliest_end-latest_start)*30)))

        for i in range(num_files):
            if sol == 'eku' and i == 1:
                continue
            insert_idx = 0
            for j in range(len(t_new[i])):
                if t_new[i][j] >= latest_start:
                    cropped_real_x[i][insert_idx] = resampled_real_x[i][j]
                    cropped_sim_x[i][insert_idx] = sim_x[i][j]
                    cropped_real_y[i][insert_idx] = resampled_real_y[i][j]
                    cropped_sim_y[i][insert_idx] = sim_y[i][j]
                    cropped_real_z[i][insert_idx] = resampled_real_z[i][j]
                    cropped_sim_z[i][insert_idx] = sim_z[i][j]
                    #
                    cropped_real_r[i][insert_idx] = resampled_real_r[i][j]
                    cropped_sim_r[i][insert_idx] = sim_r[i][j]
                    cropped_real_p[i][insert_idx] = resampled_real_p[i][j]
                    cropped_sim_p[i][insert_idx] = sim_p[i][j]
                    cropped_real_j[i][insert_idx] = resampled_real_j[i][j]
                    cropped_sim_j[i][insert_idx] = sim_j[i][j]
                    insert_idx += 1
                    if insert_idx == cropped_real_x.shape[1]:
                        break

        avg_real_x = np.average(cropped_real_x, axis=0)
        avg_sim_x = np.average(cropped_sim_x, axis=0)
        avg_rmse_x = np.sqrt(np.sum((avg_real_x-avg_sim_x)**2)/len(avg_real_x))
        avg_real_y = np.average(cropped_real_y, axis=0)
        avg_sim_y = np.average(cropped_sim_y, axis=0)
        avg_rmse_y = np.sqrt(np.sum((avg_real_y-avg_sim_y)**2)/len(avg_real_y))
        avg_real_z = np.average(cropped_real_z, axis=0)
        avg_sim_z = np.average(cropped_sim_z, axis=0)
        avg_rmse_z = np.sqrt(np.sum((avg_real_z-avg_sim_z)**2)/len(avg_real_z))
        #
        avg_real_r = np.average(cropped_real_r, axis=0)
        avg_sim_r = np.average(cropped_sim_r, axis=0)
        avg_rmse_r = np.sqrt(np.sum((avg_real_r-avg_sim_r)**2)/len(avg_real_r))
        avg_real_p = np.average(cropped_real_p, axis=0)
        avg_sim_p = np.average(cropped_sim_p, axis=0)
        avg_rmse_p = np.sqrt(np.sum((avg_real_p-avg_sim_p)**2)/len(avg_real_p))
        avg_real_j = np.average(cropped_real_j, axis=0)
        avg_sim_j = np.average(cropped_sim_j, axis=0)
        avg_rmse_j = np.sqrt(np.sum((avg_real_j-avg_sim_j)**2)/len(avg_real_j))
        print(f'avg_rmse_xyz {avg_rmse_x:.2f} {avg_rmse_y:.2f} {avg_rmse_z:.2f}')
        print(f'avg_rmse_rpj {avg_rmse_r:.2f} {avg_rmse_p:.2f} {avg_rmse_j:.2f}')

        cropped_time = np.linspace(latest_start, earliest_end, cropped_real_x.shape[1])
        if paper:
            axs[0, 2+3*sol_num].plot(cropped_time, (avg_real_x-avg_sim_x)**2, label='diff')
            axs[1, 2+3*sol_num].plot(cropped_time, (avg_real_y-avg_sim_y)**2, label='diff')
            axs[2, 2+3*sol_num].plot(cropped_time, (avg_real_z-avg_sim_z)**2, label='diff')
            #
            axs[0, 2+3*sol_num].plot(cropped_time,  (avg_real_x-avg_sim_x)**2+np.std(cropped_real_x-cropped_sim_x, axis=0), label='diff')
            axs[0, 2+3*sol_num].plot(cropped_time,  (avg_real_x-avg_sim_x)**2-np.std(cropped_real_x-cropped_sim_x, axis=0), label='diff')
            axs[1, 2+3*sol_num].plot(cropped_time,  (avg_real_y-avg_sim_y)**2+np.std(cropped_real_y-cropped_sim_y, axis=0), label='diff')
            axs[1, 2+3*sol_num].plot(cropped_time,  (avg_real_y-avg_sim_y)**2-np.std(cropped_real_y-cropped_sim_y, axis=0), label='diff')
            axs[2, 2+3*sol_num].plot(cropped_time,  (avg_real_z-avg_sim_z)**2+np.std(cropped_real_z-cropped_sim_z, axis=0), label='diff')
            axs[2, 2+3*sol_num].plot(cropped_time,  (avg_real_z-avg_sim_z)**2-np.std(cropped_real_z-cropped_sim_z, axis=0), label='diff')
        else:
            axs[0, 4].plot(cropped_time, (avg_real_x-avg_sim_x)**2, label='diff')
            axs[1, 4].plot(cropped_time, (avg_real_y-avg_sim_y)**2, label='diff')
            axs[2, 4].plot(cropped_time, (avg_real_z-avg_sim_z)**2, label='diff')
            #
            axs[3, 4].plot(cropped_time, (avg_real_r-avg_sim_r)**2, label='diff')
            axs[4, 4].plot(cropped_time, (avg_real_p-avg_sim_p)**2, label='diff')
            axs[5, 4].plot(cropped_time, (avg_real_j-avg_sim_j)**2, label='diff')

        # mse_ref_x = se_ref_x/se_ref_count
        # rmse_ref_x = np.sqrt(mse_ref_x)
        # mse_ref_y = se_ref_y/se_ref_count
        # rmse_ref_y = np.sqrt(mse_ref_y)
        # mse_ref_z = se_ref_z/se_ref_count
        # rmse_ref_z = np.sqrt(mse_ref_z)
        # avg_real_cmd_timestep /= 10
        # real_cmd_freq =  1/avg_real_cmd_timestep
        # print(f'rmse_ref_xyz ({rmse_ref_x:.2f} {rmse_ref_y:.2f} {rmse_ref_z:.2f}) real_cmd_freq {real_cmd_freq:.2f}')

        # rmse_x = np.sqrt(se_x/se_count)
        # rmse_y = np.sqrt(se_y/se_count)
        # rmse_z = np.sqrt(se_z/se_count)

        # labels
        # fig.suptitle(f'{sol}, rmse_xyz ({rmse_x:.2f} {rmse_y:.2f} {rmse_z:.2f})')
        if not paper:
            fig.suptitle(f'{sol}, avg rmse xyz {avg_rmse_x:.2f} {avg_rmse_y:.2f} {avg_rmse_z:.2f} rpj  {avg_rmse_r:.2f} {avg_rmse_p:.2f} {avg_rmse_j:.2f}')
            axs[5, 0].set_xlabel('time')
            axs[5, 1].set_xlabel('time')
            axs[5, 2].set_xlabel('time')
            axs[5, 3].set_xlabel('time')
            axs[5, 4].set_xlabel('time')
            axs[5, 5].set_xlabel('time')

            axs[0, 0].set_ylabel('x')
            axs[1, 0].set_ylabel('y')
            axs[2, 0].set_ylabel('z')
            axs[3, 0].set_ylabel('r')
            axs[4, 0].set_ylabel('p')
            axs[5, 0].set_ylabel('j')

            axs[0, 0].set_title('real_cmd')
            axs[0, 1].set_title('real')
            axs[0, 2].set_title('sim_cmd')
            axs[0, 3].set_title('sim')
            axs[0, 4].set_title('sq_diff')
            axs[0, 5].set_title('cmd_sq_diff')

            # axes limits
            y_axis_range = 7
            axs[0, 0].set_ylim(-y_axis_range/2,y_axis_range/2)
            axs[0, 1].set_ylim(-y_axis_range/2,y_axis_range/2)
            axs[0, 2].set_ylim(-y_axis_range/2,y_axis_range/2)
            axs[0, 3].set_ylim(-y_axis_range/2,y_axis_range/2)
            axs[0, 4].set_ylim(0,1.25*y_axis_range)
            axs[0, 5].set_ylim(0,1.25*y_axis_range)

            axs[1, 0].set_ylim(-y_axis_range/2,y_axis_range/2)
            axs[1, 1].set_ylim(-y_axis_range/2,y_axis_range/2)
            axs[1, 2].set_ylim(-y_axis_range/2,y_axis_range/2)
            axs[1, 3].set_ylim(-y_axis_range/2,y_axis_range/2)
            axs[1, 4].set_ylim(0,1.25*y_axis_range)
            axs[1, 5].set_ylim(0,1.25*y_axis_range)

            axs[2, 0].set_ylim(-y_axis_range/2,y_axis_range/2)
            axs[2, 1].set_ylim(-y_axis_range/2,y_axis_range/2)
            axs[2, 2].set_ylim(-y_axis_range/2,y_axis_range/2)
            axs[2, 3].set_ylim(-y_axis_range/2,y_axis_range/2)
            axs[2, 4].set_ylim(0,1.25*y_axis_range)
            axs[2, 5].set_ylim(0,1.25*y_axis_range)

            if sol =='arg':
                axs[3, 1].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[4, 1].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[5, 1].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[3, 3].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[4, 3].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[5, 3].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[3, 4].set_ylim(0,1.25*y_axis_range)
                axs[4, 4].set_ylim(0,1.25*y_axis_range)
                axs[5, 4].set_ylim(0,3*y_axis_range)
            elif sol =='eku':
                axs[3, 1].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[4, 1].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[5, 1].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[3, 3].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[4, 3].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[5, 3].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[3, 4].set_ylim(0,1.25*y_axis_range)
                axs[4, 4].set_ylim(0,1.25*y_axis_range)
                axs[5, 4].set_ylim(0,1.25*y_axis_range)
            if sol =='h2':
                axs[3, 1].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[4, 1].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[5, 1].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[3, 3].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[4, 3].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[5, 3].set_ylim(-y_axis_range/2,y_axis_range/2)
                axs[3, 4].set_ylim(0,1.25*y_axis_range)
                axs[4, 4].set_ylim(0,1.25*y_axis_range)
                axs[5, 4].set_ylim(0,5.5*y_axis_range)

        if not paper:
            tikzplotlib.clean_figure(target_resolution=10, scale_precision=1.0)
            tikzplotlib.save('./tikz/' + sol + '.tex')
            plt.savefig('./png/' + sol + '.png')
            plt.show()
    
    if paper:
        axs[0, 0].set_ylabel('x (m)')
        axs[1, 0].set_ylabel('y (m)')
        axs[2, 0].set_ylabel('z (m)')
        for col in range(9):
            axs[2, col].set_xlabel('time (s)')
        for col in range(1,9):
            for row in range(3):
                axs[row, col].yaxis.set_ticklabels([])
        for col in range(9):
            for row in range(2):
                axs[row, col].xaxis.set_ticklabels([])
        axs[0, 0].set_title('real arg')
        axs[0, 3].set_title('real eku')
        axs[0, 6].set_title('real h2')
        axs[0, 1].set_title('sim arg')
        axs[0, 4].set_title('sim eku')
        axs[0, 7].set_title('sim h2')
        for col in [2, 5, 8]:
            axs[0, col].set_title('diff (avg, pm std)')
        for col in [0, 1, 3, 4, 6, 7]:
            for row in range(3):
                axs[row, col].set_ylim(-3.5,3.5)
        for col in [2, 5, 8]:
            for row in range(3):
                axs[row, col].set_ylim(-2,6)
        for col in [0, 1, 2]:
            for row in range(3):
                axs[row, col].set_xlim(0,12)
        for col in [3, 4, 5]:
            for row in range(3):
                axs[row, col].set_xlim(0,24)
        for col in [6, 7, 8]:
            for row in range(3):
                axs[row, col].set_xlim(0,12)

        tikzplotlib.clean_figure(target_resolution=50, scale_precision=1.0)
        tikzplotlib.save('./tikz/all.tex')
        plt.savefig('./png/all.png')
        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Parsing script')
    parser.add_argument('--paper',
                        action='store_true',
                        help='Test argument', 
                        )
    ARGS = parser.parse_args()
    run(**vars(ARGS))
