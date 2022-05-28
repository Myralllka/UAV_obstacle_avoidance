#!/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import re
import sys
import os
import csv
import pandas as pd

fname_plane_dist = "_plane_distance.csv"  # file type 1
fname_plane_pts_dist = "_plane_pts_distance.csv"  # file type 2
fname_rms_repro = "_rms_repro_err.csv"  # file type 3
fname_total_repro = "_total_repro_err.csv"  # file type 4


def analise_t1(inp_data):
    pass


def analise_t2(inp_data):
    pass


def analise_t3(inp_data):
    pass


def analise_t4(inp_data):
    pass


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("not enough arguments! Please provide a directory")
        exit(-1)
    ft1 = list()
    ft2 = list()
    ft3 = list()
    ft4 = list()
    directory = sys.argv[1]
    for filename in sorted(os.listdir(directory)):
        distance = float(filename[:3])
        file_type = -1
        if filename.find(fname_plane_dist) != -1:
            file_type = 1
        elif filename.find(fname_plane_pts_dist) != -1:
            file_type = 2
        elif filename.find(fname_rms_repro) != -1:
            file_type = 3
        elif filename.find(fname_total_repro) != -1:
            file_type = 4
        else:
            print("wrong filename")
            exit(-1)

        f = os.path.join(directory, filename)
        if os.path.isfile(f):
            current = list()
            with open(f, newline="") as csvfile:
                reader = csv.reader(csvfile, delimiter=',', quotechar='|')
                for row in reader:
                    if len(row) == 1:
                        current.append(float(row[0]))
                    else:
                        arr = [float(i) for i in row if not len(i) == 0]
                        if len(arr) == 0:
                            current.append(0)
                        else:
                            current.append(sum(arr) / len(arr))

            if file_type == 1:
                ft1.append(current)
            elif file_type == 2:
                ft2.append(current)
            elif file_type == 3:
                ft3.append(current)
            elif file_type == 4:
                ft4.append(current)

    # print(len(min(ft1, key=len)))
    if len(ft1) == 0:
        pass
    # with open('file_test.txt', 'w', newline='') as csvfile:
    #     writer = csv.writer(csvfile, delimiter=',')
    #     writer.writerows(ft2)

    my_df = pd.DataFrame(ft1)
    my_df.to_csv("file_test1.csv", index=False, header=False)
    # ft1 = np.array(ft1)
    # ft2 = np.array(ft2)
    # ft3 = np.array(ft3)
    # ft4 = np.array(ft4)
    #
    # fig = plt.figure()
    # fig.set_figheight(4)
    # fig.set_figwidth(12)
