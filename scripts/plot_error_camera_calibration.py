#!/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import re

if __name__ == "__main__":
    filename_left = "in_file_left.txt"
    filename_right = "in_file_right.txt"
    data_reprojection_left = []
    data_reprojection_right = []

    with open(filename_left, 'r') as f:
        p = re.compile(r'\d+\.\d+')
        for line in f.readlines():
            floats = [float(i) for i in p.findall(line)]
            if len(floats) == 2:
                data_reprojection_left.append(floats[1])

    with open(filename_right, 'r') as f:
        p = re.compile(r'\d+\.\d+')
        for line in f.readlines():
            floats = [float(i) for i in p.findall(line)]
            if len(floats) == 2:
                data_reprojection_right.append(floats[1])
    l = min(len(data_reprojection_left), len(data_reprojection_right))
    x = np.linspace(0, l - 1, l)

    fig = plt.figure()
    fig.set_figheight(4)
    fig.set_figwidth(12)

    plt.subplot(121)
    plt.title("Right camera reprojection error")
    plt.xlabel('image number')
    plt.ylabel('reprojection RMS error [px]')
    plt.scatter(x, data_reprojection_right[:l], color='g')

    plt.subplot(122)

    plt.title("Left camera reprojection error")
    plt.xlabel('image number')
    plt.ylabel('reprojection RMS error [px]')
    plt.scatter(x, data_reprojection_left[:l], color='g')


    fig.legend(loc="best")
    plt.show()
#    fig = plt.figure(figsize=(8, 6), dpi=60)
#    fig.clf()
#    plt.title('The Reprojection RMS error')
#    plt.xlabel('image number')
#    plt.ylabel('Reprojection RMS error [px]')
#    plt.plot(x, data_reprojection_right[:l], color='g', label='image 2')
#    plt.legend(loc='best')
#    plt.show()
