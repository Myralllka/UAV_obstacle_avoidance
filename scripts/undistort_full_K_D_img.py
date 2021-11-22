#!/bin/env python3
# source: https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-part-2-13990f1b157f
# source: https://answers.opencv.org/question/220574/how-to-undistort-fisheye-image-into-max-bounding-rectangle/

import numpy as np
import os
import glob
import sys
import cv2

DIM = (1600, 1200)


def undistort(m_K, m_D, img_path, balance, dim2=None, dim3=None):
    img = cv2.imread(img_path)
    dim1 = img.shape[:2][::-1]
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1

    scaled_K = m_K * dim1[0] / DIM[
        0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0

    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K,
                                                                   m_D, dim2,
                                                                   np.eye(3),
                                                                   balance=balance)

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, m_D, np.eye(3),
                                                     new_K, dim3, cv2.CV_32F)
    print(map1.shape)
    print(map2.shape)

    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_CUBIC,
                                borderMode=cv2.BORDER_CONSTANT)
    cv2.imwrite(img_path.replace("jpeg", "full_undistored.jpeg"),
                undistorted_img)


if __name__ == '__main__':
    K = np.loadtxt(sys.argv[1])
    D = np.loadtxt(sys.argv[2])
    for p in sys.argv[3:]:
        undistort(K, D, p, 1.0, (3200, 2400), (6400, 4800))
