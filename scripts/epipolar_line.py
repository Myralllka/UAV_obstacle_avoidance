#!/bin/env python3

import cv2
import rospy
import random
import numpy as np

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

img_right, img_left = None, None
bridge = CvBridge()


def sqc(x):
    """
    Skew-symmetric matrix for cross-product
    Synopsis: S = sqc(x)
    :param x: vector 3×1
    :return: skew symmetric matrix (3×3) for cross product with x
    """
    return np.mat([[0, x[0, 2], -x[0, 1]],
                   [-x[0, 2], 0, x[0, 0]],
                   [x[0, 1], -x[0, 0], 0]])


RL_t = [0.0957874,
        -0.00830055,
        -0.0937745]
RL_r = [[-0.00861925, 0.00341071, 0.999957],
        [-0.0129028, 0.999911, -0.00352177],
        [-0.99988, -0.0129326, -0.00857447]]
K_left = [[814.70114, 0, 801.2499],
          [0, 811.53998, 602.45886],
          [0, 0, 1]]
K_right = [[815.5976000000001, 0, 796.8774100000001],
           [0, 812.26899, 584.6144],
           [0, 0, 1]]

RL_t = np.mat(RL_t)
RL_r = np.mat(RL_r)

K_left = np.mat(K_left)
K_right = np.mat(K_right)

# compute RL
E = RL_r @ -sqc(RL_t)
F = np.linalg.inv(K_left).T @ E @ np.linalg.inv(K_right)

print(F)
flag = False


def callback_right(data: Image):
    global img_right
    global flag
    img_right = bridge.imgmsg_to_cv2(data, 'passthrough')

    cv2.imshow('image right', img_right)
    cv2.imshow('image left', img_left)
    flag = True
    cv2.setMouseCallback('image right', click_event)
    while cv2.waitKey(1) != 27:
        continue
    cv2.destroyAllWindows()
    flag = False
    print("callback_right complete")


def callback_left(data: Image):
    global img_left
    if flag:
        return
    img_left = bridge.imgmsg_to_cv2(data, 'passthrough')

    print("callback_left complete")


def click_event(event, x, y, flags, params):
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        print(x, ' ', y)
        color = tuple(random.randint(0, 255) for _ in range(3))
        # print(F)
        # print(np.array([x, y, 1]))
        epiline = -(F.T @ np.array([x, y, 1]))

        x0, y0 = 0, int(-epiline[0, 2] / epiline[0, 1])
        x1, y1 = 1920, int((-epiline[0, 2] + epiline[0, 0] * 1920) / epiline[0, 1])

        p1 = (x0, y0)
        p2 = (x1, y1)

        cv2.circle(img_right, (x, y), 3, color, 3)
        cv2.line(img_left, p1, p2, color, 2)
        cv2.imshow('image right', img_right)
        cv2.imshow('image left', img_left)


def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/uav1/basler_right/image_rect", Image, callback_right)
    rospy.Subscriber("/uav1/basler_left/image_rect", Image, callback_left)
    rospy.spin()


if __name__ == "__main__":
    listener()
