#!/usr/bin/env python3
# this version works on python3 ubuntu 20
import time

import roslib
from geometry_msgs.msg import PoseStamped
from scipy.optimize import minimize
from scipy.optimize import Bounds
import rospy
import numpy as np
import math
import sys, signal
import matplotlib.pyplot as plt


FREQ = 200
dt = 1/120
last_cup = ''
active_cup = False
once_1 = True

points = []
point_x = []
point_y = []


def fun(x):
    x_hat = x[3] * math.cos(x[5]) * (Xdata[0]) + x[3] * math.sin(x[5]) * (Xdata[1])
    y_hat = -x[4] * math.sin(x[5]) * (Xdata[0]) + x[4] * math.cos(x[5]) * (Xdata[1])

    r_ = np.sqrt(np.power(x_hat, 2) + np.power(y_hat, 2))
    phi = np.arctan2(y_hat, x_hat)

    r__dot = -1 * x[0] * (r_ - x[2])
    phi_dot = x[1]

    xd_hat = r__dot * np.cos(phi) - r_ * phi_dot * np.sin(phi)
    yd_hat = r__dot * np.sin(phi) + r_ * phi_dot * np.cos(phi)

    xdot_d1 = np.cos(x[5]) * (1 / x[3]) * xd_hat - np.sin(x[5]) * (1 / x[4]) * yd_hat
    xdot_d2 = np.sin(x[5]) * (1 / x[3]) * xd_hat + np.cos(x[5]) * (1 / x[4]) * yd_hat

    return np.sum(np.power((r_ - r) / np.linalg.norm(r, ord=1), 2)) + np.sum(
        np.power((phi - theta_circle) / np.linalg.norm(theta_circle, ord=1), 2)) + np.sum(
        np.power((r__dot - r_dot) / np.linalg.norm(r_dot, ord=1), 2)) + np.sum(
        np.power((phi_dot - theta_circle_dot) / np.linalg.norm(theta_circle_dot, ord=1), 2))


class Mocap:

    def __init__(self):

        print('Starting node..')

        size_of_input = 600
        time.sleep(2)

        self.init_params()
        self.init_ros()

        close = False
        while not close:
            if active_cup:
                cup = self.cup.pose.position

                distance = np.zeros(3)
                distance[0] = np.round(self.initial_cup.pose.position.x - cup.x, 7)
                distance[1] = np.round(self.initial_cup.pose.position.y - cup.y, 7)
                distance[2] = np.round(self.initial_cup.pose.position.z - cup.z, 7)
                vec_dis = math.sqrt(distance[0]**2 + distance[1]**2 + distance[2]**2)

                rospy.loginfo("marker %s \n", distance)

                # you need to have moved from the center
                if vec_dis > 0.1:

                    rospy.loginfo("RECORDING!")
                    point_x.append(distance[0])
                    point_y.append(distance[1])

                    if len(point_x) == size_of_input:
                        points = np.array([point_x, point_y])

                        global r, theta_circle, r_dot, theta_circle_dot, Xdata

                        Xdata = [sub[200:size_of_input-1] for sub in points]
                        # print('0', Xdata[0])
                        # print('1', Xdata[1])

                        # Xdata = np.array(data)
                        Xvel = np.diff([sub[200:size_of_input] for sub in points]) / dt

                        # convert to polar coordinates
                        r = np.sqrt(np.power(Xdata[0], 2) + np.power(Xdata[1], 2))
                        theta_circle = np.arctan2(Xdata[1], Xdata[0])

                        # convert to polar velocity coordinates
                        r_dot = np.sum(Xdata * Xvel, axis=0) / np.sqrt(
                            np.power(Xdata[0], 2) + np.power(Xdata[1], 2))  # param 1
                        theta_circle_dot = (Xvel[1] * Xdata[0] - Xdata[1] * Xvel[0]) / (
                                    np.power(Xdata[0], 2) + np.power(Xdata[1], 2))  # param 2

                        # initial parameters
                        #  alpha omega radius a b theta
                        x0 = [1, 1, 0.1, 0.1, 0.1, 0.1]
                        # bounds
                        bnds = Bounds([5, -2 * math.pi, 0.0001, 1, 1, -2 * math.pi],
                                      [50, 2 * math.pi, 2, np.infty, np.infty, 2 * math.pi])

                        res_robust = minimize(fun, x0, method='SLSQP', bounds=bnds)
                        print(res_robust.x)
                        popt = res_robust.x

                        # # Plotting Results
                        w = 0.5
                        Y, X = np.mgrid[-w:w:0.1, -w:w:0.1]

                        # compute dynamics
                        x_hat = popt[3] * math.cos(popt[5]) * X + popt[3] * math.sin(popt[5]) * Y
                        y_hat = -popt[4] * math.sin(popt[5]) * X + popt[4] * math.cos(popt[5]) * Y

                        r_ = np.sqrt(np.power(x_hat, 2) + np.power(y_hat, 2))
                        phi = np.arctan2(y_hat, x_hat)

                        r__dot = -1 * np.multiply(popt[0], (r_ - popt[2]))
                        phi_dot = popt[1]

                        xd_hat = r__dot * np.cos(phi) - r_ * phi_dot * np.sin(phi)
                        yd_hat = r__dot * np.sin(phi) + r_ * phi_dot * np.cos(phi)

                        U = math.cos(popt[5]) * (1 / popt[3]) * xd_hat - math.sin(popt[5]) * (1 / popt[4]) * yd_hat
                        V = math.sin(popt[5]) * (1 / popt[3]) * xd_hat + math.cos(popt[5]) * (1 / popt[4]) * yd_hat

                        # plotting the data
                        fig = plt.figure(1)
                        plt.plot(points[0], points[1], '.r', label='real trajectory')
                        plt.xlabel('$x$')
                        plt.ylabel('$y$')
                        plt.plot()

                        ax0 = fig.add_subplot()
                        l1 = ax0.streamplot(X, Y, U, V, density=[2, 2])
                        ax0.set_title('Limit Cycle')
                        ax0.legend()
                        plt.tight_layout()
                        plt.show()

                        close = True

            signal.signal(signal.SIGINT, self.signal_handler)

    def init_params(self):
        # initialize params for OptiTrack
        self.kuka = PoseStamped()
        self.cup = PoseStamped()

    def signal_handler(self, signal, frame):
        print("\n Program exiting gracefully")
        sys.exit(0)

    def init_ros(self):
        rospy.init_node('Mocap', anonymous=True)
        self.red_cup = rospy.Subscriber("/hand/pose", PoseStamped, self.tf_redcup)

    def tf_redcup(self, data):
        global last_cup
        global active_cup
        global once_1

        if once_1:
            last_cup = data
            self.initial_cup = data
            once_1 = False
        if data.pose != last_cup.pose:
            last_cup = data
            active_cup = True
            self.cup = data
        elif data.pose == last_cup.pose:
            active_cup = False


if __name__ == "__main__":
    Mocap()
