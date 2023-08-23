#!/usr/bin/env python
from __future__ import division
# this version works on python3 ubuntu 20
# in the terminal do "source catkin/devel/setup.bash
# to add the kortex_driver to the directory

import roslib
from geometry_msgs.msg import PoseStamped

from scipy.optimize import minimize
from scipy.optimize import Bounds
import rospy
import numpy as np
import math
import time
import sys, signal
import matplotlib.pyplot as plt
# to add the kortex_driver to the directory
from kortex_driver.msg import Base_JointSpeeds
from kortex_driver.msg import JointSpeed
from kortex_driver.msg import TwistCommand
from kortex_driver.msg import Twist
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import GripperCommandActionGoal
from scipy.signal import savgol_filter
import tf


FREQ = 40
dt = 1/120
last_cup = ''
active_cup = False
once_1 = True

points = []


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
        self.init_params()
        self.init_ros()

        print('Starting node..')

        self.get_robot_pos()
        ## tilt down
        leave = False
        while not leave:
            # go to initial pose
            # start = np.array([0.35, -0.45, 0])
            start = np.array([0.55, 0, 0.2])

            self.set_pose_down(start)
            self.get_robot_pos()
            if self.ee_real_logged:
                vel_cmd = self.calculate_robot_desired_orientation()
                self.publish_robot_desired(vel_cmd)
                print("1- Difference between desired orientation and actual\n" + str(np.round(
                    np.linalg.norm(self.desired_orientation - self.quat_to_euler(self.end_efector_orientation)),
                    2)))
                if (np.round(
                        np.linalg.norm(self.desired_orientation - self.quat_to_euler(self.end_efector_orientation)),
                        2)) < 0.1:
                    leave = True
            signal.signal(signal.SIGINT, self.signal_handler)

        # stop robot
        vel_cmd = [0, 0, 0, 0, 0, 0]
        self.publish_robot_desired(vel_cmd)
        rospy.sleep(1)

        you_moved = 0
        center = 1
        # observe human polishing
        close = False
        while not close:
            if active_cup:
                cup = self.cup.pose.position

                distance = np.zeros(3)
                distance[0] = np.round(self.initial_cup.pose.position.x - cup.x, 7)
                distance[1] = np.round(self.initial_cup.pose.position.y - cup.y, 7)
                distance[2] = np.round(self.initial_cup.pose.position.z - cup.z, 7)
                vec_dis = math.sqrt(distance[0]**2 + distance[1]**2)

                rospy.loginfo("marker %s \n", distance)

                # you need to have moved from the center
                if vec_dis > 0.1 and not you_moved and center:
                    you_moved = 1
                    center = 0
                    point_x = []
                    point_y = []
                elif you_moved:

                    point_x.append(distance[0])
                    point_y.append(distance[1])

                    if len(point_x) == 800:
                        points = np.array([point_x, point_y])
                        print(points)

                        global r, theta_circle, r_dot, theta_circle_dot, Xdata

                        Xdata = [sub[:799] for sub in points]
                        # print('0', Xdata[0])
                        # print('1', Xdata[1])

                        # Xdata = np.array(data)
                        Xvel = np.diff(points) / dt

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
                        self.popt = popt

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
                        # fig = plt.figure(1)
                        # # plt.plot(points[0], points[1], '.r', label='real trajectory')
                        # # plt.plot(data[0], data[1], '.r', label='real trajectory')
                        #
                        # ax0 = fig.subplots()
                        # l1 = ax0.streamplot(X, Y, U, V, density=[2, 2])
                        # # ax0.plot(data[0], data[1], '.r', label='real trajectory')
                        # ax0.plot(points[0], points[1], '.r', label='real trajectory')
                        #
                        # plt.xlabel('$x$')
                        # plt.ylabel('$y$')
                        # ax0.set_title('Limit Cycle')
                        # ax0.legend()
                        # plt.tight_layout()
                        # plt.show()

                        close = True

            signal.signal(signal.SIGINT, self.signal_handler)

        if self.popt[2] < 0.3:
            # Robot Polish
            leave = False
            # start = time.time()
            # do a limit cycle
            while not leave:
                self.set_pose([])
                self.get_robot_pos()
                self.circle_center = [0.5, 0.0, 0.03]
                if self.ee_real_logged:
                    # param 3 (r_value)
                    self.circle_vel = self.ellipse_ds(self.end_effector_position - self.circle_center, self.popt[2])  # 0.2
                    self.publish_robot_desired(self.circle_vel)
                    self.rate.sleep()
                # print(time.time() - 		self.rate = rospy.Rate(FREQ)start)
                # if time.time() - start > 30:
                # 	leave = True

                signal.signal(signal.SIGINT, self.signal_handler)

        # stop robot
        vel_cmd = [0, 0, 0, 0, 0, 0]
        self.publish_robot_desired(vel_cmd)
        rospy.sleep(1)

    def ellipse_ds(self, x, r_value):
        a = self.popt[3]  # 1 param 4
        b = self.popt[4]  # 3 param 5
        theta = self.popt[5]  # math.pi / 3  # param 6

        x_hat = a * math.cos(theta) * (x[0]) + a * math.sin(theta) * (x[1])
        y_hat = -b * math.sin(theta) * (x[0]) + b * math.cos(theta) * (x[1])

        # parameters for limit cycle
        theta_circle = math.atan2(y_hat, x_hat)
        r = np.sqrt(x_hat ** 2 + y_hat ** 2)

        theta_circle_dot = self.popt[1]  # math.pi / 3  # param 1
        r_dot = -1 * self.popt[0] * (r - r_value)  # param 2

        # rospy.loginfo_throttle(0.1, ['r_dot' + str(r_dot)])

        x_dot = r_dot * math.cos(theta_circle) - r * theta_circle_dot * math.sin(theta_circle)
        y_dot = r_dot * math.sin(theta_circle) + r * theta_circle_dot * math.cos(theta_circle)

        xd = math.cos(theta) * (1.0 / a) * x_dot - math.sin(theta) * (1.0 / b) * y_dot
        yd = math.sin(theta) * (1.0 / a) * x_dot + math.cos(theta) * (1.0 / b) * y_dot

        v_new = np.array([xd, yd, -2 * x[2], 0, 0, 0])
        # v=0.9*self.v_old+0.1*v_new
        v = v_new
        self.v_old = v

        return v

    def init_params(self):
        # initialize params for kinova
        self.num_joints = 6
        self.ee_real_logged = False
        self.robot_desired_joints = Base_JointSpeeds()
        self.robot_desired = TwistCommand()

    def signal_handler(self, signal, frame):
        print("\n Program exiting gracefully")
        # stop robot
        vel_cmd = [0, 0, 0, 0, 0, 0]
        self.publish_robot_desired(vel_cmd)
        sys.exit(0)

    def get_robot_pos(self):
        try:
            self.trans_ee_real = self.listener.lookupTransform('/base_link', '/end_effector_link', rospy.Time(0))
            if not self.ee_real_logged:
                self.ee_real_logged = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        self.end_effector_position = np.array(self.trans_ee_real[0])
        self.end_efector_orientation = np.array(self.trans_ee_real[1])  # A quaternion

    def init_ros(self):
        rospy.init_node('Mocap', anonymous=True)
        self.rate = rospy.Rate(FREQ)
        self.robot_pub = rospy.Publisher("/my_gen3/in/cartesian_velocity", TwistCommand, queue_size=3)
        self.red_cup = rospy.Subscriber("/hand/pose", PoseStamped, self.tf_redcup)
        self.listener = tf.TransformListener()

    def set_pose_down(self, position):
        self.target_position = position
        self.desired_orientation = np.array([math.pi -0.1, math.pi/2, 0])  # Robot end effector facing down
        self.cmd_received = False
        self.ee_real_logged = False
        self.gain = 1

    def calculate_robot_desired_attractor(self, ):
        vel_cmd = np.zeros(6)
        vel_cmd[:3] = (self.target_position - self.end_effector_position) * self.gain
        vel_cmd[3:6] = self.desired_orientation
        return vel_cmd

    def calculate_robot_desired_orientation(self, ):
        vel_cmd = np.zeros(6)

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(self.end_efector_orientation)
        vel_cmd[3:6] = (self.desired_orientation - [roll, yaw, pitch]) * self.gain
        return vel_cmd

    def quat_to_direction(self, quat):
        R0 = tf.transformations.quaternion_matrix(quat)
        angle, vec, _ = tf.transformations.rotation_from_matrix(R0)
        return angle * vec

    def quat_to_euler(self, quat):
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quat)
        euler = np.zeros(3)
        euler[0] = roll
        euler[1] = yaw
        euler[2] = pitch
        return euler

    def publish_robot_desired(self, vel_cmd):
        msg = Twist()
        msg.linear_x = vel_cmd[0]
        msg.linear_y = vel_cmd[1]
        msg.linear_z = vel_cmd[2]
        msg.angular_x = vel_cmd[3]
        msg.angular_y = vel_cmd[4]
        msg.angular_z = vel_cmd[5]
        self.robot_desired.twist = msg
        self.robot_desired.reference_frame = 0
        self.robot_desired.duration = 0
        # print(self.robot_desired)
        self.robot_pub.publish(self.robot_desired)

    def set_pose(self, position):
        self.target_position = position
        self.desired_orientation = np.array([0, 0, 0])  # Robot end effector facing down
        self.cmd_received = False
        self.ee_real_logged = False
        self.gain = 1

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
            # rospy.loginfo("Red cup at \n%s", data.pose.position)
            self.cup = data
        elif data.pose == last_cup.pose:
            active_cup = False


if __name__ == "__main__":
    Mocap()
