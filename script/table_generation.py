#!/usr/bin/python
import matplotlib

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

import os
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import re
import tf
from geometry_msgs.msg import Pose, Twist, PoseStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

time = 0  # first column in file
length = 1  # second column in file
smooth = 2  # third column in file


def makefilename(name):
    return name.strip().replace(' ', '_').replace('(', '').replace(')', '')


class VisualizeResults:
    """"
        Visualization results of benchmarking
        input: file names
        output: store/show plot
    """

    # -----------------------------------------------------------------------------------------------------------------------
    # @description: Default constructor of Plot Class
    def __init__(self):
        self.mat = []
        self.file_names = []
        self.column_names = []
        self.data_to_plot = []

        self.cart_length = []
        self.time_to_reach = []
        self.cart_smoothness = []

    # -----------------------------------------------------------------------------------------------------------------------
    # @description: Read data from file and separate into column wise and plot column data
    def Run(self):

        # store all files data into array called self.mat
        self.task = sys.argv[1]
        for i in range(1, len(sys.argv)):
            if sys.argv[i].rfind('.') != -1:  # If .csv or .txt is not pass than consider as that xlable
                file_name = re.split('\d+', sys.argv[i])
                self.file_names.append(file_name[0] + str(i))
                self.readDataFromFile(sys.argv[i])
            else:
                # array of file name already filled by file_names, if optional argu are there,than replace file_name with optional argu.
                # i-len(self.file_name) - 1, -1 is because we start for loop from 1 and ayyry element start from 0
                # self.file_names[i-len(self.file_names) - 1] = sys.argv[i]
                self.file_names.append(sys.argv[i])

        self.file_names = [file_name.replace("_", ":") for file_name in self.file_names]

        # for planner in self.planner_names:
        #    print ('\033[94m' + planner + '\033[0m')
        # print self.mat[0][2:]
        # self.visulizePlot()
        # self.visualizeTrajectory()



        for matrix_id in range(0, len(self.mat)):

            print ('\033[1m' + '\033[31m' + "############ " + " Starting export " + "############## " + '\033[0m')

            plt.figure(matrix_id)
            figure = plt.figure()
            plt.rc('axes', axisbelow=True)  # draw grid line behind plot
            time_index = self.column_names.index('time')

            # remove entry of blank or zero values
            self.column_names = filter(lambda a: a != 'Unnamed: 0', self.column_names)

            # --------------------------------- Time to reach goal pose -----------------------------------------------
            # extract time data and set that as our x axis
            time_axis = self.extractColumData(matrix_id=matrix_id, colum_index=time_index)
            self.time_to_reach.append(time_axis[len(time_axis)-1])
            #print self.time_to_reach

            # ------------------------------- Cartesian length of trajectory ------------------------------------------
            x = []; y = []; z = []; qx = []; qy = []; qz = []; qw = [];

            x = self.extractColumData(matrix_id=matrix_id, colum_index=1)  # x = 1
            y = self.extractColumData(matrix_id=matrix_id, colum_index=2)  # y = 2
            z = self.extractColumData(matrix_id=matrix_id, colum_index=3)  # z = 3

            qx = self.extractColumData(matrix_id=matrix_id, colum_index=4)  # qx = 1
            qy = self.extractColumData(matrix_id=matrix_id, colum_index=5)  # qy = 2
            qz = self.extractColumData(matrix_id=matrix_id, colum_index=6)  # qz = 3
            qw = self.extractColumData(matrix_id=matrix_id, colum_index=7)  # qw = 3


            self.cart_length.append(self.computeEcludianDistance(x=x, y=y, z=z))
            #print self.computeRoatationDistance(qx=qx, qy=qy, qz=qz, qw=qw)


            # ------------------------------- Sommothness of trajectory ----------------------------------------------
            self.cart_smoothness.append(self.computeCartesianSommothNessOfTraj(x=x, y=y, z=z))

        table_header = "\multirow{2}{*}{Algorithm} & \multicolumn{3}{c}{Solver iterations}  \Tstrut \\\\"
        table_header_2 = "& Execution time & Trajectory length (m) & Smoothness \Bstrut \\\\ \hline"

        print (table_header)
        print (table_header_2)

        for time, length, smoothness in zip(self.time_to_reach, self.cart_length, self.cart_smoothness):

            print ("----------------------------------------------------------------------------------------------------")
            print "pose:0" + ' & ' + ("%0.3f" % time) + ' & ' + ("%0.3f" % length) + ' & ' + ("%0.3f" % smoothness) + '\\\\'
            print ("----------------------------------------------------------------------------------------------------")



    def computeCartesianSommothNessOfTraj(self,x,y,z):
        cart_smoothness = 0.0

        smooth_x = 0.0; smooth_y = 0.0; smooth_z = 0.0

        for p in range(0, len(x)-2):
            f_x0 = abs(x[p] - x[0])
            f_y0 = abs(y[p] - y[0])
            f_z0 = abs(z[p] - z[0])

            f_x1 = abs(x[p+1] - x[0])
            f_y1 = abs(y[p+1] - y[0])
            f_z1 = abs(z[p+1] - z[0])

            f_x2 = abs(x[p+2] - x[0])
            f_y2 = abs(y[p+2] - y[0])
            f_z2 = abs(z[p+2] - z[0])

            smooth_x = smooth_x + abs(f_x2 + f_x1 - 2*f_x0)
            smooth_y = smooth_y + abs(f_y2 + f_y1 - 2*f_y0)
            smooth_z = smooth_z + abs(f_z2 + f_z1 - 2*f_z0)

        cart_smoothness = smooth_x / len(x) + smooth_y / len(y) + smooth_z / len(z)
        return cart_smoothness

    def computeRotationSommothNessOfTraj(self,x,y,z):
        rot_smoothness = 0.0

        smooth_x = 0.0; smooth_y = 0.0; smooth_z = 0.0

        for p in range(0, len(x)-2):
            f_x0 = abs(x[p] - x[0])
            f_y0 = abs(y[p] - y[0])
            f_z0 = abs(z[p] - z[0])

            f_x1 = abs(x[p+1] - x[0])
            f_y1 = abs(y[p+1] - y[0])
            f_z1 = abs(z[p+1] - z[0])

            f_x2 = abs(x[p+2] - x[0])
            f_y2 = abs(y[p+2] - y[0])
            f_z2 = abs(z[p+2] - z[0])

            smooth_x = smooth_x + abs(f_x2 + f_x1 - 2*f_x0)
            smooth_y = smooth_y + abs(f_y2 + f_y1 - 2*f_y0)
            smooth_z = smooth_z + abs(f_z2 + f_z1 - 2*f_z0)

        rot_smoothness = smooth_x / len(x) + smooth_y / len(y) + smooth_z / len(z)
        return rot_smoothness



    def computeEcludianDistance(self, x, y, z):

        cart_length = 0.0
        for p in range(1, len(x)):
            way_point_p1 = np.array([x[p], y[p], z[p]])
            way_point_p0 = np.array([x[p-1], y[p-1], z[p-1]])
            cart_length = cart_length + self.distance(way_point_p1, way_point_p0)

        return cart_length

    def computeRoatationDistance(self, qx, qy, qz, qw):
        rot_length = 0.0
        for p in range(1, len(qx)):

            quat_1 = Quaternion()
            quat_1 = (qx[p], qy[p], qz[p], qw[p])
            rpy_1 = tf.transformations.euler_from_quaternion(quat_1, 'rzyx')

            quat_2 = Quaternion()
            quat_2 = (qx[p-1], qy[p-1], qz[p-1], qw[p-1])
            rpy_2 = tf.transformations.euler_from_quaternion(quat_2, 'rzyx')

            way_point_p1 = np.array([rpy_1[0], rpy_1[1], rpy_1[2]])
            way_point_p0 = np.array([rpy_2[0], rpy_2[1], rpy_2[2]])
            rot_length = rot_length + self.distance(way_point_p1, way_point_p0)

        return rot_length

    def distance(self,vec_a, vec_b):
        return np.sqrt(np.sum((vec_a-vec_b)**2))

    def extractColumData(self, matrix_id, colum_index):
        clm_data = []

        for data in self.mat[matrix_id][:, colum_index]:
            clm_data.append(data)

        return (clm_data)

    # -----------------------------------------------------------------------------------------------------------------------
    """
        @:description Extract data from matrix which store file by file
        @:param matrix_id File number from extract data
    """

    def extractDataFromMatrix(self, matrix_id):
        x = [];
        y = [];
        z = []

        for px in self.mat[matrix_id][2:, 1]:
            x.append(float(px))
        for py in self.mat[matrix_id][2:, 2]:
            y.append(float(py))
        for pz in self.mat[matrix_id][2:, 3]:
            z.append(float(pz))
        return (x, y, z)

    # -----------------------------------------------------------------------------------------------------------------------
    """
        @:description HardCoded function used to visualize trajectory
    """

    def visualizeTrajectory(self):
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1, projection='3d')

        x = [];
        y = [];
        z = []
        x, y, z = self.extractDataFromMatrix(0)
        print x, y, z
        ax.plot(x, y, z)
        # ax.plot(x,y,z,linewidth=1)
        plt.show()

    def visualize2DPlot(self, x_axis, y_axis, clm_index, x_min_range, x_max_range, y_min_range, y_max_range, fig):
        filenames = []

        ax = fig.add_subplot(1, 1, 1)

        plt.legend()

        # set scale
        plt.xscale('linear')
        plt.yscale('linear')

        plt.xticks(range(x_min_range, x_max_range, 5))
        # plt.yticks(range(-5, 5, 1))

        plt.title("Cartesian error Vs Time", fontsize=14, fontweight='bold')

        # add major grid
        plt.grid(b=True, which='major', color='darkgray', linestyle='-', alpha=0.3)

        labels = [clm for clm in
                  self.column_names]  # ['Demonstration', 'Guide trajectory', 'Noisy rollouts', 'GSTOMP output']
        colors = ['blue', 'green', 'purple', 'cyan', 'orange', 'brown', 'pink', 'red', 'olive', 'gray']
        markers = ['o', '*', '.', 'x', '+', '.', '^', '_']

        # plot_data = [self.extractDataFromMatrix(0), self.extractDataFromMatrix(1)]
        draw_start_end_markers = [True, True, True, True]
        ax.set_xlabel('Time (sec)')
        ax.set_ylabel('Cartesian error value (cm) and (rad)')
        ax.set_xlim(x_min_range, x_max_range)  # TODO remove hardcoded limits
        # ax.set_ylim(-5, 5)
        # ax.set_zlim(0.9, 1.6)

        # ax.plot(x_axis, y_axis, label=labels[clm_index], color=colors[clm_index],  marker=markers[clm_index], linewidth=1)
        ax.plot(x_axis, y_axis, label=labels[clm_index], color=colors[clm_index], linewidth=1)

        """
        for traj, label, color, draw_markers in zip(plot_data, labels, colors, draw_start_end_markers):

            if type(traj[0][0]) is not list:
                line, = ax.plot(traj[0], traj[1], traj[2], label=label, color=color, marker='o', linewidth=1)
                if draw_markers:
                    ax.plot([traj[0][0]], [traj[1][0]], [traj[2][0]], marker='o', markersize=9, color=color)
                    ax.plot([traj[0][-1]], [traj[1][-1]], [traj[2][-1]], marker='^', markersize=9, color=color)
            else:
                for ttrajj in traj:
                    line, = ax.plot(ttrajj[0], ttrajj[1], ttrajj[2], color=color, linewidth=1,
                                    alpha=alpha_on_noisy_trajectories)
                line, = ax.plot(ttrajj[0], ttrajj[1], ttrajj[2], color=color, linewidth=1, marker='o',
                                label=label, alpha=alpha_on_noisy_trajectories)

                print ('\033[94m' + str("Hello1") + '\033[0m')
        """
        ax.legend(loc='upper right')

        # plt.show()
        # print ('\033[94m' + legend[i] + str(ii) + " -----Start_pose (x, y, z) = " + str(self.px[i][0])+str(self.py[i][0])+str(self.pz[i][0]) + '\033[0m')
        # print ('\033[94m' + legend[i] + str(ii) + " -----End_pose (x, y, z) = " + str(self.px[i][len(self.px[i])-1])+str(self.py[i][len(self.py[i])-1])+str(self.pz[i][len(self.pz[i])-1]) + '\033[0m')
        # filenames.append(filename)

    # -----------------------------------------------------------------------------------------------------------------------
    """
        @:description Parepare and Visualize 3D trajectory
    """

    def visulizePlot(self):

        alpha_on_noisy_trajectories = 0.1
        print ('\033[1m' + '\033[31m' + "############ " + " Starting export " + "############## " + '\033[0m')
        filenames = []
        for ii in range(0, 81, 10):

            fig = plt.figure()
            ax = fig.add_subplot(1, 1, 1, projection='3d')
            ax.view_init(elev=28, azim=ii)

            labels = [planner for planner in
                      self.planner_names]  # ['Demonstration', 'Guide trajectory', 'Noisy rollouts', 'GSTOMP output']
            colors = ['blue', 'green', 'purple', 'red']

            plot_data = [self.extractDataFromMatrix(matrix_id=matrix_id) for matrix_id in range(0, len(self.mat))]
            # plot_data = [self.extractDataFromMatrix(0), self.extractDataFromMatrix(1)]
            draw_start_end_markers = [True, True, True, True]
            ax.set_xlabel('X axis')
            ax.set_ylabel('Y axis')
            ax.set_zlabel('Z axis')
            # ax.set_xlim(0.4, 1.2) # TODO remove hardcoded limits
            # ax.set_ylim(-0.9, -0.1)
            # ax.set_zlim(0.9, 1.6)

            for traj, label, color, draw_markers in zip(plot_data, labels, colors, draw_start_end_markers):

                if type(traj[0][0]) is not list:
                    line, = ax.plot(traj[0], traj[1], traj[2], label=label, color=color, marker='o', linewidth=1)
                    if draw_markers:
                        ax.plot([traj[0][0]], [traj[1][0]], [traj[2][0]], marker='o', markersize=9, color=color)
                        ax.plot([traj[0][-1]], [traj[1][-1]], [traj[2][-1]], marker='^', markersize=9, color=color)
                else:
                    for ttrajj in traj:
                        line, = ax.plot(ttrajj[0], ttrajj[1], ttrajj[2], color=color, linewidth=1,
                                        alpha=alpha_on_noisy_trajectories)
                    line, = ax.plot(ttrajj[0], ttrajj[1], ttrajj[2], color=color, linewidth=1, marker='o',
                                    label=label, alpha=alpha_on_noisy_trajectories)

                    print ('\033[94m' + str("Hello1") + '\033[0m')

            ax.legend(loc='upper right')
            # filename = os.getcwd() + "/demo_trajectory_" + str(ii) + '_' + label + ".jpeg"
            # fig.savefig(filename, format='eps', dpi=300)
            plt.show()
            # print ('\033[94m' + legend[i] + str(ii) + " -----Start_pose (x, y, z) = " + str(self.px[i][0])+str(self.py[i][0])+str(self.pz[i][0]) + '\033[0m')
            # print ('\033[94m' + legend[i] + str(ii) + " -----End_pose (x, y, z) = " + str(self.px[i][len(self.px[i])-1])+str(self.py[i][len(self.py[i])-1])+str(self.pz[i][len(self.pz[i])-1]) + '\033[0m')
            # filenames.append(filename)

        # Make sure we are not reusing old data
        for traj in plot_data:
            traj = None

        print('\033[1m' + '\033[31m' + "############ " + " Done exporting " + "############## " + '\033[0m')
        print('Exported to files: ' + str(filenames))

    # -----------------------------------------------------------------------------------------------------------------------

    # @description: read data from file line by line and store in matrix format.
    #               self.mat[0] consist of first file data
    #               self.mat[1] consist of second file data, so on.
    # Here store in np.matrix for easy access columns and rows.
    def readDataFromFile(self, file_name):
        path = "output_data/" + str(file_name)
        filedata = pd.read_csv(path, sep=',', skiprows=2, header=0)  # skipping first two line
        self.mat.append(filedata.values)
        # print self.mat
        self.column_names = [key for key in filedata.keys()]


#####################################################################################################################################

if __name__ == '__main__':
    print ('\033[94m' + "Start plotting script..." + '\033[0m')
    SCRIPT = VisualizeResults()
    SCRIPT.Run()
    print ("End ploting script...")
