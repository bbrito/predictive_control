#!/usr/bin/python
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

import math
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys, time
import re
time = 0   # first column in file
length = 1 # second column in file
smooth = 2 # third column in file

def makefilename(name):
  return name.strip().replace(' ','_').replace('(','').replace(')','')

class VisualizeResults:
    """"
        Visualization results of benchmarking
        input: file names
        output: store/show plot
    """
#-----------------------------------------------------------------------------------------------------------------------
    # @description: Default constructor of Plot Class
    def __init__(self):
        self.mat = []
        self.file_names = []
        self.column_names = []
        self.data_to_plot = []
#-----------------------------------------------------------------------------------------------------------------------
    # @description: Read data from file and separate into column wise and plot column data
    def Run(self):

        # store all files data into array called self.mat
        self.task = sys.argv[1]
        for i in range(1,len(sys.argv)):
            if sys.argv[i].rfind('.') != -1:    #  If .csv or .txt is not pass than consider as that xlable
                file_name = re.split('\d+',sys.argv[i])
                self.file_names.append(file_name[0])
                self.readDataFromFile(sys.argv[i])
            else:
                # array of file name already filled by file_names, if optional argu are there,than replace file_name with optional argu.
                # i-len(self.file_name) - 1, -1 is because we start for loop from 1 and ayyry element start from 0
                #self.file_names[i-len(self.file_names) - 1] = sys.argv[i]
                self.file_names.append(sys.argv[i])

        self.planner_names = [file_name.replace("_", "\n") for file_name in self.file_names]

        for planner in self.planner_names:
            print ('\033[94m' + planner + '\033[0m')
        #print self.mat[0][2:]
        self.visulizePlot()
        #self.visualizeTrajectory()

#-----------------------------------------------------------------------------------------------------------------------
    """
        @:description Extract data from matrix which store file by file
        @:param matrix_id File number from extract data
    """
    def extractDataFromMatrix(self, matrix_id):
        x = []; y = []; z = []

        for px in self.mat[matrix_id][2:, 1]:
            x.append(float(px))
        for py in self.mat[matrix_id][2:, 2]:
            y.append(float(py))
        for pz in self.mat[matrix_id][2:, 3]:
            z.append(float(pz))
        return (x, y, z)

#-----------------------------------------------------------------------------------------------------------------------
    """
        @:description HardCoded function used to visualize trajectory
    """
    def visualizeTrajectory(self):
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1, projection='3d')

        x = []; y = []; z = []
        x,y,z = self.extractDataFromMatrix(0)
        print x, y, z
        ax.plot(x,y,z)
        #ax.plot(x,y,z,linewidth=1)
        plt.show()

#-----------------------------------------------------------------------------------------------------------------------
    """
        @:description Parepare and Visualize 3D trajectory
    """
    def visulizePlot(self):

        alpha_on_noisy_trajectories = 0.1
        print ('\033[1m' + '\033[31m' + "############ " + " Starting export " + "############## " + '\033[0m')
        filenames = ""
        for ii in range(0, 81, 10):

            fig = plt.figure()
            ax = fig.add_subplot(1, 1, 1, projection='3d')
            ax.view_init(elev=28, azim=ii)

            labels = ["pd_trajectory"] #[planner for planner in self.planner_names] #['Demonstration', 'Guide trajectory', 'Noisy rollouts', 'GSTOMP output']
            colors = ['blue', 'green', 'purple', 'red']

            plot_data = [self.extractDataFromMatrix(matrix_id=matrix_id) for matrix_id in range(0, len(self.mat))]
            #plot_data = [self.extractDataFromMatrix(0), self.extractDataFromMatrix(1)]
            draw_start_end_markers = [True, True, True, True]
            ax.set_xlabel('X axis')
            ax.set_ylabel('Y axis')
            ax.set_zlabel('Z axis')
            # ax.set_xlim(0.4, 1.2) # TODO remove hardcoded limits
            # ax.set_ylim(-0.9, -0.1)
            # ax.set_zlim(0.9, 1.6)

            for traj, label, color, draw_markers in zip(plot_data, labels, colors, draw_start_end_markers):

                #x_min = int(math.ceil(min(traj[0]))); x_max = int(math.ceil(max(traj[0])))
                #y_min = int(math.ceil(min(traj[1]))); y_max = int(math.ceil(max(traj[1])))
                #z_min = int(math.ceil(min(traj[2]))); z_max = int(math.ceil(max(traj[2])))

                #plt.xticks(range(x_min, x_max, 8))
                #plt.yticks(range(y_min, y_max, 8))

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
            #print os.getcwd()
            filename = os.getcwd() + "/plots/" + "pd_trajectory:" + str(ii) + '_' + str(time.strftime("%d-%m-%Y-%H:%M:%S")) + '.png'
            filenames = filename
            fig.savefig(filename, format='png', dpi=300)
            #filename = os.getcwd() + "/demo_trajectory_" + str(ii) + '_' + label + ".jpeg"
            #fig.savefig(filename, format='eps', dpi=300)
            plt.show()
            # print ('\033[94m' + legend[i] + str(ii) + " -----Start_pose (x, y, z) = " + str(self.px[i][0])+str(self.py[i][0])+str(self.pz[i][0]) + '\033[0m')
            # print ('\033[94m' + legend[i] + str(ii) + " -----End_pose (x, y, z) = " + str(self.px[i][len(self.px[i])-1])+str(self.py[i][len(self.py[i])-1])+str(self.pz[i][len(self.pz[i])-1]) + '\033[0m')
            #filenames.append(filename)

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
        filedata = pd.read_csv(path, sep=',',skiprows=2, header=0)  # skipping first two line
        self.mat.append(filedata.values)
        #print self.mat
        self.column_names = [key for key in filedata.keys()]
#####################################################################################################################################

if __name__ == '__main__':
    print ('\033[94m' + "Start plotting script..." + '\033[0m')
    SCRIPT = VisualizeResults()
    SCRIPT.Run()
    print ("End ploting script...")
    print ( '\033[1m' +'\033[92m' + "######### "+ "output plots stored path: plots/XYZ.eps" + " ########### " + '\033[0m')