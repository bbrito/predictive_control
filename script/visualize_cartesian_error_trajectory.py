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
#time = 0   # first column in file
#length = 1 # second column in file
#smooth = 2 # third column in file
import time

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
                self.file_names.append(file_name[0]+str(i))
                self.readDataFromFile(sys.argv[i])
            else:
                # array of file name already filled by file_names, if optional argu are there,than replace file_name with optional argu.
                # i-len(self.file_name) - 1, -1 is because we start for loop from 1 and ayyry element start from 0
                #self.file_names[i-len(self.file_names) - 1] = sys.argv[i]
                self.file_names.append(sys.argv[i])

        self.file_names = [file_name.replace("_", ":") for file_name in self.file_names]


        #for planner in self.planner_names:
        #    print ('\033[94m' + planner + '\033[0m')
        #print self.mat[0][2:]
        #self.visulizePlot()
        #self.visualizeTrajectory()



        for matrix_id in range(0, len(self.mat)):

            print ('\033[1m' + '\033[31m' + "############ " + " Starting export " + "############## " + '\033[0m')

            plt.figure(matrix_id)
            figure = plt.figure()
            plt.rc('axes', axisbelow=True) #draw grid line behind plot
            time_index = self.column_names.index('time')

            # remove entry of blank or zero values
            self.column_names = filter(lambda a: a != 'Unnamed: 0', self.column_names)

            # extract time data and set that as our x axis
            time_axis = self.extractColumData(matrix_id=matrix_id, colum_index=time_index)

            # extract range of plots
            x_min = int(math.ceil(min(time_axis))); x_max = int(math.ceil(max(time_axis)))
            y_min, y_max = self.findRangeOfMatrix(matrix_index=matrix_id)

            # generate plot of position and/or orientation
            # position = 1, orientation = 4, position + orientation = 1
            start_range = [1, 4, 1]

            # position = 1, orientation = 4, position + orientation = 1
            stop_range = [4, 8, len(self.column_names)]
            
            for clm_index in range(1, len(self.column_names)): #len(self.column_names)
                self.data_to_plot = []
                self.data_to_plot = self.extractColumData(matrix_id=matrix_id, colum_index=clm_index)

                self.visualize2DPlot(x_axis=time_axis,y_axis=self.data_to_plot, clm_index=clm_index-1,
                                     x_min_range=x_min, x_max_range=x_max, y_min_range=y_min, y_max_range=y_max,fig=figure)

            filename = os.getcwd() + "/plots/" + "cartesian" + '_' + self.file_names[matrix_id] + '_' + str(time.strftime("%d-%m-%Y-%H:%M:%S")) + '.png'
            figure.savefig(filename, format='png', dpi=300)

            #print('\033[1m' + '\033[31m' + "############ " + " Done exporting " + "############## " + '\033[0m')
            print ('\033[1m' + '\033[92m' + "######### " + "Exported to files: " + str(filename)+ " ###########" + '\033[0m')

            #plt.show()

    def findRangeOfMatrix(self,matrix_index):

        y_min = []; y_max = []
        for clm in range(1, len(self.column_names)-1):
            clm_data = []
            for data in self.mat[matrix_index][:,clm]:
                clm_data.append(data)
            y_min.append(int(math.ceil(min(clm_data))))
            y_max.append(int(math.ceil(max(clm_data))))

        return (min(y_min),max(y_max))


    def extractColumData(self, matrix_id, colum_index):
        clm_data = []

        for data in self.mat[matrix_id][:, colum_index]:
            clm_data.append(data)

        return (clm_data)
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


    def visualize2DPlot(self, x_axis, y_axis, clm_index, x_min_range, x_max_range, y_min_range, y_max_range,fig):
        filenames = []


        ax = fig.add_subplot(1,1,1)

        plt.legend()

        #set scale
        plt.xscale('linear')
        plt.yscale('linear')

        plt.xticks(range(x_min_range, x_max_range, 5))
        #plt.yticks(range(-5, 5, 1))

        plt.title("Cartesian error Vs Time",fontsize=14, fontweight='bold')

        # add major grid
        plt.grid(b=True, which='major', color='darkgray', linestyle='-', alpha=0.3)

        labels = [clm for clm in self.column_names]  # ['Demonstration', 'Guide trajectory', 'Noisy rollouts', 'GSTOMP output']
        colors = ['blue', 'green', 'purple', 'cyan', 'orange', 'brown', 'pink', 'red', 'olive', 'gray']
        markers = ['o', '*', '.', 'x', '+', '.', '^', '_']

        # plot_data = [self.extractDataFromMatrix(0), self.extractDataFromMatrix(1)]
        draw_start_end_markers = [True, True, True, True]
        ax.set_xlabel('Time (sec)')
        ax.set_ylabel('Cartesian error (cm) and (rad)')
        ax.set_xlim(x_min_range, x_max_range) # TODO remove hardcoded limits
        #ax.set_ylim(-0.5, 1.2)
        # ax.set_zlim(0.9, 1.6)

        #ax.plot(x_axis, y_axis, label=labels[clm_index], color=colors[clm_index],  marker=markers[clm_index], linewidth=1)
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

        #plt.show()
        # print ('\033[94m' + legend[i] + str(ii) + " -----Start_pose (x, y, z) = " + str(self.px[i][0])+str(self.py[i][0])+str(self.pz[i][0]) + '\033[0m')
        # print ('\033[94m' + legend[i] + str(ii) + " -----End_pose (x, y, z) = " + str(self.px[i][len(self.px[i])-1])+str(self.py[i][len(self.py[i])-1])+str(self.pz[i][len(self.pz[i])-1]) + '\033[0m')
        # filenames.append(filename)


    #-----------------------------------------------------------------------------------------------------------------------
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

            labels = [planner for planner in self.planner_names] #['Demonstration', 'Guide trajectory', 'Noisy rollouts', 'GSTOMP output']
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
