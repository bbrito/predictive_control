%% install ROSBAG READER
addpath('/home/bdebrito/Documents/MPCC_ICRA2018/matlab_rosbag-0.5.0-linux64')

clear rosbag_wrapper;
clear ros.Bag;
clear all

%% Load a bag and get information about it
% Using load() lets you auto-complete filepaths.
% bag = ros.Bag.load('2018-09-11-13-18-00.bag');
bag = ros.Bag.load('Data/2018-09-11-15-01-56.bag');
%bag = ros.Bag.load('Data/2018-09-11-15-33-48.bag');
bag.info()

%% Read all messages on a few topics
topic1 = '/Robot_1/pose';
topic2 = '/controller_feedback';

%% Read messages incrementally
bag.resetView(topic2);

message_count = 1;

time = [];

robot_state_ekf = [];
robot_state_mocap = [];
robot_state_mocap_rotated = [];
reference_traj_ekf=[];
person1 =[]
person2 =[]
person1_rotated =[]
person2_rotated =[]
s_sim=[]
obsta_0 = 0;
obstb_0 = 0;
obsth_0 = [];
obsta_1 = 0;
obstb_1 = 0;
obsth_1 = [];
i=1;

while bag.hasNext();
[msg, meta] = bag.read();

robot_state_ekf(1,i) = msg.prediction_horizon.poses(1).pose.position(1);
robot_state_ekf(2,i) = msg.prediction_horizon.poses(1).pose.position(2);
robot_state_ekf(3,i) = msg.prediction_horizon.poses(1).pose.position(3);

person1(1,i) =msg.obstx_0;
person1(2,i) =msg.prediction_horizon.poses(1).pose.position(2)-sqrt(-(msg.obstx_0-msg.prediction_horizon.poses(1).pose.position(1))^2+msg.obstacle_distance1^2);
person2(1,i) =msg.obstx_1;
person2(2,i) =msg.prediction_horizon.poses(1).pose.position(2)-sqrt(-(msg.obstx_1-msg.prediction_horizon.poses(1).pose.position(1))^2+msg.obstacle_distance2^2);
person1_rotated(1,i) =msg.obsty_0;
person1_rotated(2,i) =-msg.obstx_0;
person2_rotated(1,i) =msg.obsty_1;
person2_rotated(2,i) =-msg.obstx_1;
obsth_0(1,i) = msg.obsth_0;
obsth_1(1,i) = msg.obsth_1;

%s_sim(i) = msg.prediction_horizon.poses(1).pose.position(4);
i=i+1;
end
obsta_0 = msg.obsta_0;
obstb_0 = msg.obstb_0;
obsta_1 = msg.obsta_1;
obstb_1 = msg.obstb_1;

bag.resetView(topic2);
[msg, meta] = bag.read();
for i=1:1:length(msg.reference_path.poses)

reference_traj_sim(1,i) = msg.reference_path.poses(i).pose.position(1);
reference_traj_sim(2,i) = msg.reference_path.poses(i).pose.position(2);

end

bag.resetView(topic1);
while bag.hasNext();
[msg, meta] = bag.read();

robot_state_mocap(1,i) = msg.pose.position(1);
robot_state_mocap(2,i) = msg.pose.position(2);
robot_state_mocap(3,i) = msg.pose.orientation(3);

%s_sim(i) = msg.prediction_horizon.poses(1).pose.position(4);
i=i+1;
end

save("people.mat",'person1','person2')