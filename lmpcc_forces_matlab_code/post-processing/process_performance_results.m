clear rosbag_wrapper;
clear ros.Bag;
clear all
clc
%% Load a bag and get information about it
% Using load() lets you auto-complete filepaths.
% bag = ros.Bag.load('2018-09-11-13-18-00.bag');
%bag = ros.Bag.load('../../bags/2020-03-31-16-02-57.bag'); % with interaction decaying countour cost
%bag = ros.Bag.load('../../bags/2020-03-31-16-39-35.bag'); % no interaction decaying countour cost
%bag = ros.Bag.load('../../bags/2020-03-31-17-44-19.bag'); % no interaction lmpcc baseline
%bag = ros.Bag.load('../../bags/2020-03-31-18-27-23.bag'); % no interaction lmpcc baseline
%bag = ros.Bag.load('../../bags/2020-03-31-20-09-11.bag'); % no interaction lmpcc baseline with wall boundaries 2 meters 
%bag = ros.Bag.load('../../bags/2020-03-31-20-52-25.bag'); % no interaction lmpcc baseline with wall boundaries 2 meters minus robot radius
%bag = ros.Bag.load('../../bags/2020-04-01-15-00-17.bag'); % 12 pedestrians no interaction lmpcc baseline with wall boundaries 2 meters minus robot radius 
%bag = ros.Bag.load('../../bags/2020-04-01-18-18-09.bag'); % 6 pedestrians no interaction lmpcc baseline with wall boundaries 2 meters minus robot radius 
%bag = ros.Bag.load('../../bags/2020-04-01-19-08-37.bag'); % 6 pedestrians with interaction lmpcc baseline with wall boundaries 2 meters minus robot radius
%bag = ros.Bag.load('../../bags/2020-04-01-20-10-24.bag'); % 6 pedestrians with interaction lmpcc discounted cost with wall boundaries 2 meters minus robot radius 
bag = ros.Bag.load('../../bags/2020-04-01-21-19-59.bag'); % 6 pedestrians no interaction lmpcc discounted cost with wall boundaries 2 meters minus robot radius 
bag.info()

%% Read all messages on a few topics
topic1 = '/controller_feedback';
topic2 = '/ellipse_objects_feed';
topic3 = '/robot_collision_space';
%% Read messages incrementally
bag.resetView(topic1);

message_count = 1;

computation_time = [];
collisions = [];

i = 1;
%% Process feedback msg
while bag.hasNext();
[msg, meta] = bag.read();

computation_time(i) = msg.computation_time;
collisions(i) = msg.collision;

i=i+1;
end

%% Statistics
mean_computation_time = mean(computation_time)
std_computation_time = std(computation_time)
min_computation_time = min(computation_time)
max_computation_time = max(computation_time)
median_computation_time = median(computation_time)

%% Process Number of collisions msg

number_of_collisions = 0;
width = 0.4;
length =0.4;
r_discs_ = sqrt(((width)^2+(length)^2)/2.0);
bag.resetView(topic1);
while bag.hasNext();
[msg, meta] = bag.read();
    
    min_dist = 1000;
    ped_radius = max([msg.obsta_0,msg.obstb_0]); % assuming the same radius for all
    dist0 = sqrt((msg.obstx_0-msg.computed_control.angular(1))^2+(msg.obsty_0-msg.computed_control.angular(2))^2);
    dist1 = sqrt((msg.obstx_1-msg.computed_control.angular(1))^2+(msg.obsty_1-msg.computed_control.angular(2))^2);
    dist2 = sqrt((msg.obstx_2-msg.computed_control.angular(1))^2+(msg.obsty_2-msg.computed_control.angular(2))^2);
    dist3 = sqrt((msg.obstx_3-msg.computed_control.angular(1))^2+(msg.obsty_3-msg.computed_control.angular(2))^2);
    dist4 = sqrt((msg.obstx_4-msg.computed_control.angular(1))^2+(msg.obsty_4-msg.computed_control.angular(2))^2);
    dist5 = sqrt((msg.obstx_5-msg.computed_control.angular(1))^2+(msg.obsty_5-msg.computed_control.angular(2))^2);
    min_dist = min([min_dist,dist0,dist1,dist2,dist3,dist4,dist5]);

    if min_dist<max([ped_radius,r_discs_])
        number_of_collisions=number_of_collisions+1;
    end
end

display("NUmber of collisions");
sum(number_of_collisions)