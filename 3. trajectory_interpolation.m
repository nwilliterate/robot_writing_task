% Copyright (C) 2021 All rights reserved.
%
% Authors:     Seonghyeon Jo <seonghyeonjo@etri.re.kr>
% Date:        Oct, 18, 2021
%
% -------------------------------------------------
% Trajectory interpolation
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%%
clc; clear;
addpath(genpath('.'));

% load data
folder_name = "0.raw_data\test1\[20211018-";
timeline = {"1332","1333","1334","1335","1336"};

real_x = [];
real_f = [];
for i =1:5
    real_car_quat = table2array(readtable(folder_name+timeline{i}+"]franka_data_cartesian_quat.csv"));
    real_force = table2array(readtable(folder_name+timeline{i}+"]franka_data_force_sensor.csv"));

    real_x = [real_x; real_car_quat];
    real_f = [real_f; real_force(:,1)];
end

trained_x = table2array(readtable("2. learning_data\test1\feature_data_lstm_test_data.csv"));
trained_x(1,:)= [];
%
% unnormalize
for i=1:7
    trained_x(:,i) = trained_x(:,i) * (max(real_x(:,i)) - min(real_x(:,i))) + min(real_x(:,i));
end
trained_x(:,8) = trained_x(:,8) * (max(real_f(:,1)) - min(real_f(:,1))) + min(real_f(:,1));

sample_size = length(trained_x);
sim_period = 0.001;
sample_time = 0.077;

ref_x = []; ref_dx = []; ref_ddx = [];
temp_s = []; temp_sd = []; temp_sdd = [];
for i=1:sample_size-1
    for j=1:8
        [s,sd,sdd] = traj(trained_x(i,j), trained_x(i+1,j), 0:sim_period:sample_time);
        temp_s(:,j) = s;
        temp_sd(:,j) = sd;
        temp_sdd(:,j) = sdd;
    end
    ref_x = [ref_x; temp_s];
    ref_dx = [ref_dx; temp_sd];
    ref_ddx = [ref_ddx; temp_sdd];
end

T = table(ref_x);
file_name = "3. trajectory_data\test1\trajectory.csv";
writetable(T, file_name, 'Delimiter',',')  

% plotting
figure(1)
set(gcf,'color','w');
tiledlayout(8,1,'TileSpacing','Compact','Padding','Compact');
plotline = {'-m','-r','-g','-b','-k'};
for i=1:8
nexttile
plot(ref_x(:,i),'-b','LineWidth',1.5')
grid on;
end


figure(2)
set(gcf,'color','w');
hold off;
for i =1:5
real_car_quat = table2array(readtable(folder_name+timeline{i}+"]franka_data_cartesian_quat.csv"));
plot3((real_car_quat(:,1)), (real_car_quat(:,2)), (real_car_quat(:,3)),':k','LineWidth',1)
hold on;
end
plot3(ref_x(:,1), ref_x(:,2),ref_x(:,3),'--k','LineWidth',1.5')
grid on;


function [p, pd, pdd]= traj(q0, q1, t)
tf = max(t(:));
V = (q1-q0)/tf;
p = zeros(length(t), 1);
pd = p;
pdd = p;
for i = 1:length(t)
    tt = t(i);
    p(i) = (q1+q0-V*tf)/2 + V*tt;
    pd(i) = V;
    pdd(i) = 0;
end
end