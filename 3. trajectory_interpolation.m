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

task_index = 2; 
if (task_index == 1)
    task_folder = "task1";
    folder_name = "0. raw_data\task1\[20211018-";
    timeline = {"1332","1333","1334","1335","1336"};
elseif (task_index == 2)
    task_folder = "task2";
    folder_name = "0. raw_data\task2\[20211022-";
    timeline = {"1404","1405","1407","1409","1410"};
end

sim_period = 0.001;
sample_time = 0.064;

real_x = [];
real_f = [];
for i =1:5
    real_car_quat = table2array(readtable(folder_name+timeline{i}+"]franka_data_cartesian_quat.csv"));
    real_force = table2array(readtable(folder_name+timeline{i}+"]franka_data_force_sensor.csv"));

    real_x = [real_x; real_car_quat];
    real_f = [real_f; real_force(:,1)];
end


trained_x = table2array(readtable("2. learning_data\"+task_folder+"\feature_data_lstm_test_data.csv"));
trained_x(1,:)= [];
%
% unnormalize
for i=1:7
    trained_x(:,i) = trained_x(:,i) * (max(real_x(:,i)) - min(real_x(:,i))) + min(real_x(:,i));
end
trained_x(:,8) = trained_x(:,8) * (max(real_f(:,1)) - min(real_f(:,1))) + min(real_f(:,1));

sample_size = length(trained_x);


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

% round
for i=1:8
    ref_x(:,i)= round(ref_x(:,i),6);
end
T = table(ref_x(:,1:7));
file_name = "3. trajectory_data\"+task_folder+"\trajectory_pose.csv";
writetable(T, file_name, 'Delimiter',',','WriteVariableNames',0)  

T = table([ref_x(:,8) zeros(length(ref_x(:,8)),5)]);
file_name = "3. trajectory_data\"+task_folder+"\trajectory_force.csv";
writetable(T, file_name, 'Delimiter',',','WriteVariableNames',0)  


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
plot3(ref_x(:,1), ref_x(:,2),ref_x(:,3),'.k','LineWidth',1.5')
plot3(ref_x(:,1)-ref_x(:,8)*0.001, ref_x(:,2),ref_x(:,3),'.r','LineWidth',1.5')
grid on;
ax = gca;
r = 0.05;
axis([ax.XLim(1)-r ax.XLim(2)+r ax.YLim(1)-r ax.YLim(2)+r ax.ZLim(1)-r ax.ZLim(2)+r])

figure(3)
set(gcf,'color','w');
hold off;
idx = abs(ref_x(:,8)) > 2;
for i =1:5
real_car_quat = table2array(readtable(folder_name+timeline{i}+"]franka_data_cartesian_quat.csv"));
plot3((real_car_quat(:,1)), (real_car_quat(:,2)), (real_car_quat(:,3)),':k','LineWidth',1)
hold on;
end
plot3(ref_x(idx,1), ref_x(idx,2),ref_x(idx,3),'.k','LineWidth',1.5')
% plot3(ref_x(:,1)-ref_x(:,8)*0.001, ref_x(:,2),ref_x(:,3),'.r','LineWidth',1.5')
grid on;
ax = gca;
r = 0.05;
axis([ax.XLim(1)-r ax.XLim(2)+r ax.YLim(1)-r ax.YLim(2)+r ax.ZLim(1)-r ax.ZLim(2)+r])



function [p, pd, pdd]= traj(q0, q1, t)
tf = max(t(:));
V = (q1-q0)/tf;
p = q0; %zeros(length(t), 1);
pd = V;
pdd = 0;
for i = 2:length(t)
    p(i) =  p(i-1)+V*0.001;
    pd(i) = V;
    pdd(i) = 0;
end
end