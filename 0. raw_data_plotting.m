% Copyright (C) 2021 All rights reserved.
%
% Authors:     Seonghyeon Jo <seonghyeonjo@etri.re.kr>
% Date:        Oct, 18, 2021
%
% -------------------------------------------------
% Raw data plotting
%
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%%
clc; clear;
addpath(genpath('.'));

folder_name = "0. raw_data\test1\[20211018-";
timeline = {"1332","1333","1334","1335","1336"};
plotline = {'-m','-r','-g','-b','-k'};

figure(1)
set(gcf,'color','w');
tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');
hold off;
for i =1:5
real_car_quat = table2array(readtable(folder_name+timeline{i}+"]franka_data_cartesian_quat.csv"));
plot3((real_car_quat(:,1)), (real_car_quat(:,2)), (real_car_quat(:,3)),plotline{i},'LineWidth',1.5)
hold on;
grid on;
end

figure(2)
set(gcf,'color','w');
tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');
hold off;
for i =1:5
real_force = table2array(readtable(folder_name+timeline{i}+"]franka_data_force_sensor.csv"));
plot(real_force(:,1),plotline{i},'LineWidth',1.5)
hold on;
grid on;
end

