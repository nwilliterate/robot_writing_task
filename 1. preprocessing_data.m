% Copyright (C) 2021 All rights reserved.
%
% Authors:     Seonghyeon Jo <seonghyeonjo@etri.re.kr>
% Date:        Oct, 18, 2021
%
% -------------------------------------------------
% Franka Robotics
% demo data preprocessing
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

% normalize
for i=1:7
    real_x(:,i) = (real_x(:,i)- min(real_x(:,i))) / (max(real_x(:,i)) - min(real_x(:,i)));
end
real_f(:,1) = (real_f(:,1)- min(real_f(:,1))) / (max(real_f(:,1)) - min(real_f(:,1)));

x = [real_x real_f];
sample_size = length(x);
temp_t = [1];
temp_x = round(x(1,:),3);
num = 1;
threshold = 0.05;
for i=1:sample_size
    if(norm(temp_x(num,1:8) - x(i,1:8)) > threshold) 
        num = num + 1;
        temp_t(num) = i;
        temp_x(num,:) = round(x(i,:),3);
    end
end

% Plotting
figure(1)
set(gcf,'color','w');
tiledlayout(8,1,'TileSpacing','Compact','Padding','Compact');
sample_size = length(real_x);
t = 1:sample_size;
t = t*0.001;
for i=1:7

    nexttile
    hold off
    plot(t, real_x(:,i),'-k');
    hold on;
    plot(temp_t*0.001,(temp_x(:,i)),'ob','LineWidth',1,'MarkerSize', 5);
end
nexttile
hold off
plot(t, real_f(:,1),'-k');
hold on;
plot(temp_t*0.001,(temp_x(:,8)),'ob','LineWidth',1,'MarkerSize', 5);


% Save data
T = table(temp_x);

round(sample_size/num, 0)
file_name = "1. preprocessing_data\test1\train_data_"+ round(sample_size/num,0) + "_" + threshold+ ".csv";
writetable(T,file_name,'Delimiter',',')