% Copyright (C) 2021 All rights reserved.
%
% Authors:      Seonghyeon Jo <seonghyeonjo@etri.re.kr>
% Date:         Oct, 18, 2021
% Last Updated: Des, 13, 2021
%
% -------------------------------------------------
% Franka Robotics
% Demo Data Preprocessing
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%%
clc; clear;
addpath(genpath('.'));

task_index = 3;

if (task_index == 1)
    task_folder = "task1";
    folder_name = "0. raw_data\task1\[20211018-";
    timeline = {"1332","1333","1334","1335","1336"};
elseif (task_index == 2)
    task_folder = "task2";
    folder_name = "0. raw_data\task2\[20211022-";
    timeline = {"1404","1405","1407","1409","1410"};
elseif (task_index == 3)
    task_folder = "task3";
    folder_name = "0. raw_data\task3\[20211213-";
    timeline = {"185443","185530","185622","185709","185754"};
end

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
tiledlayout(3,3,'TileSpacing','Compact','Padding','Compact');
sample_size = length(real_x);
t = 1:sample_size;
t = t*0.001;
for i=1:7
    ax = nexttile;
    hold off
    plot(t, real_x(:,i),'-k');
    hold on;
    plot(temp_t*0.001,(temp_x(:,i)),'ob','LineWidth',1,'MarkerSize', 2);
    xlim([0 sample_size*0.001]);
    if i < 4
        ylabel("x_"+num2str(i)+"(m)");
    else
        ylabel("x_"+num2str(i)+"(rad)");
    end
    xlabel('time(s)');
end
nexttile
hold off
plot(t, real_f(:,1),'-k');
hold on;
plot(temp_t*0.001,(temp_x(:,8)),'ob','LineWidth',1,'MarkerSize', 2);
xlim([0 sample_size*0.001]);
ylabel("x_"+num2str(8)+"(N)");
xlabel('t(s)');
saveas(gcf,'fig\preprocessing_result1.eps','epsc');

figure(2)
set(gcf,'color','w');
tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');
hold off;
% for i =1:5
%     real_car_quat = table2array(readtable(folder_name+timeline{i}+"]franka_data_cartesian_quat.csv"));
plot3((real_x(:,1)), (real_x(:,2)), (real_x(:,3)),'--k','LineWidth',1.5)
hold on;
plot3((temp_x(:,1)), (temp_x(:,2)), (temp_x(:,3)),'-b','LineWidth',1.5)
grid on;
% end
xlabel('x_1(m)');
ylabel('x_2(m)');
zlabel('x_3(m)');
saveas(gcf,'fig\preprocessing_result2.eps','epsc');


% Save data
T = table(temp_x);
round(sample_size/num, 0)
file_name = "1. preprocessing_data\"+task_folder+"\train_data_"+ round(sample_size/num,0) + "_" + threshold+ ".csv";
writetable(T,file_name,'Delimiter',',');