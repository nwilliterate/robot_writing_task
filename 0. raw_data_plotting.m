% Copyright (C) 2021 All rights reserved.
%
% Authors:      Seonghyeon Jo <seonghyeonjo@etri.re.kr>
% Date:         Oct, 18, 2021
% Last Updated: Des, 13, 2021
%
% -------------------------------------------------
% Kinesthetic Teaching Data Plot.
%
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%%
clc; clear;
addpath(genpath('.'));
task_index = 3; fig_index = 2;

if (fig_index == 1)
    fig_type = ".eps";
elseif (fig_index == 2)
    fig_type = ".png";
elseif (fig_index == 3)
    fig_type = ".jpg";
end

if (task_index == 1)
    folder_name = "0. raw_data\task1\[20211018-";
    timeline = {"1332","1333","1334","1335","1336"};
elseif (task_index == 2)
    folder_name = "0. raw_data\task2\[20211022-";
    timeline = {"1404","1405","1407","1409","1410"};
elseif (task_index == 3)
    task_folder = "task3";
    folder_name = "0. raw_data\task3\[20211213-";
    timeline = {"185443","185530","185622","185709","185754"};
end


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
xlabel('P_x(m)');
ylabel('P_y(m)');
zlabel('P_z(m)');
if (fig_index == 1)
    saveas(gcf,"fig\teaching_result1" + fig_type, 'epsc');
else
    saveas(gcf,"fig\teaching_result1" + fig_type);
end

figure(2)
set(gcf,'color','w');
tiledlayout(3,3,'TileSpacing','Compact','Padding','Compact');
ylabel_name = {"P_x(m)", "P_y(m)", "P_z(m)", "R_x", "R_y", "R_z", "R_w", "F_x(N)"};
for j =1:7
    nexttile
    hold off;
    for i =1:5
        real_car_quat = table2array(readtable(folder_name+timeline{i}+"]franka_data_cartesian_quat.csv"));
        t = 0.001:0.001:length(real_car_quat)*0.001;
        plot(t, real_car_quat(:,j),plotline{i},'LineWidth',1.5)
        hold on;
        grid on;
    end
    ylabel(ylabel_name{j});
%     if j < 4
%         ylabel("x_"+num2str(j)+"(m)");
%     else
%         ylabel("x_"+num2str(j)+"(rad)");
%     end
    xlabel('time(s)');
end
nexttile
for i =1:5
    real_force = table2array(readtable(folder_name+timeline{i}+"]franka_data_force_sensor.csv"));
    t = 0.001:0.001:length(real_force)*0.001;
    plot(t, real_force(:,1),plotline{i},'LineWidth',1.5)
    hold on;
    grid on;
    ylabel(ylabel_name{8});
    xlabel('time(s)');
end

if (fig_index == 1)
    saveas(gcf,"fig\teaching_result2" + fig_type, 'epsc');
else
    saveas(gcf,"fig\teaching_result2" + fig_type);
end
