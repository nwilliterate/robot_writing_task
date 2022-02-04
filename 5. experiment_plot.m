% Copyright (C) 2021 All rights reserved.
% Authors:      Seonghyeon Jo <seonghyeonjo@etri.re.kr>
%
% Date:         Oct, 18, 2021
% Last Updated: Feb, 04, 2022
%
% -------------------------------------------------
% Experimental Results in Real Environment
% 
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%%
clc; clear;
addpath(genpath('.'));

fig_index = 2;
if (fig_index == 1)
    fig_type = ".eps";
elseif (fig_index == 2)
    fig_type = ".png";
elseif (fig_index == 3)
    fig_type = ".jpg";
end

task_index = 3;
if (task_index == 1)
    task_folder = "task1";
elseif (task_index == 2)
    task_folder = "task2";
elseif (task_index == 3)
    task_folder = "task3";
    timeline = "\[20211213-2017]";
end


% reference joint catersian
ref_x = table2array(readtable("3. trajectory_data\"+task_folder+"\trajectory_pose_20s.csv"));
ref_f = table2array(readtable("3. trajectory_data\"+task_folder+"\trajectory_force_20s.csv"));

% reference joint catersian
test1_x = table2array(readtable("5. experiment_data\"+task_folder+ timeline+"franka_data_cartesian_pose_data.csv"));
test1_f = table2array(readtable("5. experiment_data\"+task_folder+ timeline+"franka_data_force_sensor.csv"));
% test2_x = table2array(readtable("5. experiment_data\"+task_folder+ timeline+"franka_data_cartesian_pose_data.csv"));
% test2_f = table2array(readtable("5. experiment_data\"+task_folder+ timeline+"franka_data_force_sensor.csv"));

% Plotting
fig = figure(1);
tiledlayout(3,3,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
t = (1:length(test1_x))*0.001;
t2 = (1:length(ref_x))*0.001;

ylabel_name = {"P_x(m)", "P_y(m)", "P_z(m)", "R_x", "R_y", "R_z", "R_w", "F_x(N)"};
for i=1:7
    ax = nexttile;
    hold off
    plot(t, test1_x(:,i),'-k','LineWidth',1.5')
    hold on;
    plot(t2, ref_x(:,i),'-r','LineWidth',1.5')
    
    xlim([0 length(t)*0.001])
    ylim([ax.YLim(1)-0.05 ax.YLim(2)+0.05])
    grid on;
    ylabel(ylabel_name{i}, 'FontSize', 10);
    xlabel('time(s)');
end
ax = nexttile;
hold off
plot(t, test1_f(:,1),'-k','LineWidth',1.5')
hold on;
plot(t2, ref_f(:,1),'-r','LineWidth',1.5')
grid on;
xlim([0 length(t)*0.001])
ylim([ax.YLim(1)-2 ax.YLim(2)+2])
ylabel(ylabel_name{8}, 'FontSize', 10);
xlabel('time(s)');
lgd = legend('real x', 'gen x');
lgd.Layout.Tile = 9;
lgd.FontSize = 10;
if (fig_index == 1)
    saveas(gcf,"fig\experiment_result1" + fig_type, 'epsc');
else
    saveas(gcf,"fig\experiment_result1" + fig_type);
end

% 
fig=figure(2);
set(gcf,'color','w');
tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');
nexttile
hold off
plot3(test1_x(:,1), test1_x(:,2),test1_x(:,3),'-k','LineWidth',1.5')
hold on;
% plot3(test2_x(:,1), test2_x(:,2),test2_x(:,3),'-r','LineWidth',1.5')
plot3(ref_x(:,1), ref_x(:,2),ref_x(:,3),'-r','LineWidth',1.5')
ax=gca;
scale = 0.05;
axis([ax.XLim(1)-scale ax.XLim(2)+scale ax.YLim(1)-scale ax.YLim(2)+scale ax.ZLim(1)-scale ax.ZLim(2)+scale]);
grid on;
xlabel('P_x(m)','FontSize', 12);
ylabel('P_y(m)','FontSize', 12);
zlabel('P_z(m)','FontSize', 12);
grid on;
lgd = legend('real x', 'gen x','Location','northeast');
if (fig_index == 1)
    saveas(gcf,"fig\experiment_result2" + fig_type, 'epsc');
else
    saveas(gcf,"fig\experiment_result2" + fig_type);
end