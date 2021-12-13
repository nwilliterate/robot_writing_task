% Copyright (C) 2021 All rights reserved.
%
% Authors:      Seonghyeon Jo <seonghyeonjo@etri.re.kr>
% Date:         Oct, 18, 2021
% Last Updated: Des, 13, 2021
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
ref_x = table2array(readtable("3. trajectory_data\"+task_folder+"\trajectory_pose.csv"));
ref_f = table2array(readtable("3. trajectory_data\"+task_folder+"\trajectory_force.csv"));

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
for i=1:7
    ax = nexttile;
    hold off
    plot(t, test1_x(:,i),'-k','LineWidth',1.5')
    hold on;
    plot(t2, ref_x(:,i),'-r','LineWidth',1.5')
    
    xlim([0 length(t)*0.001])
    ylim([ax.YLim(1)-0.05 ax.YLim(2)+0.05])
    grid on;
    ylabel("x_"+num2str(i)+"");
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
ylabel("x_"+num2str(8)+"");
xlabel('time(s)');
saveas(gcf,'fig\experiment_result1.eps','epsc');

% 
fig=figure(2);
set(gcf,'color','w');
tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');
nexttile
hold off
plot3(test1_x(:,1), test1_x(:,2),test1_x(:,3),'-k','LineWidth',1.5')
hold on;
% plot3(test2_x(:,1), test2_x(:,2),test2_x(:,3),'-r','LineWidth',1.5')
plot3(ref_x(1:length(test1_x),1), ref_x(1:length(test1_x),2),ref_x(1:length(test1_x),3),'-r','LineWidth',1.5')
ax=gca;
scale = 0.075;
axis([ax.XLim(1)-scale ax.XLim(2)+scale ax.YLim(1)-scale ax.YLim(2)+scale ax.ZLim(1)-scale ax.ZLim(2)+scale]);
grid on;
xlabel('x_1(m)');
ylabel('x_2(m)');
zlabel('x_3(m)');

saveas(gcf,'fig\experiment_result2.eps','epsc');