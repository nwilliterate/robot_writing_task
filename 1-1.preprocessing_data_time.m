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


% data load
real_x_max = -9999*ones(8,1);
real_x_min = 9999*ones(8,1);
for d =1:5
    real_car_quat = table2array(readtable(folder_name+timeline{d}+"]franka_data_cartesian_quat.csv"));
    real_force = table2array(readtable(folder_name+timeline{d}+"]franka_data_force_sensor.csv"));
    real_x{d} = [real_car_quat real_force(:,1)];
    
    for i=1:8
        if (real_x_max(i) < max(real_x{d}(:,i)))
            real_x_max(i) = max(real_x{d}(:,i));
        end
        if (real_x_min(i) > min(real_x{d}(:,i)))
            real_x_min(i) = min(real_x{d}(:,i));
        end
    end
end

% normalization
for d =1:5
    for i=1:8
        real_x_nor{d}(:,i) = (real_x{d}(:,i) - real_x_min(i)) / (real_x_max(i) - real_x_min(i));
    end
end

% 
for d = 1:5
sample_size = length(real_x_nor{d});
temp_t = [1];
temp_x = round(real_x_nor{d}(1,:),3);
num = 1;
threshold = 0.003;
for i=1:sample_size
    if(norm(temp_x(num,1:8) - real_x_nor{d}(i,1:8)) > threshold) 
        num = num + 1;
        temp_t(num) = i;
        temp_x(num,:) = round(real_x_nor{d}(i,:),3);
    end
end
temp_xx{d} = temp_x;
end

max_length = 0;
for d=1:5
    if (max_length < length(temp_xx{d}))
        max_length = length(temp_xx{d});
    end
end

% Resampling
save_table = [];
for d=1:5
    t = 1:length(temp_xx{d});
    t = t/length(temp_xx{d})*max_length;
    
    [p,q] = rat(1 / (1/(t(2)-t(1))));
    for i=1:8
        preprocessing_demo{d}(:,i) = round(resample(temp_xx{d}(:,i), p, q),3); 
    end
    save_table = [save_table; preprocessing_demo{d}];
    
end
temp_x = save_table;
% Save data
T = table(temp_x);
round(sample_size/num, 0)
file_name = "1. preprocessing_data\"+task_folder+"\train_data_"+ round(sample_size/num,0) + "_" + threshold+ ".csv"
writetable(T,file_name,'Delimiter',',');

% Plotting
figure(1)
set(gcf,'color','w');
tiledlayout(3,3, 'TileSpacing','Compact','Padding','Compact');
for i=1:8
    ax = nexttile;
    hold off
    hold on;
    for j = 1:5
        plot(temp_xx{j}(:,i),':k' ,'LineWidth',1.5')
    end
    grid on;
end

figure(2)
set(gcf,'color','w');
tiledlayout(3,3, 'TileSpacing','Compact','Padding','Compact');
plotline = {'-m','-r','-g','-b','-k'};
ylabel_name = {"P_x(m)", "P_y(m)", "P_z(m)", "R_x", "R_y", "R_z", "R_w", "F_x(N)"};
for i=1:8
    ax = nexttile;
    hold off;
    for d = 1:5
        if d~=1 
            hold on;
        end
        plot(preprocessing_demo{d}(:,i) ,plotline{d} ,'LineWidth',1')
    end
    grid on;
    ylabel(ylabel_name{i});
    xlabel('time(s)');
end

figure(3)
set(gcf,'color','w');
tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');
hold off;
for i =1:5
    plot3(preprocessing_demo{i}(:,1),preprocessing_demo{d}(:,2),preprocessing_demo{d}(:,3), plotline{i},'LineWidth',1.5)
    hold on;
    grid on;
end
xlabel('P_x(m)');
ylabel('P_y(m)');
zlabel('P_z(m)');
saveas(gcf,'fig\teaching_result1.eps','epsc');


% figure(4)
% set(gcf,'color','w');
% tiledlayout(3,3, 'TileSpacing','Compact','Padding','Compact');
% for i=1:8
%     ax = nexttile;
%     hold off
%     hold on;
%     for d = 1:5
%         t = 1:length(real_x_nor{d}(:,i));
%         t = t/length(real_x_nor{d}(:,i))*max_length;
%         plot(t, real_x_nor{d}(:,i),'LineWidth',1.5')
%     end
%     grid on;
% end
% 
