% Copyright (C) 2021 Electronics and Telecommunications Research Institute(ETRI). All rights reserved.
% Authors:     Seonghyeon Jo <seonghyeonjo@etri.re.kr>
%
% Date:        Ang, 10, 2021
% 
% -------------------------------------------------
% Compare to gail and lstm
% 
% -------------------------------------------------
%
% the following code has been tested on Matlab 2020b
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
    folder_name = "0. raw_data\task1\[20211018-";
    timeline = {"1332","1333","1334","1335","1336"};
elseif (task_index == 2)
    task_folder = "task2";
    folder_name = "0. raw_data\task2\[20211022-";
    timeline = {"1404","1405","1407","1409","1410"};
elseif (task_index == 3)
    task_folder = "task3";
    demo_file_name = "train_data_45_0.05";
    folder_name = "0. raw_data\task3\[20211213-";
    timeline = {"185443","185530","185622","185709","185754"};
end

table_demo = table2array(readtable("1. preprocessing_Data/"+task_folder+"/"+demo_file_name+".csv"));
table_lstm = table2array(readtable("2. learning_data\"+task_folder+"\feature_data_lstm_test_data.csv"));
table_lstm(1,:) = [];


len = length(table_demo)/5;
% Plotting
figure(1)
tiledlayout(3,3,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
ylabel_name = {"P_x(m)", "P_y(m)", "P_z(m)", "R_x", "R_y", "R_z", "R_w", "F_x(N)"};
for i=1:8
    ax = nexttile;
    hold off
    plot(table_demo(1:len,i),':k','LineWidth',1.5')
    a(1) = sum((table_demo(1:len,i) - table_lstm(5:len+4,i)).^2);
    
    temp = abs((table_demo(1:len,i) - table_lstm(5:len+4,i))./table_demo(1:len,i));
    temp = temp(isfinite(temp));
    b(1) = mean(temp);
    
    hold on
    for d=2:5
        plot(table_demo(len*(d-1)+1:len*d,i),':k','LineWidth',1.5')
        a(d) = sum((table_demo(len*(d-1)+1:len*d,i) - table_lstm(5:len+4,i)).^2);
        temp = abs((table_demo(len*(d-1)+1:len*d,i) - table_lstm(5:len+4,i))./table_demo(len*(d-1)+1:len*d,i));
        temp = temp(isfinite(temp));
        b(d) = mean(temp);
    end
    
    plot(table_lstm(4:len+4,i),'-r','LineWidth',1')
    rmse(i) = sqrt(sum(a)/(len*5));
    mape(i) = sum(b)*100/5;
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel(ylabel_name{i}, 'FontSize', 10);
    grid on;
end
if (fig_index == 1)
    saveas(gcf,"fig\learning_result3" + fig_type, 'epsc');
else
    saveas(gcf,"fig\learning_result3" + fig_type);
end

fig=figure(2);
set(gcf,'color','w');
tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');
nexttile
hold off
plot3(table_demo(1:len,1), table_demo(1:len,2), table_demo(1:len,3),':k','LineWidth',1.5')
hold on
 for d=1:5
     plot3(table_demo(len*(d-1)+1:len*d,1), table_demo(len*(d-1)+1:len*d,2), table_demo(len*(d-1)+1:len*d,3),':k','LineWidth',1.5')
 end
 
plot3(table_lstm(5:len+4,1), table_lstm(5:len+4,2),table_lstm(5:len+4,3),'-R','LineWidth',1.5')
ax = gca;
r = 0.075;
axis([ax.XLim(1)-r ax.XLim(2)+r ax.YLim(1)-r ax.YLim(2)+r ax.ZLim(1)-r ax.ZLim(2)+r])
xlabel('P_x(m)','FontSize', 12);
ylabel('P_y(m)','FontSize', 12);
zlabel('P_z(m)','FontSize', 12);
grid on;
if (fig_index == 1)
    saveas(gcf,"fig\learning_result4" + fig_type, 'epsc');
else
    saveas(gcf,"fig\learning_result4" + fig_type);
end