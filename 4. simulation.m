% Copyright (C) 2021 All rights reserved.
% Authors:      Seonghyeon Jo <seonghyeonjo@etri.re.kr>
%
% Date:         Oct, 18, 2021
% Last Updated: Feb, 04, 2022
%
% -------------------------------------------------
% Hybrid Imdenance Controler
% Franka Emika Robot
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
end

% reference joint catersian
% ref_x = table2array(readtable("3. trajectory_data\"+task_folder+"\trajectory_pose.csv"));
% ref_f = table2array(readtable("3. trajectory_data\"+task_folder+"\trajectory_force.csv"));
ref_x = table2array(readtable("3. trajectory_data\"+task_folder+"\trajectory_pose_20s.csv"));
ref_f = table2array(readtable("3. trajectory_data\"+task_folder+"\trajectory_force_20s.csv"));

% simulation setting
sample_size = length(ref_x);
sim_period = 0.001;
t = (1:sample_size)*sim_period;
sample_size = size(t, 2);
sim_time= (sample_size)*sim_period;

% reference joint euler
eul =  [];
for i=1:sample_size
    R = quat2rotm([ref_x(i,7) ref_x(i,4) ref_x(i,5) ref_x(i,6)]);
    z1 = atan2(R(2,3),R(1,3));
    y = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    z2 = atan2(R(3,2), - R(3,1)); 
    eul(i,:) = [z2 y z1]';
end
traj_x = [ref_x(:,1:3) eul];
traj_f = ref_f;

% Impedance Gain
Md = diag([1 1 1 1 1 1]'); 
Kd = 0.3*diag([250 250 250 200 200 200]')*10;
Bd = sqrt(Kd)*1;

% ext_force
f_ext = zeros(sample_size, 6);

init_q = [0.000168779	-0.785313	2.96143e-05	-2.35664	-0.000253931	3.14146	0.7852]';
x = [init_q; zeros(7,1)];

% control loop
for i=1:sample_size
    q = x(1:7,i);
    qd = x(8:14,i);
    
    % Franka Kinematics
    % joint space -> task space(euler)
    [p, R]=get_pose(q);
    
    z1 = atan2(R(2,3),R(1,3));
    y = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    z2 = atan2(R(3,2), - R(3,1)); 
    eul = [z2 y z1]';
    quat(i,:) = rotm2quat(R);
    car_pos(i,:) = [p; eul];
    
    % Model
    M = get_MassMatrix(q);
    J = get_JacobianZ2YZ1(q);
    J_dot = get_Jacobian_dot(q, qd);
    
    % Cartesian Error
    e = (traj_x(i,:) - car_pos(i,:))';
    e_dot = - J*qd;
    
    % model feedback linearization 
    % Computing time too long
    % c = get_CoriolisVector(q, qd); 
    % G = get_GravityVector(q);
    % F = get_FrictionTorque(qd);
    
    % input
    % position
    % u = M*pinv(J)*(ref_xdd(i,:)+inv(Md)*Bd*(e_dot)+inv(Md)*Kd*e+inv(Md)*f_ext(i,:)'-J_dot*qd)-J'*f_ext(i,:)';   % use simple_rk and simple_plant
    
%     if (car_pos(i,1) > 0.413)
%        f_ext(i,1) = 120*(0.413-car_pos(i,1));
%     end
    
    % force/position
    u = M*pinv(J)*(inv(Md)*Bd*(e_dot)+inv(Md)*Kd*e+inv(Md)*f_ext(i,:)'-inv(Md)*traj_f(i,:)'-J_dot*qd)-J'*f_ext(i,:)';   % use simple_rk and simple_plant
    % u = M*pinv(J)*(ref_xdd(i,:)+inv(Md)*Bd*(e_dot)+inv(Md)*Kd*e-inv(Md)*f_ext(i,:)'-J_dot*qd)-J'*f_ext(i,:)';    % use rk and plant

    if(i ~= sample_size)
        % rk
        x(:,i+1) = simple_rk(x(:,i), u, sim_period);
        % x(:,i+1) = rk(x(:,i), u, sim_period);
        
        % int
        % x(:, i+1) = x(:, i) + 0.001*(simple_plant(x(:,i), u));
        % x(:, i+1) = x(:, i) + 0.001*(plant(x(:,i), u));
    end
end

% Plotting
% Comparison with the desired Cartesian
fig = figure(1);
% fig.Position = [0 0 780 1000]; 
tiledlayout(3,3,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');

ylabel_name = {"P_x(m)", "P_y(m)", "P_z(m)", "R_x", "R_y", "R_z", "R_w", "F_x(N)"};
for i=1:3
    ax = nexttile;
    hold off
    plot(t, traj_x(:,i),'-k','LineWidth',1.5')
    hold on;
    plot(t, car_pos(:,i),'-r','LineWidth',1.5')
    ylim([ax.YLim(1)-0.025  ax.YLim(2)+0.025])
    xlim([0 sim_time])
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel(ylabel_name{i}, 'FontSize', 10);
    grid on;
    ylim([ax.YLim(1)-0.05 ax.YLim(2)+0.05])
end

for i=4:7
    ax = nexttile;
    hold off
    plot(t, ref_x(:,i),'-k','LineWidth',1.5')
    hold on;
    if(i==7)
        plot(t, -quat(:,1),'-r','LineWidth',1.5')
    else
        plot(t, -quat(:,i-2),'-r','LineWidth',1.5')
    end
    xlim([0 sim_time])
    grid on;
    ylim([ax.YLim(1)-0.05 ax.YLim(2)+0.05])
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel(ylabel_name{i}, 'FontSize', 10);
end
ax = nexttile;
hold off;
plot(t, traj_f(:,1),'-k','LineWidth',1.5')
hold on;
plot(t, f_ext(:,1),'-r','LineWidth',1.5')
grid on
xlabel('Time (sec)', 'FontSize', 10)
ylabel(ylabel_name{8}, 'FontSize', 10);
ylim([ax.YLim(1)-2 ax.YLim(2)+2])
xlim([0 sim_time])
lgd = legend('xd', 'x');
lgd.Layout.Tile = 9;
lgd.FontSize = 10;

if (fig_index == 1)
    saveas(gcf,"fig\simulation_result1" + fig_type, 'epsc');
else
    saveas(gcf,"fig\simulation_result1" + fig_type);
end

% 3D Cartesian Pose Plot
fig=figure(2);
set(gcf,'color','w');
% fig.Position = [0 0 500 500]; 
tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');
nexttile
hold off
plot3(traj_x(:,1), traj_x(:,2),traj_x(:,3),'-k','LineWidth',1.5')
hold on
% plot3(traj_x(1,1), traj_x(1,2),traj_x(1,3),'or','LineWidth',1.5')
% plot3(traj_x(sample_size,1), traj_x(sample_size,2),traj_x(sample_size,3),'ob','LineWidth',1.5')
plot3(car_pos(:,1), car_pos(:,2),car_pos(:,3),'-r','LineWidth',1.5')
% legend('ref x','start point', 'end point', 'cur x')
ax = gca;
r = 0.075;
axis([ax.XLim(1)-r ax.XLim(2)+r ax.YLim(1)-r ax.YLim(2)+r ax.ZLim(1)-r ax.ZLim(2)+r])
% axis([-0. 1 -0.5 0.5 0.1 1]);
% axis([-0. 1 -0.5 0.5 0.1 1]);

xlabel('P_x(m)','FontSize', 12);
ylabel('P_y(m)','FontSize', 12);
zlabel('P_z(m)','FontSize', 12);
grid on;
lgd = legend('xd', 'x','Location','northeast');

if (fig_index == 1)
    saveas(gcf,"fig\simulation_result2" + fig_type, 'epsc');
else
    saveas(gcf,"fig\simulation_result2" + fig_type);
end