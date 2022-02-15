%%
close all; clear; clc;

%% cd path
path =  'C:\Users\user\Desktop\trajectory_planning_ws';
cd(path)

%% Load data
t = textread("t.txt");
CarPosCmd = textread("CarPosCmd_list.txt");
JointPosCmd = textread("JointPosCmd_list.txt");
CarVelCmd = textread("CarVelCmd_list.txt");
JointVelCmd = textread("JointVelCmd_list.txt");
CarAccCmd = textread("CarAccCmd_list.txt");
JointAccCmd = textread("JointAccCmd_list.txt");

%% lebel
CarLabel = {'X', 'Y', 'Z', 'Yaw(z)', 'Pitch(y)', 'Roll(x)'};
JointLabel = {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'};

%% 
joint_PosMax = [ 6.28, 2.35, 2.61, 6.28, 2.56, 6.28 ];
joint_PosMin = [ -6.28, -2.35, -2.61, -6.28, -2.56, -6.28 ];
joint_VelMax = [ 1.57, 1.57, 1.57, 1.57, 1.57, 1.57 ];
joint_AccMax = [ 1.57*3, 1.57*3, 1.57*3, 1.57*3, 1.57*3, 1.57*3  ];

%% plot CarPosCmd
figure('Name' , 'Cartesian Position Cmd'),
for i = 1:6
    subplot(2,3,i);
    plot(t, CarPosCmd(:,i) , 'LineWidth' , 1.2);
    title(CarLabel{i})
    grid on;
    axis tight
end
%% plot JointPosCmd
figure('Name' , 'Joint Position Cmd'),
for i = 1:6
    subplot(2,3,i);
    plot(t, JointPosCmd(:,i) , 'LineWidth' , 1.2); hold on
    y_cons_max = linspace(joint_PosMax(i), joint_PosMax(i), length(t));
    y_cons_min = linspace(joint_PosMin(i), joint_PosMin(i), length(t));
    plot(t, y_cons_max, 'r--' , 'LineWidth' , 1.2);
    plot(t, y_cons_min, 'r--' , 'LineWidth' , 1.2);
    xlim([t(1)-3 t(end)+3])
    title(JointLabel{i})
    grid on;
end
%% CarVelCmd
figure('Name' , 'Cartesian Velocity Cmd'),
for i = 1:6
    subplot(2,3,i);
    plot(t, CarVelCmd(:,i) , 'LineWidth' , 1.2);
    title(CarLabel{i})
    grid on;
    axis tight
end
%% JointVelCmd
figure('Name' , 'Joint Velocity Cmd'),
for i = 1:6
    subplot(2,3,i);
    plot(t, JointVelCmd(:,i) , 'LineWidth' , 1.2); hold on
    y_cons = linspace(joint_VelMax(i), joint_VelMax(i), length(t));
    plot(t, y_cons, 'r--' , 'LineWidth' , 1.2);
    plot(t, -y_cons, 'r--' , 'LineWidth' , 1.2);
    xlim([t(1)-3 t(end)+3])
    title(JointLabel{i})
    grid on;
end
%% CarAccCmd
figure('Name' , 'Cartesian Acceleration Cmd'),
for i = 1:6
    subplot(2,3,i);
    plot(t, CarAccCmd(:,i) , 'LineWidth' , 1.2);
    title(CarLabel{i})
    grid on;
    axis tight
end
%% JointAccCmd
figure('Name' , 'Joint Acceleration Cmd'),
for i = 1:6
    subplot(2,3,i);
    plot(t, JointAccCmd(:,i) , 'LineWidth' , 1.2); hold on
    y_cons = linspace(joint_AccMax(i), joint_AccMax(i), length(t));
    plot(t, y_cons, 'r--' , 'LineWidth' , 1.2);
    plot(t, -y_cons, 'r--' , 'LineWidth' , 1.2);
    xlim([t(1)-3 t(end)+3])
    title(JointLabel{i})
    grid on;
    axis tight
end

%% 3D Trajectory plot
figure('Name' , '3D Trajectory plot')
plot3(CarPosCmd(1:10:end , 1) , CarPosCmd(1:10:end,2) , CarPosCmd(1:10:end,3) , "LineWidth" , 5)
title("Trajectory plot")
xlim([min(CarPosCmd(:,1))-100 max(CarPosCmd(:,1))+100]), xlabel("X")
ylim([min(CarPosCmd(:,2))-100 max(CarPosCmd(:,2))+100]), ylabel("Y")
zlim([min(CarPosCmd(:,3))-100 max(CarPosCmd(:,3))+100]), zlabel("Z")
grid on